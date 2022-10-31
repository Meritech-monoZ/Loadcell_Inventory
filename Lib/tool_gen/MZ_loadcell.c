/*
 * MZ_loadcell.c
 *
 *  Created on: 22-Jun-2022
 *      Author: Piyush
 */

#include "MZ_loadcell.h"
#include "main.h"
#include "MZ_error_handler.h"
#include "MZ_timer.h"
#include "MZ_print.h"
#include "MZ_Mqtt_public.h"
#include "MZ_Modem_public.h"
#include "MZ_main.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"

/* Define some common use MACRO - START */
#define LOADCELL_READ_TIMER_EXPIRE_SET			(1)						///< Set the Timer Expire for load cell read
#define LOADCELL_READ_TIMER_EXPIRE_CLEAR		(0)						///< Clear the Timer Expire for load cell read
#define TIME_180SEC								(pdMS_TO_TICKS(180000))	///< Timer is set for 180 seconds
#define TIME_90SEC								(pdMS_TO_TICKS(90000))	///< Timer is set for 90 seconds
#define LOADCELL_DATA_SEND_TIME					(pdMS_TO_TICKS(10000))	///< Timer is set for 10 seconds
#define TIMER_ID_CLEAR							(0)						///< Clear the timer id
/* Define some common use MACRO - END */

static size_t loadcell_data_timer_id = TIMER_ID_CLEAR;					/*!< load cell timer id - Initialize it to 0 */

#define LOADCELL_READ_TIME		 		(TIME_90SEC)						//< Set 90 seconds timer for read sensor data
static char loadcell_read_timer_expire_flag = LOADCELL_READ_TIMER_EXPIRE_CLEAR;	/*!< Flag is created and cleared for sensor data read */

/* Thread related MACRO and variables - START */
#define LOADCELL_APP_STACK_SIZE			(1024)										/*!< Stack size for the thread */
static mz_thread_t 					    loadcell_thread_id = NULL;					/*!< Thread id handler */
static StaticTask_t 					loadcell_cb_mem;							/*!< Thread control block */
static StackType_t 						loadcell_stack[LOADCELL_APP_STACK_SIZE];	/*!< Thread stack */

/* Thread related MACRO and variables - END */

/* loadcell MACRO - START */

#define LOAD_CELL_COEF					440
#define REF_LOAD_FACTOR					2

#define LOADCELL_WEIGHT_SIZE 			40
#define OBJECT_QTY_SIZE 				20
#define REFERENCE_WEIGHT_SIZE 			20
#define INVENTORY_CHANGE_SIZE 			5

#define PAYLOAD_STRING_SIZE 			255

#define LOADCELL_TARE_SAMPLES 			5
#define LOADCELL_WEIGHT_SAMPLE 			10

#define FLAG_SET 						1
#define FLAG_CLEAR 						0

#define SET								1
#define CLEAR						 	0

#define LOADCELL_VALUE_READ_TIMEOUT 	150
#define DATA_READ_CYCLES 				24
#define OUTPUT_DATA_MIN_RANGE 			0x800000

#define SWITCH_PRESS_CNTR 				500
#define SHRT_PRESS_MIN_CNTR 			5
#define SHRT_PRESS_MAX_CNTR 			450

#define INVENTORY_CHANGE_SET 			1
#define INVENTORY_CHANGE_CLEAR 			0

#define DUMMY_DELAY_5US 				5
#define DELAY_1MS						1
#define DELAY_2MS 						2
#define DELAY_5MS 						5
#define DELAY_10MS 						10
#define DELAY_200MS 					200
#define DELAY_400MS 					400
#define DELAY_2000MS 					2000

#define LED_TOGGLE_COUNTS 				6
#define MEAS_WEIGHT_BUF_SIZE 			40
#define OBJECT_QTY_BUF_SIZE 			20
#define REF_WEIGHT_BUF_SIZE 			20
#define INVENTORY_CHANGE_BUF_SIZE 		5

#define INIT_0    						0
#define FLOAT_0    						0.0

/* loadcell MACRO - END */

/* load cell Global Variables and Buffer - START */

int8_t shrtpress = INIT_0;
int8_t longpress = INIT_0;
int32_t switchCntr = INIT_0;

volatile int32_t measValue = INIT_0;
volatile int32_t refernceWeight = INIT_0;
volatile int32_t measWeight = INIT_0;

static int64_t preWeight = INIT_0;
static int8_t dataTxReady = INIT_0;  		//transmit flag

static char  loadCellWeight[LOADCELL_WEIGHT_SIZE] = {0};		/*!< Store load cell weight value after calculation from raw value and decimal place */
static char  objectQty[OBJECT_QTY_SIZE] 		= {0};			/*!< Store object quantity value after calculation from raw value and decimal place */
static char  referenceWeight[REFERENCE_WEIGHT_SIZE] = {0};		/*!< Store reference weight value after calculation from raw value and decimal place */
static char  inventory_change[INVENTORY_CHANGE_SIZE] = {0};		/*!< Store inventory change value after calculation from raw value and decimal place */

/* load cell Global Variables and Buffer - END */

loadcell_t loadcell_d;


/* MQTT related MACRO and variables - START */
static char payload_string[PAYLOAD_STRING_SIZE] = "";				/*!< payload string buffer to pass the final payload information to MonoZ_Lib */
static st_mqtt_message pmsg;
/* MQTT related MACRO and variables - END */

static void loadcell_read_timer_cb(TimerHandle_t xTimer);
static void create_mqtt_payload(st_mqtt_message * pmsg , char * buff);
static void send_payload_to_server(st_mqtt_message * pmsg);
static void loadcell_app_thread(void * arg);

#define MZ_MQTT_PUB_TOPIC 		"\"v1/devices/me/telemetry\""
#define MZ_MQTT_PUB_QOS			MQTT_QOS0
#define MZ_MQTT_SUB_TOPIC		"\"v1/devices/me/attributes\""
#define MZ_MQTT_SUB_QOS			MQTT_QOS2
#define MZ_IMS_WT				"weight"
#define MZ_IMS_OBJ				"Object_Qty"
#define MZ_IMS_WTQT				"WT/Qty"
#define MZ_IMS_IC				"Inventory_Change"

/** @fn static void loadcell_read_timer_cb(TimerHandle_t xTimer)
 * @brief load cell reading timer callback - START
 * This Timer callback will be called after the load cell reading timer is
 * expired.The timer value is set as LOADCELL_READ_TIME
 * @param xTimer TimerHandle_t
 * @retval None
 */
static void loadcell_read_timer_cb(TimerHandle_t xTimer)
{
	/* Set the load cell timer expire flag */
	loadcell_read_timer_expire_flag = LOADCELL_READ_TIMER_EXPIRE_SET;
}
/* load cell reading timer callback - END */

/** @fn static void loadcell_data_timer_cb(TimerHandle_t xTimer)
 * @brief loadcell_data timer callback - START
 * This Timer callback will be called after the loadcell_data timer is
 * expired.The timer value is set as LOADCELL_DATA_SEND_TIME
 * @param xTimer TimerHandle_t
 * @retval None
 */
static void loadcell_data_timer_cb(TimerHandle_t xTimer)
{
	dataTxReady = FLAG_SET;

	/* Print when the application is ready for data transmission */
	mz_puts("Ready for data Transmission\r\n");
}

/** @fn static void create_mqtt_payload(void)
 * @brief MQTT Create payload API - START
 * This API will be used to create the payload string/buffer from load cell final
 * value received after processing of raw value.
 */
static void create_mqtt_payload(st_mqtt_message * pmsg , char * buff)
{
	sprintf(buff,"{\"%s\":%s,\"%s\":%s,\"%s\":%s,\"%s\":%s}%c",MZ_IMS_WT, loadCellWeight, MZ_IMS_OBJ, objectQty, MZ_IMS_WTQT, referenceWeight, MZ_IMS_IC, inventory_change, 26);

	pmsg->topic = MZ_MQTT_PUB_TOPIC;
	pmsg->qos = MZ_MQTT_PUB_QOS;
	pmsg->retain = MQTT_RETAIN_OFF;
	pmsg->message = buff;
}
/* MQTT Create payload API - END */

/** @fn static void send_payload_to_server(void)
 * @brief MQTT send payload API - START
 * This API will be used to send the payload string/buffer to MonoZ_Lib.
 * It will also print if the sending of payload to MonoZ_Lib was successful or
 * any error occurred
 */
static void send_payload_to_server(st_mqtt_message * pmsg)
{
	/* Set the mqtt Data and send to MonoZ_Lib */
	//mz_error_t status = mz_mqtt_pub(pmsg);

	mz_error_t status = MZ_init_cmd_direct("AT+QMTDISC=0\r\n", AT_TIME_15SEC, AT_TIME_15SEC);
	status |= MZ_init_cmd_direct("AT+QMTOPEN=0,\"cloud.monoz.io\",1883\r\n", AT_TIME_15SEC, AT_TIME_15SEC);
	status |= MZ_init_cmd_direct("AT+QMTCONN=0,\"weighing_scale\",\"MZ_IM_System\",\"meritech123\"\r\n", AT_TIME_15SEC, AT_TIME_15SEC);
	status |= MZ_init_cmd_direct("AT+QMTPUB=0,0,0,0,\"v1/devices/me/telemetry\"\r\n", AT_TIME_15SEC, 0);
	status |= MZ_init_cmd_direct(payload_string, AT_TIME_15SEC, AT_TIME_15SEC);

	/* Check the status of the request */
	if(MZ_OK == status)
	{
		/* print success on CLI */
		mz_puts("Data send to MonoZ_Lib\r\n");
	}
	else
	{
		/* print of error string on CLI */
		mz_puts("Data send to MonoZ_Lib FAILED\r\n");
		//mz_puts(mz_error_to_str(status));
	}
}
/* MQTT send payload API - END */

/** @fn static void HAL_Delay_us(uint32_t delay_t)
 * @brief HAL_Delay_us - START
 * We use this when we need dummy delay.
 * @param uint32_t delay_t
 * @retval None
 */
void HAL_Delay_us(uint32_t delay_t)
{
	uint32_t delay = delay_t;
	while (delay > 0)
	{
		delay--;
		__NOP();
	}
}
/* HAL_Delay_us - END */

/** @fn void loadcell_init(loadcell_t loadcell)
  * @brief loadcell_init - START
  * @param loadcell_t load cell
  * This is used to initialize the load cell and LED configuration.
  * @retval None
  */
void loadcell_init(loadcell_t loadcell)
{
	__HAL_RCC_GPIOG_CLK_ENABLE();

	loadcell_d.clk_gpio_port = GPIOG;
	loadcell_d.dat_gpio_port = GPIOG;
	loadcell_d.clk_pin = GPIO_PIN_2;
	loadcell_d.dat_pin = GPIO_PIN_1;
	loadcell_d.offset = CLEAR;
	loadcell_d.coef = CLEAR;

	GPIO_InitTypeDef  gpio = {0};

	gpio.Pin = loadcell_d.clk_pin;   //load cell clk pin
	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pull = GPIO_PULLUP;
	gpio.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(loadcell_d.clk_gpio_port, &gpio);

	gpio.Pin = loadcell_d.dat_pin;   //load cell data pin
	gpio.Mode = GPIO_MODE_INPUT;
	gpio.Pull = GPIO_PULLUP;
	gpio.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(loadcell_d.dat_gpio_port, &gpio);

	gpio.Pin = tareSw;  		 	 // tare and ref load selection switch
	gpio.Mode = GPIO_MODE_IT_FALLING;
	gpio.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOG, &gpio);

	gpio.Pin = refLoadLed;  		 // LED pin
	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pull = GPIO_PULLDOWN;
	gpio.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOG, &gpio);

	HAL_GPIO_WritePin(loadcell_d.clk_gpio_port, loadcell_d.clk_pin, GPIO_PIN_SET);
	HAL_Delay_us(DUMMY_DELAY_5US);
	HAL_GPIO_WritePin(loadcell_d.clk_gpio_port, loadcell_d.clk_pin, GPIO_PIN_RESET);
	HAL_Delay_us(DUMMY_DELAY_5US);

	loadcell_value(loadcell);
}
/* loadcell_init - END */

/** @fn int32_t loadcell_value(loadcell_t loadcell))
  * @brief loadcell_init - START
  * @param loadcell_t load cell
  * This is used to calculate the load cell Value.
  * @retval load cell Value
  */

int32_t loadcell_value(loadcell_t loadcell)
{
	int32_t data = INIT_0;
	uint32_t  startTime = HAL_GetTick();

	/* Sense the data pin, when its goes
     * to low, it indicates data is ready for retrieval
     */
	while(HAL_GPIO_ReadPin(loadcell_d.dat_gpio_port, loadcell_d.dat_pin) == GPIO_PIN_SET)
	{
		HAL_Delay(DELAY_1MS);
		if(HAL_GetTick() - startTime > LOADCELL_VALUE_READ_TIMEOUT)
			break;
        else {} // Default waiting case.
	}

	/*  Applying 25 positive clock pulses at the clock pin,
	 *  data is shifted out from the data pin.
	 *  Each clock pulse shifts out one bit,
	 *  starting with the MSB bit first,
	 *  until all 24 bits are shifted out.
	 *  The 25th pulse at clock input will pull data pin back to high.
	 */
	for(int8_t readData = 0; readData < DATA_READ_CYCLES; readData++)
	{
		HAL_GPIO_WritePin(loadcell_d.clk_gpio_port, loadcell_d.clk_pin, GPIO_PIN_SET);
		HAL_Delay_us(DUMMY_DELAY_5US);
		HAL_GPIO_WritePin(loadcell_d.clk_gpio_port, loadcell_d.clk_pin, GPIO_PIN_RESET);
		HAL_Delay_us(DUMMY_DELAY_5US);
		data = data << 1;

		/* Check the status of the data pin, if high then increment data by 1 */
		if(HAL_GPIO_ReadPin(loadcell_d.dat_gpio_port, loadcell_d.dat_pin) == GPIO_PIN_SET)
		  data ++;
        else {} // Default waiting case.
	}
	HAL_GPIO_WritePin(loadcell_d.clk_gpio_port, loadcell_d.clk_pin, GPIO_PIN_SET);
	HAL_Delay_us(DUMMY_DELAY_5US);
	HAL_GPIO_WritePin(loadcell_d.clk_gpio_port, loadcell_d.clk_pin, GPIO_PIN_RESET);
	HAL_Delay_us(DUMMY_DELAY_5US);

	data = (data ^ OUTPUT_DATA_MIN_RANGE);

	return data;
}
/* loadcell_value - END */

/** @fn void loadcell_tare(loadcell_t loadcell, uint16_t sample)
  * @brief loadcell_tare - START
  * @param loadcell_t loadcell, uint16_t sample
  * This is used to tare the load cell weight.
  * @retval None
  */
void loadcell_tare(loadcell_t loadcell, uint16_t sample)
{
	int32_t avg = INIT_0;

	/*Takes the samples of the load cell value for tare*/
	for(uint16_t noSample = 0; noSample < sample; noSample++)
	{
		avg += loadcell_value(loadcell_d);
		HAL_Delay(DELAY_5MS);
	}
	loadcell_d.offset = (int32_t)(avg / sample);
}
/* loadcell_tare - END */

/** @fn int32_t loadcell_weight(loadcell_t loadcell, uint16_t sample)
  * @brief loadcell weight calculator - START
  * @param loadcell_t loadcell, uint16_t sample
  * This is used to convert load cell value to weight.
  * @retval load cell weight
  */
int32_t loadcell_weight(loadcell_t loadcell, uint16_t sample)
{
	int32_t weight = INIT_0;
	int32_t avg = INIT_0;

	/*Takes the samples of the load cell value for calculate the weight*/
	for(uint16_t noSample = 0; noSample < sample; noSample++)
	{
		avg += loadcell_value(loadcell_d);
		HAL_Delay(DELAY_2MS);
	}
	int32_t data = (int32_t)(avg / sample);
	weight =  (data - loadcell_d.offset) / LOAD_CELL_COEF;
	return weight;
}
/* loadcell_weight - END */

/** @fn static void loadcell_app_thread(void * arg)
  * @brief load cell main Application thread.  START
  * @param void * arg
  *     It creates all the timer
  *     load cell reading timer,
  * 	load cell data timer
  *     It calculates the final count of the object and sends it to the MQTT server.
  * @retval None
  */
static void loadcell_app_thread(void * arg)
{
	(void)arg;

	int8_t refLdSelected = INIT_0, inventChange = INIT_0;
	int32_t noWeightCnt = INIT_0, lastWeightInc = INIT_0;
	int32_t lastWeightDec = INIT_0, remWeightCnt = INIT_0;

	char measWeightBuf[MEAS_WEIGHT_BUF_SIZE] = {0};
	char objectQtyBuf[OBJECT_QTY_BUF_SIZE] = {0};
	char refWeightBuf[REF_WEIGHT_BUF_SIZE] = {0};
	char inventChangeBuf[INVENTORY_CHANGE_BUF_SIZE] = {0};

	float refLdTole = FLOAT_0;
	/*
	 * create the loadcell reading timer.
	 * As per requirement, We are creating a recursive timer using
	 * mz_tm_create_start_recursive() API.
	 * In-case of other type of timer, please refer MZ_timer.h
	 * The expire time for this timer set to LOADCELL_READ_TIME.
	 * "loadcell_read_timer_cb" is passed as an argument.
	 * When the timer expires, the "loadcell_read_timer_cb" API will be
	 * processed.
	 */
	if(MZ_OK == mz_tm_create_start_recursive("Read timer",
											LOADCELL_READ_TIME,
											loadcell_read_timer_cb))
	{
		/* print success on CLI */
		mz_puts("loadcell reading timer started\r\n");
	}
    else {} // Default waiting case.

	/* initialize the loadcell configuration */
	loadcell_init(loadcell_d);
	HAL_Delay(DELAY_200MS);

	/* Tare the no load weight */
	loadcell_tare(loadcell_d, LOADCELL_TARE_SAMPLES);

	/*
	 * This is the infinite loop for this thread - the thread will execute this
	 * loop forever and not come outside of this loop
	 */

	while(1)
	{
		/* Need to write the periodic executing logic in this loop block */

		/*
		 * Process the server re-registration based on registration required or
		 * not status - Check if the flag is set
		 */

		/*calculate the load cell Value*/
		measValue = loadcell_value(loadcell_d);

		/*convert load cell value to weight*/
		measWeight = loadcell_weight(loadcell_d, LOADCELL_WEIGHT_SAMPLE);
		measWeight = (int32_t)measWeight;

		 /* Short button press which is using for load Tare */
		if(shrtpress == FLAG_SET)
		{
			shrtpress = FLAG_CLEAR;
			loadcell_tare(loadcell_d, LOADCELL_TARE_SAMPLES);

			// toggle LED (indication of load tare)
			for(int8_t toggCntr = 0; toggCntr < LED_TOGGLE_COUNTS; toggCntr++)
			{
				HAL_GPIO_TogglePin(GPIOG, refLoadLed);
				HAL_Delay(DELAY_400MS);
			}
		}
		/* long button press which is using for reference load selection */
		else if(longpress == FLAG_SET)
		{
			longpress = FLAG_CLEAR;
			if(measWeight != 0)
			{
				//glow LED (indication of reference load is set)
				HAL_GPIO_WritePin(GPIOG, refLoadLed, GPIO_PIN_SET);

				inventChange = INVENTORY_CHANGE_SET;
				refLdSelected = FLAG_SET;

				/*convert load cell value to weight*/
				refernceWeight = loadcell_weight(loadcell_d, LOADCELL_WEIGHT_SAMPLE);
				refernceWeight = (int)refernceWeight;

				sprintf(refWeightBuf, "%ld", refernceWeight);
				strcpy(referenceWeight, refWeightBuf);

				HAL_Delay(DELAY_2000MS);

				//LED Reset after 2 sec
				HAL_GPIO_WritePin(GPIOG, refLoadLed, GPIO_PIN_RESET);
			}
	        else {} // Default waiting case.
		}
        else {} // Default waiting case.

		/* When the reference load is selected */
		if(refLdSelected == FLAG_SET)
		{
			noWeightCnt = (measWeight / refernceWeight);
			remWeightCnt = (measWeight % refernceWeight);

			/*The reference load tolerance for counting the object is set*/
			refLdTole = (float)refernceWeight / REF_LOAD_FACTOR;

			/* Considers 1 item if the remaining weight is greater than the reference load tolerance.*/
			if(remWeightCnt >= refLdTole)
				noWeightCnt++;
	        else {} // Default waiting case.
		}
        else {} // Default waiting case.

		sprintf(measWeightBuf, "%ld", measWeight);
		sprintf(objectQtyBuf, "%ld", noWeightCnt);
		sprintf(inventChangeBuf, "%d", inventChange);

		strcpy(loadCellWeight, measWeightBuf);
		strcpy(objectQty, objectQtyBuf);
		strcpy(inventory_change, inventChangeBuf);

		/* Convert load cell value to weight */
		measWeight = loadcell_weight(loadcell_d, LOADCELL_WEIGHT_SAMPLE);

		lastWeightInc = (measWeight - preWeight);
		lastWeightDec = (preWeight - measWeight);

		/*  The increase and decrease in the previous weight change
		 *  exceeds the reference load tolerance, then "start the
		 *  load cell data once".
		 */
		if(((lastWeightInc >= refLdTole) && (refLdTole != 0)) || ((lastWeightDec >= refLdTole) && (refLdTole != 0)))
		{
			/* Stopping the load cell data one time timer */
			if(loadcell_data_timer_id)
			{
				if(MZ_OK != mz_tm_stop(loadcell_data_timer_id))
				{
					/* print of error string on CLI */
					mz_puts("loadcell data timer stopping failed\r\n");
				}
		        else {} // Default waiting case.
			}
			else
			{
				/* Create the load cell data timer.*/
				loadcell_data_timer_id = mz_tm_create_one("loadcell data timer",
															LOADCELL_DATA_SEND_TIME,
															loadcell_data_timer_cb);
			}

			/* Starting the loadcell data one time timer */
			if(MZ_OK != mz_tm_start(loadcell_data_timer_id))
			{
				/* print of error string on CLI */
				mz_puts("loadcell data timer creation failed\r\n");
			}
	        else {} // Default waiting case.

			preWeight = measWeight;
		}
        else {} // Default waiting case.

		/* Send data to MQTT server */
		if(dataTxReady == FLAG_SET)
		{
			dataTxReady = FLAG_CLEAR;
			/* Stopping the loadcell data one time timer */
			if(MZ_OK != mz_tm_stop(loadcell_data_timer_id))
			{
				/* print of error string on CLI */
				mz_puts("loadcell data timer stopping failed\r\n");
			}
	        else {} // Default waiting case.

			/* Create the payload from the received data */
			create_mqtt_payload(&pmsg, payload_string);

			/* send the payload to mqtt server */
			send_payload_to_server(&pmsg);

			inventChange = INVENTORY_CHANGE_CLEAR;
			lastWeightInc = CLEAR;
			lastWeightDec = CLEAR;
			preWeight = measWeight;
		}
        else {} // Default waiting case.

		/* Put a delay to avoid blocking on this thread */
		HAL_Delay(DELAY_10MS);
	}
	//End of while(1) - Do not place any code after this.
}
/* load cell main Application thread. - END */

/** @fn void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
  * @brief GPIO falling edge interrupt - START
  * @param uint16_t GPIO_Pin
  * This is used to sense the button state.
  * @retval None
  */
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
	/* Read the status of button pin */
	if(GPIO_Pin == tareSw)
	{
		/* Enter in infinite loop */
		while(1)
		{
			HAL_Delay(DELAY_1MS);
			/* Start the counter and break out of loop after switch release or timer expire */
			if(HAL_GPIO_ReadPin(GPIOG, tareSw) == GPIO_PIN_RESET)
				switchCntr++;
			else
				break;

			/* Release when timer expire */
			if(switchCntr > SWITCH_PRESS_CNTR)
				break;
	        else {} // Default waiting case.
		}
	}
    else {} // Default waiting case.

	// shortpress for tare load
	if((switchCntr > SHRT_PRESS_MIN_CNTR) && (switchCntr < SHRT_PRESS_MAX_CNTR))
	{
		shrtpress = FLAG_SET;
		longpress = FLAG_CLEAR;
		switchCntr = CLEAR;
	}
	// longpress for reference load selection
	else if(switchCntr > SHRT_PRESS_MAX_CNTR)
	{
		longpress = FLAG_SET;
		shrtpress = FLAG_CLEAR;
		switchCntr = CLEAR;
	}
    else {} // Default waiting case.
}
/* HAL_GPIO_EXTI_Falling_Callback - END */

/*
 * loadcell Application initialization API. - START
 *
 * 1. It call all necessary initializations before application start
 * 2. It create the main loadcell application
 * 3. It set all variables to its initialization states
 */
mz_error_t loadcell_app_init(void)
{
	mz_error_t _ret = MZ_OK;

	/* Create the loadcell application thread */
	if(!mz_thread_create(	&loadcell_thread_id,
							"loadcell Scheduler",
							loadcell_app_thread,
							NULL,
							osPriorityNormal,
							loadcell_stack,
							LOADCELL_APP_STACK_SIZE,
							&loadcell_cb_mem,
							sizeof(loadcell_cb_mem)))
	{
		_ret = MZ_THREAD_CREATE_FAIL;
	}
    else {} // Default waiting case.

	return _ret;
}
/* loadcell Application initialization API. - START */






