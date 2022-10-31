/*
 * loadcell.h
 *
 *  Created on: 22-Jun-2022
 *      Author: Piyush
 */

#ifndef MZ_LOADCELL_H_
#define MZ_LOADCELL_H_

#include "main.h"
#include "MZ_error_handler.h"


//***********************************// define gpio //**********************************//
#define		tareSw			GPIO_PIN_0
#define		dtTrasmitSw		GPIO_PIN_3
#define		refLoadSel		GPIO_PIN_4
#define		refLoadLed		GPIO_PIN_9


//***********************************// loadcell gpio struct //**********************************//

typedef struct
{
	GPIO_TypeDef  *clk_gpio_port;
	GPIO_TypeDef  *dat_gpio_port;
	uint16_t      clk_pin;
	uint16_t      dat_pin;
	int32_t       offset;
	float         coef;
}loadcell_t;

//************************************// loadcell functions //***********************************//

mz_error_t 	loadcell_app_init(void);

void        loadcell_init(loadcell_t loadcell);

int32_t     loadcell_value(loadcell_t loadcell);

void        loadcell_tare(loadcell_t loadcell, uint16_t sample);
int32_t	    loadcell_weight(loadcell_t loadcell, uint16_t sample);

//***********************************************************************************************//

#endif /* MZ_LOADCELL_H_ */
