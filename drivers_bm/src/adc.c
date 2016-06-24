/* Copyright 2016, XXXXXXXXX  
 * All rights reserved.
 *
 * This file is part of CIAA Firmware.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */


/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */

/** \addtogroup Examples CIAA Firmware Examples
 ** @{ */
/** \addtogroup Baremetal Bare Metal LED Driver
 ** @{ */

/*
 * Initials     Name
 * ---------------------------
 *
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * yyyymmdd v0.0.1 initials initial version
 */

/*==================[inclusions]=============================================*/


#include "adc.h"












/*==================[macros and definitions]=================================*/

#define TRUE 1
#define FALSE 0


ADC_CLOCK_SETUP_T ADCSetupClk;


/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
/** \brief Main function
 *
 * This is the main entry point of the software.
 *
 * \returns 0
 *
 * \remarks This function never returns. Return value is only to avoid compiler
 *          warnings or errors.
 */
void adc_init (uint32_t Modulo){
	if(Modulo==0)
		{Chip_ADC_Init (LPC_ADC0, &ADCSetupClk);
		}

	if(Modulo==1)
	{Chip_ADC_Init (LPC_ADC1, &ADCSetupClk);
			}
}

void adc_config_canal (ADC_CHANNEL_T canal, FunctionalState estado ){
	Chip_ADC_EnableChannel(LPC_ADC0, canal, estado);
}

void adc_config_modulo (uint32_t modulo, ADC_CHANNEL_T canal){
	 Chip_SCU_ADC_Channel_Config (modulo,canal);
 }

void adc_modoInicio (ADC_START_MODE_T modoInicio, ADC_EDGE_CFG_T flanco){
	Chip_ADC_SetStartMode (LPC_ADC0, modoInicio, flanco);
}



void  leer_dato_adc (uint16_t *dato, ADC_CHANNEL_T canal){
	Chip_ADC_ReadValue (LPC_ADC0, canal, dato );
}

FlagStatus leer_estado_adc (ADC_CHANNEL_T canal,ADC_STATUS_T estadoConversion){
	return Chip_ADC_ReadStatus(LPC_ADC0,canal,estadoConversion);
}
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/


