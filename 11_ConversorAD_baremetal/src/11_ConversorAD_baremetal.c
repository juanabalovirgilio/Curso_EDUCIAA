/* Copyright 2016, XXXXXX
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

/** \
 **
 ** This is a mini example of the CIAA Firmware.
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */

/** \addtogroup Examples CIAA Firmware Examples
 ** @{ */
/** \addtogroup Baremetal Bare Metal example source file
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



#include "timers.h"
#include "led.h"
#include "11_ConversorAD_baremetal.h"         /* <= own header */


/*==================[macros and definitions]=================================*/
#define DELAY 200
#define Modulo_0 0
#define Modulo_1 1
#define Canal_0 0
#define Canal_1 1
#define Canal_2 2
#define Canal_3 3
#define Canal_4 4
#define Canal_5 5

#define Led_1 1
#define Led_2 2
#define Led_3 3
/*==================[internal data declaration]==============================*/
uint16_t dato=0;




ADC_CHANNEL_T canal;
FunctionalState estado;
ADC_START_MODE_T modoInicio;
ADC_EDGE_CFG_T flanco;
ADC_STATUS_T estadoConversion;


/*==================[internal functions declaration]=========================*/
void RIT_IRQHandler (void){
	RIT_clear_flag();

	adc_modoInicio (ADC_START_NOW, ADC_TRIGGERMODE_RISING);


}

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



int main(void)
{




   /* Se inicializan el timer para generar las interrupciones que permitan generar los escalones del diente de sierra*/
timers_init ();

led_init ();

adc_init (Modulo_0);
adc_config_canal (ADC_CH1, ENABLE );
adc_modoInicio (ADC_NO_START, ADC_TRIGGERMODE_FALLING);
adc_config_modulo (Modulo_0, ADC_CH1);



	   while (1){


		  while (leer_estado_adc(ADC_CH1, ADC_DR_DONE_STAT)!=SET){}

		  leer_dato_adc (&dato, ADC_CH1);

		  if(dato>= 512) encender_led (Led_1);
		  else apagar_led (Led_1);}


	   return 0;
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

