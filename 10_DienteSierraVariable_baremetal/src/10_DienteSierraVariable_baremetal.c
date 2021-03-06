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

/** \brief Diente de Sierra Bare Metal example source file
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
#include "dac.h"
#include "tecla.h"
#include "10_DienteSierraVariable_baremetal.h"         /* <= own header */


/*==================[macros and definitions]=================================*/
#define DELAY 200

/*==================[internal data declaration]==============================*/

uint32_t contEscalon=0;
uint32_t FondoEscala=1024;
uint32_t Vref=0;
uint8_t intervalo=1;


Tecla tecEj10;

/*==================[internal functions declaration]=========================*/
void RIT_IRQHandler (void){
	RIT_clear_flag();


	/*"cont" se incrementa cada 1ms. El salto de cada escal�n est� regido por "contEscalon" y ser� cada 10ms. De esta manera, la diente de sierra se recorrera� completamente en 10seg*/
	  contEscalon++;
	  if(contEscalon>=FondoEscala){
		 	contEscalon=0;}


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

dac_init();

tecla_init();


	   while (1){
		   leer_tecla (&tecEj10);
		   if(tecEj10.tecla_1==TRUE){if (FondoEscala>=1024)FondoEscala=1024;
		   	   	   	   	   	   	   	   else FondoEscala+=10;}
		   if(tecEj10.tecla_2==TRUE){if (FondoEscala<=10)FondoEscala=10;
	   	   	   	   	   	   	   	   	   else FondoEscala-=10;}
		   if(tecEj10.tecla_3==TRUE){if (intervalo>=20)intervalo=20;
	   	   	   	   	   	   	   	   	   else intervalo+=1;}
		   if(tecEj10.tecla_4==TRUE){if (intervalo<=1)intervalo=1;
	   	   	   	   	   	   	   	   	   else intervalo-=1;}
		   timers_Set_Value(intervalo);
		   cargar_dato_dac (contEscalon);
	   }

	   return 0;
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

