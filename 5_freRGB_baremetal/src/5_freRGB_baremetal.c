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

/** \brief Blinking Bare Metal example source file
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


#include "led.h"
#include "timers.h"
#include "tecla.h"
/*#include "interrupt_timers.h"*/
#include "5_freRGB_baremetal.h"         /* <= own header */


/*==================[macros and definitions]=================================*/
#define DELAY 200

/*==================[internal data declaration]==============================*/
uint8_t bandera=0;
uint32_t cont=0;
uint32_t contaux=0;
uint8_t contCombinaciones=0;
uint8_t baseParpadeo=0;
LedRGB VectorColores [13];



/*==================[internal functions declaration]=========================*/
void RIT_IRQHandler (void){
	RIT_clear_flag();

	 baseParpadeo ++;
	 if(baseParpadeo>=250){
		 if (bandera) bandera=0;
		 else bandera=1;
		 baseParpadeo=0;
	 	 }

	 if (cont>=20) {    contaux++;
		 	 	 	 	 if(contaux>=50){
		 	 	 	 	 contaux=0;
		 	 	 	 	 contCombinaciones++;
		 	 	 	 	 if (contCombinaciones>=14) contCombinaciones=0;
		 	 	 	 	}

		 	 	 	 	 cont=0;

	 	 	 	 	 }
	 else cont ++;
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
Tecla Tecla_EduCIAA;


VectorColores[0].Rojo=255;
VectorColores[0].Verde=255;
VectorColores[0].Azul=255;


VectorColores[1].Rojo=190;
VectorColores[1].Verde=255;
VectorColores[1].Azul=255;


VectorColores[2].Rojo=190;
VectorColores[2].Verde=190;
VectorColores[2].Azul=255;


VectorColores[3].Rojo=190;
VectorColores[3].Verde=190;
VectorColores[3].Azul=190;

VectorColores[4].Rojo=125;
VectorColores[4].Verde=190;
VectorColores[4].Azul=190;

VectorColores[5].Rojo=125;
VectorColores[5].Verde=125;
VectorColores[5].Azul=190;

VectorColores[6].Rojo=125;
VectorColores[6].Verde=125;
VectorColores[6].Azul=125;

VectorColores[7].Rojo=60;
VectorColores[7].Verde=125;
VectorColores[7].Azul=125;

VectorColores[8].Rojo=60;
VectorColores[8].Verde=60;
VectorColores[8].Azul=125;

VectorColores[9].Rojo=60;
VectorColores[9].Verde=60;
VectorColores[9].Azul=60;

VectorColores[10].Rojo=0;
VectorColores[10].Verde=60;
VectorColores[10].Azul=60;

VectorColores[11].Rojo=0;
VectorColores[11].Verde=0;
VectorColores[11].Azul=60;

VectorColores[12].Rojo=0;
VectorColores[12].Verde=0;
VectorColores[12].Azul=0;




   /* Se inicializan los puertos correspondientes a los Leds de la placa y las teclas*/
led_init ();
timers_init ();
tecla_init();
/*timers_Set_Value (DELAY);*/

	   while (1){
		  leer_tecla (&Tecla_EduCIAA);

		   if (!(Tecla_EduCIAA.tecla_1||Tecla_EduCIAA.tecla_2||Tecla_EduCIAA.tecla_3||Tecla_EduCIAA.tecla_4))
		   {if(bandera){encender_led(Led_1);
		   	   	   	   	apagar_led(Led_3);}
		   else {encender_led(Led_3);
	   	   	   	apagar_led(Led_1);}


		   }
		  if (Tecla_EduCIAA.tecla_1){
			  apagar_led(Led_3);
			  apagar_led(Led_1);
			  VectorColores[contCombinaciones].Ref=cont*12;
		   	   	 tooglear_ledRGB_ref ( &(VectorColores[contCombinaciones]));}

	   }

	   return 0;
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

