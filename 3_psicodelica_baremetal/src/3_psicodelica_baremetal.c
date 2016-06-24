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
#include "3_psicodelica_baremetal.h"         /* <= own header */


/*==================[macros and definitions]=================================*/
#define DELAY 5000000

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



int main(void)
{uint8_t Led;
int32_t cont;


LedRGB RGB_EduCIAA;


   /* Se inicializan los puertos correspondientes a los Leds de la placa */
led_init ();

	   while (1){
		   	   /* Se encienden y apagan los led 1, 2 y 3 */
		   	   for(Led=3; Led!=0; Led--)
		   	   {   encender_led (Led);

			   	   for(cont=DELAY;cont!=0;cont --);

			   	   apagar_led (Led);

			   	   for(cont=DELAY;cont!=0;cont --);
		   	   }

		   	   /*Seteo del Led Rojo en la estructura y reseteo del Verde y Azul*/
		   	RGB_EduCIAA.Rojo=TRUE;
		   	RGB_EduCIAA.Verde=FALSE;
		   	RGB_EduCIAA.Azul=FALSE;
		   	encender_ledRGB (&RGB_EduCIAA);
		   	for(cont=DELAY;cont!=0;cont --);
		   	apagar_ledRGB(&RGB_EduCIAA);
		   	for(cont=DELAY;cont!=0;cont --);

		   	   /*Seteo del Led Verde en la estructura y reseteo del Rojo y Azul*/
			RGB_EduCIAA.Rojo=FALSE;
			RGB_EduCIAA.Verde=TRUE;
			RGB_EduCIAA.Azul=FALSE;
			encender_ledRGB (&RGB_EduCIAA);
			for(cont=DELAY;cont!=0;cont --);
			apagar_ledRGB(&RGB_EduCIAA);
			for(cont=DELAY;cont!=0;cont --);

		 	   /*Seteo del Led Azul en la estructura y reseteo del Rojo y Verde*/
			RGB_EduCIAA.Rojo=FALSE;
			RGB_EduCIAA.Verde=FALSE;
			RGB_EduCIAA.Azul=TRUE;
			encender_ledRGB (&RGB_EduCIAA);
			for(cont=DELAY;cont!=0;cont --);
			apagar_ledRGB(&RGB_EduCIAA);
			for(cont=DELAY;cont!=0;cont --);

			RGB_EduCIAA.Rojo=0;
			RGB_EduCIAA.Verde=0;
			RGB_EduCIAA.Azul=0;
	   }
	   return 0;
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

