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

/** \brief Blinking Bare Metal driver led
 **
 **
 **
 **/

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


#include "tecla.h"












/*==================[macros and definitions]=================================*/

#define TRUE 1
#define FALSE 0

#define PackPuerto01 1

#define Tecla1Pin 4
#define Tecla2Pin 8
#define Tecla3Pin 9
#define Tecla4Pin 9


#define Tecla1Bit 1<<4
#define Tecla2Bit 1<<8
#define Tecla3Bit 1<<9
#define Tecla4Bit 1<<9


#define GPIO0 0
#define GPIO1 1

#define SW1_MUX_GROUP 1
#define SW1_MUX_PIN 0
#define SW1_GPIO_PORT 0
#define SW1_GPIO_PIN 4

#define SW2_MUX_GROUP 1
#define SW2_MUX_PIN 1
#define SW2_GPIO_PORT 0
#define SW2_GPIO_PIN 8

#define SW3_MUX_GROUP 1
#define SW3_MUX_PIN 2
#define SW3_GPIO_PORT 0
#define SW3_GPIO_PIN 9

#define SW4_MUX_GROUP 1
#define SW4_MUX_PIN 6
#define SW4_GPIO_PORT 1
#define SW4_GPIO_PIN 9

#define OUTPUT_DIRECTION 1
#define INPUT_DIRECTION 0

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
uint8_t tecla_init (void)
{Chip_GPIO_Init (LPC_GPIO_PORT); //Inicializa puertos a partir de una direccion base


/*Asignacion de funciones de los pines*/
Chip_SCU_PinMux(PackPuerto01, 0 ,MD_PUP|MD_EZI|MD_ZI,FUNC0);
Chip_SCU_PinMux(PackPuerto01, 1 ,MD_PUP|MD_EZI|MD_ZI,FUNC0);
Chip_SCU_PinMux(PackPuerto01, 2 ,MD_PUP|MD_EZI|MD_ZI,FUNC0);
Chip_SCU_PinMux(PackPuerto01, 6 ,MD_PUP|MD_EZI|MD_ZI,FUNC0);


/*Asignacion de puertos*/

Chip_GPIO_SetDir 	( LPC_GPIO_PORT, GPIO0, Tecla1Bit, entrada );
Chip_GPIO_SetDir 	( LPC_GPIO_PORT, GPIO0, Tecla2Bit, entrada);
Chip_GPIO_SetDir 	( LPC_GPIO_PORT, GPIO0, Tecla3Bit, entrada );
Chip_GPIO_SetDir 	( LPC_GPIO_PORT, GPIO1, Tecla4Bit, entrada );


return TRUE;
}



void leer_tecla (Tecla *TeclaCIAA)

{

	 	if(!Chip_GPIO_ReadPortBit(LPC_GPIO_PORT, GPIO0, Tecla1Pin)) TeclaCIAA->tecla_1=TRUE;
	 	else TeclaCIAA->tecla_1=FALSE;

		if(!Chip_GPIO_ReadPortBit(LPC_GPIO_PORT, GPIO0, Tecla2Pin)) TeclaCIAA->tecla_2=TRUE;
		else TeclaCIAA->tecla_2=FALSE;

		if(!Chip_GPIO_ReadPortBit(LPC_GPIO_PORT, GPIO0, Tecla3Pin))TeclaCIAA->tecla_3=TRUE;
		else TeclaCIAA->tecla_3=FALSE;

		if(!Chip_GPIO_ReadPortBit(LPC_GPIO_PORT, GPIO1, Tecla4Pin))TeclaCIAA->tecla_4=TRUE;
		else TeclaCIAA->tecla_4=FALSE;



}


/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/


