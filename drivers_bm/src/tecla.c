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
#define PackPuerto02 2
#define PackPuerto03 3
#define PackPuerto04 4
#define PackPuerto05 5
#define PackPuerto06 6
#define PackPuerto07 7

#define PinPuerto0 0
#define PinPuerto1 1
#define PinPuerto2 2
#define PinPuerto3 3
#define PinPuerto4 4
#define PinPuerto5 5
#define PinPuerto6 6

#define Tecla1Pin 4
#define Tecla2Pin 8
#define Tecla3Pin 9
#define Tecla4Pin 9
#define TeclaF0Pin 0
#define TeclaF1Pin 1
#define TeclaF2Pin 2
#define TeclaF3Pin 3
#define TeclaC0Pin 8
#define TeclaC1Pin 12
#define TeclaC2Pin 13

#define Tecla1Bit 1<<4
#define Tecla2Bit 1<<8
#define Tecla3Bit 1<<9
#define Tecla4Bit 1<<9
#define TeclaF0Bit 1<<0
#define TeclaF1Bit 1<<1
#define TeclaF2Bit 1<<2
#define TeclaF3Bit 1<<3
#define TeclaC0Bit 1<<8
#define TeclaC1Bit 1<<12
#define TeclaC2Bit 1<<13

#define GPIO0 0
#define GPIO1 1
#define GPIO2 2
#define GPIO3 3



#define OUTPUT_DIRECTION 1
#define INPUT_DIRECTION 0


uint8_t fila=0;
uint8_t col=0;
uint8_t tecla=0;
uint8_t modoLectura=1;
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
Chip_SCU_PinMux(PackPuerto01, PinPuerto0 ,MD_PUP|MD_EZI|MD_ZI,FUNC0);
Chip_SCU_PinMux(PackPuerto01, PinPuerto1 ,MD_PUP|MD_EZI|MD_ZI,FUNC0);
Chip_SCU_PinMux(PackPuerto01, PinPuerto2 ,MD_PUP|MD_EZI|MD_ZI,FUNC0);
Chip_SCU_PinMux(PackPuerto01, PinPuerto6 ,MD_PUP|MD_EZI|MD_ZI,FUNC0);


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



uint8_t teclado_init (void){
	Chip_GPIO_Init (LPC_GPIO_PORT); //Inicializa puertos a partir de una direccion base

	/*Asignacion de funciones de los pines*/
	Chip_SCU_PinMux(PackPuerto04, PinPuerto0 ,MD_PUP,FUNC0);
	Chip_SCU_PinMux(PackPuerto04, PinPuerto1 ,MD_PUP,FUNC0);
	Chip_SCU_PinMux(PackPuerto04, PinPuerto2 ,MD_PUP,FUNC0);
	Chip_SCU_PinMux(PackPuerto04, PinPuerto3 ,MD_PUP,FUNC0);
	Chip_SCU_PinMux(PackPuerto01, PinPuerto5 ,MD_PUP|MD_EZI|MD_ZI,FUNC0);
	Chip_SCU_PinMux(PackPuerto07, PinPuerto4 ,MD_PUP|MD_EZI|MD_ZI,FUNC0);
	Chip_SCU_PinMux(PackPuerto07, PinPuerto5 ,MD_PUP|MD_EZI|MD_ZI,FUNC0);

	/*Asignacion de puertos*/
	Chip_GPIO_SetDir 	( LPC_GPIO_PORT, GPIO2, TeclaF0Bit, salida);
	Chip_GPIO_SetDir 	( LPC_GPIO_PORT, GPIO2, TeclaF1Bit, salida);
	Chip_GPIO_SetDir 	( LPC_GPIO_PORT, GPIO2, TeclaF2Bit, salida);
	Chip_GPIO_SetDir 	( LPC_GPIO_PORT, GPIO2, TeclaF3Bit, salida);
	Chip_GPIO_SetDir 	( LPC_GPIO_PORT, GPIO1, TeclaC0Bit, entrada);
	Chip_GPIO_SetDir 	( LPC_GPIO_PORT, GPIO3, TeclaC1Bit, entrada);
	Chip_GPIO_SetDir 	( LPC_GPIO_PORT, GPIO3, TeclaC2Bit, entrada);

	return TRUE;
}




void leer_teclado (Tecla *TeclaCIAA)

{		tecla=0;
		for(fila=0; fila<4; fila++){
					/*Se ponen todas las filas en alto*/
					Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT,GPIO2,TeclaF0Pin);
					Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT,GPIO2,TeclaF1Pin);
					Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT,GPIO2,TeclaF2Pin);
					Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT,GPIO2,TeclaF3Pin);

					/*Segun el estado de la variable fila, se habilita una fila en particular*/
					switch(fila){
					case 0: Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,GPIO2,TeclaF0Pin);
							break;
					case 1: Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,GPIO2,TeclaF1Pin);
							break;
					case 2: Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,GPIO2,TeclaF2Pin);
							break;
					case 3: Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,GPIO2,TeclaF3Pin);
					        break;
										}
					/*Se coteja el estado de las columnas para una fila determinada*/
					for(col=0; col<3; col++){
							switch (col){
							case 0: if(!Chip_GPIO_ReadPortBit(LPC_GPIO_PORT, GPIO1, TeclaC0Pin)) tecla= (3*fila)+col+1; /* este cálculo permite obtener el valor de la tecla*/
									break;
							case 1: if(!Chip_GPIO_ReadPortBit(LPC_GPIO_PORT, GPIO3, TeclaC1Pin)) tecla= (3*fila)+col+1; /* este cálculo permite obtener el valor de la tecla*/
									break;
							case 2: if(!Chip_GPIO_ReadPortBit(LPC_GPIO_PORT, GPIO3, TeclaC2Pin)) tecla= (3*fila)+col+1; /* este cálculo permite obtener el valor de la tecla*/
									break;
							}
	   	   	   	   }
   	   	}

		/*Finalmente se carga el elemento correspondiente en la estructura según la tecla pulsada*/
		switch (tecla){
					case 0: {TeclaCIAA->tecla_0=FALSE;
							 TeclaCIAA->tecla_1=FALSE;
							 TeclaCIAA->tecla_2=FALSE;
							 TeclaCIAA->tecla_3=FALSE;
							 TeclaCIAA->tecla_4=FALSE;
							 TeclaCIAA->tecla_5=FALSE;
							 TeclaCIAA->tecla_6=FALSE;
							 TeclaCIAA->tecla_7=FALSE;
							 TeclaCIAA->tecla_8=FALSE;
							 TeclaCIAA->tecla_9=FALSE;
							 TeclaCIAA->tecla_A=FALSE;
							 TeclaCIAA->tecla_N=FALSE;
							 TeclaCIAA->valorTecla=FALSE;
								}
							break;
					case 1: TeclaCIAA->tecla_1=TRUE;
							TeclaCIAA->valorTecla=1;
							break;
					case 2:	TeclaCIAA->tecla_2=TRUE;
							TeclaCIAA->valorTecla=2;
							break;
					case 3:	TeclaCIAA->tecla_3=TRUE;
							TeclaCIAA->valorTecla=3;
							break;
					case 4:	TeclaCIAA->tecla_4=TRUE;
							TeclaCIAA->valorTecla=4;
							break;
					case 5:	TeclaCIAA->tecla_5=TRUE;
							TeclaCIAA->valorTecla=5;
							break;
					case 6:	TeclaCIAA->tecla_6=TRUE;
							TeclaCIAA->valorTecla=6;
							break;
					case 7:	TeclaCIAA->tecla_7=TRUE;
							TeclaCIAA->valorTecla=7;
							break;
					case 8:	TeclaCIAA->tecla_8=TRUE;
							TeclaCIAA->valorTecla=8;
							break;
					case 9:	TeclaCIAA->tecla_9=TRUE;
							TeclaCIAA->valorTecla=9;;
							break;
					case 10:TeclaCIAA->tecla_A=TRUE;
							TeclaCIAA->valorTecla=42; /*valor decimal en ASCII del Asterisco*/
							break;
					case 11:TeclaCIAA->tecla_0=TRUE;
							TeclaCIAA->valorTecla=0;
							break;
					case 12:TeclaCIAA->tecla_N=TRUE;
							TeclaCIAA->valorTecla=35; /*valor decimal en ASCII del Numeral*/
							break;
					}

		}


/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/


