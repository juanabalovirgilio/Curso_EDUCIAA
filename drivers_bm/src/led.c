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


#include "led.h"












/*==================[macros and definitions]=================================*/

#define TRUE 1
#define FALSE 0

#define PackPuerto02 2

#define RGBPinRojo 0
#define RGBPinVerde 1
#define RGBPinAzul 2
#define LED1Pin 14
#define LED2Pin 11
#define LED3Pin 12

#define RGBBitRojo 1<<0
#define RGBBitVerde 1<<1
#define RGBBitAzul 1<<2
#define LED1Bit 1<<14
#define LED2Bit 1<<11
#define LED3Bit 1<<12

#define GPIO0 0
#define GPIO1 1
#define GPIO5 5
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
uint8_t led_init (void)
{Chip_GPIO_Init (LPC_GPIO_PORT); //Inicializa puertos a partir de una direccion base


/*Asignacion de funciones de los pines   */
Chip_SCU_PinMux(PackPuerto02,RGBPinRojo,MD_PUP,FUNC4);
Chip_SCU_PinMux(PackPuerto02,RGBPinVerde,MD_PUP,FUNC4);
Chip_SCU_PinMux(PackPuerto02,RGBPinAzul,MD_PUP,FUNC4);
Chip_SCU_PinMux(PackPuerto02,LED1Pin,MD_PUP,FUNC0);
Chip_SCU_PinMux(PackPuerto02,LED2Pin,MD_PUP,FUNC0);
Chip_SCU_PinMux(PackPuerto02,LED3Pin,MD_PUP,FUNC0);


/*Asignacion de puertos   */

Chip_GPIO_SetDir 	( LPC_GPIO_PORT, GPIO5, RGBBitRojo, salida );
Chip_GPIO_SetDir 	( LPC_GPIO_PORT, GPIO5, RGBBitVerde, salida );
Chip_GPIO_SetDir 	( LPC_GPIO_PORT, GPIO5, RGBBitAzul, salida );
Chip_GPIO_SetDir 	( LPC_GPIO_PORT, GPIO0, LED1Bit, salida );
Chip_GPIO_SetDir 	( LPC_GPIO_PORT, GPIO1, LED2Bit, salida );
Chip_GPIO_SetDir 	( LPC_GPIO_PORT, GPIO1, LED3Bit, salida );

return TRUE;
}

 uint8_t encender_led (int8_t Led)
{switch (Led){
	case Led_1: Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT,GPIO0,LED1Pin );
				break;
	case Led_2: Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT,GPIO1,LED2Pin);
				break;
	case Led_3: Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT,GPIO1,LED3Pin);
				break;
	default: return FALSE;
				break;
}
	return TRUE;
}

 uint8_t apagar_led (int8_t Led)
{switch (Led){
	case Led_1: Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,GPIO0,LED1Pin);
				break;
	case Led_2: Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,GPIO1,LED2Pin);
				break;
	case Led_3: Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,GPIO1,LED3Pin);
				break;
	default: return FALSE;
				break;
}
	return TRUE;
}

 uint8_t togglear_led (int8_t Led)
{switch (Led){
	case Led_1: Chip_GPIO_SetPinToggle(LPC_GPIO_PORT,GPIO0,LED1Pin);
				break;
	case Led_2: Chip_GPIO_SetPinToggle(LPC_GPIO_PORT,GPIO1,LED2Pin);
				break;
	case Led_3: Chip_GPIO_SetPinToggle(LPC_GPIO_PORT,GPIO1,LED3Pin);
				break;
	default: return FALSE;
				break;
}
	return TRUE;
}


 /*En la siguiente función se evalúa el estado de cada Led del RGB. Si ninguno es modificado, la función devuelve un estado "False"*/
 uint8_t encender_ledRGB ( LedRGB *LedRGB)
 {
  uint8_t estado;

 if(LedRGB->Rojo) {Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT,GPIO5,RGBPinRojo);
  	  	  	  	  estado=TRUE;}
  else estado=FALSE;

  if(LedRGB->Verde) {Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT,GPIO5,RGBPinVerde);
  	  	  	  	  estado=TRUE;}
  else estado=FALSE;

  if(LedRGB->Azul) {Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT,GPIO5,RGBPinAzul);
  	  	  	  	  estado=TRUE;}
  else estado=FALSE;

  return estado;
 }


 /*En la siguiente función se evalúa el estado de cada Led del RGB. Si ninguno es modificado, la función devuelve un estado "False"*/
 uint8_t apagar_ledRGB ( LedRGB *LedRGB)
 {uint8_t estado;

 if(LedRGB->Rojo) {Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,GPIO5,RGBPinRojo);
  	  	  	  	  estado=TRUE;}
  else estado=FALSE;

  if(LedRGB->Verde) {Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,GPIO5,RGBPinVerde);
  	  	  	  	  estado=TRUE;}
  else estado=FALSE;

  if(LedRGB->Azul) {Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,GPIO5,RGBPinAzul);
  	  	  	  	  estado=TRUE;}
  else estado=FALSE;

  return estado;
 }


 /*En la siguiente función se evalúa el estado de cada Led del RGB. Si ninguno es modificado, la función devuelve un estado "False"*/
 uint8_t togglear_ledRGB ( LedRGB *LedRGB)
 {uint8_t estado;

 if(LedRGB->Rojo) {Chip_GPIO_SetPinOutToggle(LPC_GPIO_PORT,GPIO5,RGBPinRojo);
  	  	  	  	  estado=TRUE;}
  else estado=FALSE;

  if(LedRGB->Verde) {Chip_GPIO_SetPinOutToggle(LPC_GPIO_PORT,GPIO5,RGBPinVerde);
  	  	  	  	  estado=TRUE;}
  else estado=FALSE;

  if(LedRGB->Azul) {Chip_GPIO_SetPinOutToggle(LPC_GPIO_PORT,GPIO5,RGBPinAzul);
  	  	  	  	  estado=TRUE;}
  else estado=FALSE;

  return estado;
 }

 /*En la siguiente función se evalúa el estado de cada Led del RGB. Si ninguno es modificado, la función devuelve un estado "False"*/
   void togglear_ledRGB_ref ( LedRGB *LedRGB)
   {

   if((LedRGB->Rojo)>=(LedRGB->Ref)) Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT,GPIO5,RGBPinRojo);
       else Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,GPIO5,RGBPinRojo);

   if((LedRGB->Verde)>=(LedRGB->Ref)) Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT,GPIO5,RGBPinVerde);
       else Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,GPIO5,RGBPinVerde);

   if((LedRGB->Azul)>=(LedRGB->Ref)) Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT,GPIO5,RGBPinAzul);
       else Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,GPIO5,RGBPinAzul);


   }

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

