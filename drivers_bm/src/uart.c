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


#include "uart.h"












/*==================[macros and definitions]=================================*/

#define TRUE 1
#define FALSE 0


LPC_USART_T *PuntUART;

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

void uart_init (void){
	Chip_UART_Init(LPC_USART2);
	Chip_SCU_PinMux(7,1,MD_PDN,FUNC6);
	Chip_SCU_PinMux(7,2,MD_PLN|MD_EZI|MD_ZI, FUNC6);
	Chip_UART_SetupFIFOS(LPC_USART2, UART_FCR_FIFO_EN|UART_FCR_TRG_LEV0);
	Chip_UART_SetBaud(LPC_USART2,115200);
	Chip_UART_TXEnable(LPC_USART2);
}

uint8_t uart_leer_dato (){
	return Chip_UART_ReadByte(LPC_USART2);
}

void uart_escribir_dato(uint8_t data){
	Chip_UART_SendByte(LPC_USART2, data );
}

uint8_t uart_estado_T (){
	Chip_UART_ReadLineStatus(LPC_USART2);
	if (LPC_USART2->LSR &&UART_LSR_TEMT!=FALSE) return TRUE;
	else return FALSE;
}

uint8_t uart_estado_R (){
	Chip_UART_ReadLineStatus(LPC_USART2);
	if (LPC_USART2->LSR &&UART_LSR_RDR !=FALSE) return TRUE;
		else return FALSE;
}

void uart_escribir_string(char message[], uint8_t size)
{
	uint8_t msjIndex = 0;
	uint64_t i;

	/* sending byte by byte*/
	while(( uart_estado_T () != 0) && (msjIndex < size))
	{
		Chip_UART_SendByte((LPC_USART_T *)LPC_USART2, message[msjIndex]);

		/*delay*/
		for (i=0;i<50000;i++)
		{
			asm  ("nop");
		}
		msjIndex++;
	}

}


/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/


