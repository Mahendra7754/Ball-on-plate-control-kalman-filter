/**
*  ***********************************************************************
*  @file    BSP.c
*  @author  B. Mysliwetz
*  @version V2.20
*
*  @brief   HAL layer with target specific functions for the Keil MCBSTM32
*           evalboard; contains low level serial char-IO & LED-IO functions. 
*
*  Changes:
*  16.11.2008 keil  V1.0
*  18.11.2008 wu    V1.1 Adaption of SVC Handler
*  03.03.2011 mys   V1.2 Serial IO functions for fputc(), fgetc() outsourced 
*  25.02.2013 mys   V2.0 Doxygen compatible & new project folder structure 
*  13.08.2016 mys   V2.1 Use stm32f10x.h, stm32f10x_rcc.h, stm32f10x_usart.h  
*                        for peripherals definitions under MDK V4.71 instead
*                        of OLD stm32f10x_lib.h
*  07.12.2016 mys   V2.2 added _aux_getch(), _aux_putch(), [ _aux_keyhit() ] 
*
*  @date    07/12/2016
**************************************************************************
*/

#include  <stdio.h>

#include  <stm32f10x.h>          // STM32F10x peripherals definitions MYS 160813
#include  <stm32f10x_rcc.h>
#include  <stm32f10x_gpio.h>
#include  <stm32f10x_usart.h>

#include  "..\basicIO\mctDefs.h" // useful constants, macros, type shorthands 
#include  "platform_cfg.h"       // board specific (re-)definitions
#include  "STM32_Init.h"         // STM32 initialization package

//#pragma import(__use_no_semihosting_swi)


void    BSP_SetLED ( UINT8 LED_bitpattern )
/**
*  ***********************************************************************
*  @brief  Set all 8 LEDs simultaneously.
*  @param  LED_bitpattern : 1 turns on, 0 turns off individual LED
*  @retval none
**************************************************************************
*/
{
  GPIO_LED->ODR = LED_bitpattern << 8;    // 1 bit turns on LED on MCBSTM32B
}


void    BSP_TurnOnOffLED ( const UINT8 LED_No, const UINT8 LED_On )
/**
*  ***********************************************************************
*  @brief  Switch ON/OFF individual/single LED (of 8 LEDs).
*  @param  LED_No : LED number (0..7)
*  @param  LED_On : 1 turns on, 0 turns off individual LED
*  @retval none
**************************************************************************
*/
{
  UINT32     tmp = 1 << (LED_No + LED_On*16);
  GPIO_LED->BSRR = tmp;                  // 1 turns on LED on MCBSTM32B
}


void    BSP_SetEXTIPR ( unsigned int EXTI_bitpattern )
/**
*  ***********************************************************************
*  @brief  Write to IO port that accesses the EXTI->PendingRegister.
*  @param  EXTI_bitpattern : a 1 bit turns off pending interrupt
*  @retval none
**************************************************************************
*/
{
  EXTI->PR = EXTI_bitpattern;    // 1 resets a pending EXTI bit
}


unsigned int  BSP_GetEXTIPR ( )
/**
*  ***********************************************************************
*  @brief  Reads IO port that accesses the EXTI->PendingRegister.
*  @retval none
**************************************************************************
*/
{
  return EXTI->PR;    // a 1 means EXTI is pending
}



char  _aux_putch (char ch)
/**
*  ***********************************************************************
*  @brief  Write character to AUX serial port.
*  @param  ch : character to output
*  @retval output character
**************************************************************************
*/
{
  while ( !(AUX_USART->SR & USART_FLAG_TXE) );
  AUX_USART->DR = (ch & 0xFF);
  return (ch);
}


char  _aux_getch (void)
/**
*  ***********************************************************************
*  @brief  Read character from AUX serial port (BLOCKiNG MODE).
*  @retval input character
**************************************************************************
*/
{
  while ( !(AUX_USART->SR & USART_FLAG_RXNE) );
  return (AUX_USART->DR & 0xFF);
}



char  _putch (char ch)   // 161207 changed param int -> char
/**
*  ***********************************************************************
*  @brief  Write character to stdout / serial CONSOLE.
*  @param  ch : character to output
*  @retval output character
**************************************************************************
*/
{
  while ( !(CONSOLE_USART->SR & USART_FLAG_TXE) );
  CONSOLE_USART->DR = (ch & 0xFF);
  return (ch);
}


char  _getch (void)
/**
*  ***********************************************************************
*  @brief  Read character from stdin / serial CONSOLE (BLOCKiNG MODE).
*  @retval input character
**************************************************************************
*/
{
  while ( !(CONSOLE_USART->SR & USART_FLAG_RXNE) );
  return (CONSOLE_USART->DR & 0xFF);
}


char  _keyhit (void)
/**
*  ***********************************************************************
*  @brief  Read character from stdin / serial CONSOLE (NON-BLOCKiNG MODE).
*  @retval input character
**************************************************************************
*/
{
  if ( CONSOLE_USART->SR & USART_FLAG_RXNE )
    return (CONSOLE_USART->DR & 0xFF);
  else
    return (0);
}


int  fputc (int ch, FILE *f)
/**
*  ***********************************************************************
*  @brief  Write character to stdout / serial port#1.
*  @param  ch : character to output
*  @param  f  : filepointer (as yet ignored here)
*  @retval output character
**************************************************************************
*/
{
  return ( _putch(ch) );
}


int  fgetc (FILE *f)
/**
*  ***********************************************************************
*  @brief  Read character from stdin / serial port#1 (BLOCKiNG MODE).
*  @param  f  : filepointer (as yet ignored here)
*  @retval input character
**************************************************************************
*/
{
  return ( _putch( _getch() ) );
}


void  _ttywrch (int ch)
/**
*  ***********************************************************************
*  @brief  Write character to stdout / serial port#1.
*  @param  ch : character to output
*  @retval none
**************************************************************************
*/
{
  _putch( ch );
}


int  ferror (FILE *f)
/**
*  ***********************************************************************
*  @brief  Return error status of file f, not yet implemented.
*  @param  f  : filepointer (as yet ignored here)
*  @retval error code (as yet const EOF)
**************************************************************************
*/
{
  // your implementation of ferror
  return EOF;
}


void _sys_exit (int return_code)
/**
*  ***********************************************************************
*  @brief  Handle system exit (regular uC/OS tasks never return).
*  @param  return_code : pass info from exiting task.
*  @retval none.
**************************************************************************
*/

{
  label:  goto label;              // endless loop
}


void  BSP_Init (void)
/**
*  ***********************************************************************
*  @brief  Initialize CPU/MCU and application relevant peripherals.
*  @retval none.
**************************************************************************
*/
{
  stm32_Init( );                  // STM32 peripherals initialization
}


unsigned long  BSP_CPU_ClkFreq (void)
/**
*  ***********************************************************************
*  @brief  Read CPU registers to determine clock frequency of the chip.
*  @retval CPU clock frequency, in [Hz].
**************************************************************************
*/
{
    RCC_ClocksTypeDef  rcc_clocks;

    RCC_GetClocksFreq(&rcc_clocks);
    return ((unsigned long)rcc_clocks.HCLK_Frequency);
}


unsigned long  OS_CPU_SysTickClkFreq (void)
/**
*  ***********************************************************************
*  @brief  Get system tick clock frequency.
*  @retval Clock frequency (of system tick).
**************************************************************************
*/
{
    unsigned long  freq;

    freq = BSP_CPU_ClkFreq();
    return (freq);
}


__asm void SVC_Handler (void)
/**
*  ***********************************************************************
*  @brief  SVC_Handler.
*  @retval none.
**************************************************************************
*/
{
  PRESERVE8

  TST     LR,#4                         ; Called from Handler Mode ?
  MRSNE   R12,PSP                       ; Yes, use PSP
  MOVEQ   R12,SP                        ; No, use MSP
  LDR     R12,[R12,#24]                 ; Read Saved PC from Stack
  LDRH    R12,[R12,#-2]                 ; Load Halfword
  BICS    R12,R12,#0xFF00               ; Extract SVC Number

  PUSH    {R4,LR}                       ; Save Registers
  LDR     LR,=SVC_Count
  LDR     LR,[LR]
  CMP     R12,LR
  BHS     SVC_Dead                      ; Overflow
  LDR     LR,=SVC_Table
  LDR     R12,[LR,R12,LSL #2]           ; Load SVC Function Address
  BLX     R12                           ; Call SVC Function

  POP     {R4,LR}
  TST     LR,#4
  MRSNE   R12,PSP
  MOVEQ   R12,SP
  STM     R12,{R0-R3}                   ; Function return values
  BX      LR                            ; RETI

SVC_Dead
  B       SVC_Dead                      ; None Existing SVC

SVC_Cnt    EQU    (SVC_End-SVC_Table)/4
SVC_Count  DCD    SVC_Cnt

; Import user SVC functions here.
;*IMPORT  __SVC_0
;*IMPORT  __SVC_1
;*IMPORT  __SVC_2
;*IMPORT  __SVC_3

SVC_Table
; Insert user SVC functions here
;*DCD     __SVC_0                   ; SVC 0 Function Entry
;*DCD     __SVC_1                   ; SVC 1 Function Entry
;*DCD     __SVC_2                   ; SVC 2 Function Entry
;*DCD     __SVC_3                   ; SVC 2 Function Entry

  DCD    0                          ; dummy vectors
  DCD    0
  DCD    0
  DCD    0

SVC_End

  ALIGN
}

/************************************************************************\
*    END OF MODULE BSP.c for the Keil MCBSTM32 evalboard
\************************************************************************/
