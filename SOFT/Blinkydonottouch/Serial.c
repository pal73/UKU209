/*----------------------------------------------------------------------------
 * Name:    Serial.c
 * Purpose: Low level serial routines
 * Note(s):
 *----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2010 Keil - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include "stm32f2xx.h"                  /* STM32F2xx Definitions              */

/*----------------------------------------------------------------------------
  Initialize UART pins, Baudrate
 *----------------------------------------------------------------------------*/
void SER_init (void) {
  int i;

  /* Configure UART4 for 115200 baud                                          */
  RCC->APB2ENR  |=  (1UL <<  0);        /* Enable AFIO clock                  */
  RCC->AHB1ENR  |=  (1UL <<  2);        /* Enable GPIOC clock                 */
  GPIOC->AFR[1] |= 0x00008800;          /* PC10 UART4_Tx, PC11 UART4_Rx  (AF8)*/
  GPIOC->MODER  &= 0xFF0FFFFF;
  GPIOC->MODER  |= 0x00A00000;

  RCC->APB1ENR  |= (1UL << 19);         /* Enable UART#4 clock                */
  UART4->BRR = 0x0100;                  /* Configure 115200 baud, @ 30MHz     */
  UART4->CR3 = 0x0000;                  /*           8 bit, 1 stop bit,       */     
  UART4->CR2 = 0x0000;                  /*           no parity                */
  for (i = 0; i < 0x1000; i++) __NOP(); /* avoid unwanted output              */
  UART4->CR1 = 0x200C;
}


/*----------------------------------------------------------------------------
  Write character to Serial Port
 *----------------------------------------------------------------------------*/
int sendchar (int c) {

#ifdef __DBG_ITM
  ITM_SendChar(c);
#else  
  while (!(UART4->SR & 0x0080));
  UART4->DR = (c & 0x1FF);
#endif  

  return (c);
}


/*----------------------------------------------------------------------------
  Read character from Serial Port   (blocking read)
 *----------------------------------------------------------------------------*/
int getkey (void) {

  while (!(UART4->SR & 0x0020));
  return (UART4->DR);
}
