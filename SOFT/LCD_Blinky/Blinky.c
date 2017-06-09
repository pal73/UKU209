/*----------------------------------------------------------------------------
 * Name:    Blinky.c
 * Purpose: LED Flasher and graphic demo for STM3220-EVAL
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

#include <stdio.h>
#include "stm32f2xx.h"                  /* STM32F2xx Definitions              */
#include "GLCD.h"

#define __FI        1                   /* Font index 16x24                   */
#if (__FI == 1)                         /* Font index 16x24                   */                         
  #define __FONT_WIDTH  16
  #define __FONT_HEIGHT 24
#else                                   /* Font index  6x8                    */
  #define __FONT_WIDTH   6
  #define __FONT_HEIGHT  8
#endif

#define LED_NUM     4                   /* Number of user LEDs                */
const unsigned long led_mask[] = { 1UL<<6, 1UL<<8, 1UL<<9, 1UL<< 7 };

extern unsigned char ClockLEDOn;
extern unsigned char ClockLEDOff;
extern unsigned char ClockANI;
extern unsigned char Bg_16bpp_t[];
extern unsigned char Bg_16bpp_l[];
extern unsigned char Bg_16bpp_r[];
extern unsigned char Bg_16bpp_b[];
extern unsigned char ARM_Ani_16bpp[];


/*----------------------------------------------------------------------------
  Function that initializes LEDs
 *----------------------------------------------------------------------------*/
void LED_init(void) {
  RCC->AHB1ENR  |= ((1UL <<  2) |       /* Enable GPIOC clock                 */
                    (1UL <<  6) |       /* Enable GPIOG clock                 */
                    (1UL <<  8)  );     /* Enable GPIOI clock                 */

  GPIOC->MODER    &= ~(3UL << 2*7);     /* PC.7 is output                     */
  GPIOC->MODER    |=  (1UL << 2*7); 
  GPIOC->OTYPER   &= ~(1UL <<   7);     /* PC.7 is output Push-Pull           */
  GPIOC->OSPEEDR  &= ~(3UL << 2*7);     /* PC.7 is 50MHz Fast Speed           */
  GPIOC->OSPEEDR  |=  (2UL << 2*7); 
  GPIOC->PUPDR    &= ~(3UL << 2*7);     /* PC.7 is Pull up                    */
  GPIOC->PUPDR    |=  (1UL << 2*7); 

  GPIOG->MODER    &= ~((3UL << 2*6) |
                       (3UL << 2*8)  ); /* PG.6, PG.8 is output               */
  GPIOG->MODER    |=  ((1UL << 2*6) |
                       (1UL << 2*8)  ); 
  GPIOG->OTYPER   &= ~((1UL <<   6) |
                       (1UL << 2*8)  ); /* PG.6, PG.8 is output Push-Pull     */
  GPIOG->OSPEEDR  &= ~((3UL << 2*6) |
                       (3UL << 2*8)  ); /* PG.6, PG.8 is 50MHz Fast Speed     */
  GPIOG->OSPEEDR  |=  ((2UL << 2*6) |
                       (2UL << 2*8)  ); 
  GPIOG->PUPDR    &= ~((3UL << 2*6) |
                       (3UL << 2*8)  ); /* PG.6, PG.8 is Pull up              */
  GPIOG->PUPDR    |=  ((1UL << 2*6) |
                       (1UL << 2*8)  ); 

  GPIOI->MODER    &= ~(3UL << 2*9);     /* PI.9 is output                     */
  GPIOI->MODER    |=  (1UL << 2*9); 
  GPIOI->OTYPER   &= ~(1UL <<   9);     /* PI.9 is output Push-Pull           */
  GPIOI->OSPEEDR  &= ~(3UL << 2*9);     /* PI.9 is 50MHz Fast Speed           */
  GPIOI->OSPEEDR  |=  (2UL << 2*9); 
  GPIOI->PUPDR    &= ~(3UL << 2*9);     /* PI.9 is Pull up                    */
  GPIOI->PUPDR    |=  (1UL << 2*9); 
}

/*----------------------------------------------------------------------------
  Function that turns on requested LED
 *----------------------------------------------------------------------------*/
void LED_On (unsigned int num) {

  if (num < 2) {
    GPIOG->BSRRL = led_mask[num];
  }
  else if (num == 2) {
    GPIOI->BSRRL = led_mask[num];
  }
  else {
    GPIOC->BSRRL = led_mask[num];
  }
}

/*----------------------------------------------------------------------------
  Function that turns off requested LED
 *----------------------------------------------------------------------------*/
void LED_Off (unsigned int num) {

  if (num < 2) {
    GPIOG->BSRRH = led_mask[num];
  }
  else if (num == 2) {
    GPIOI->BSRRH = led_mask[num];
  }
  else {
    GPIOC->BSRRH = led_mask[num];
  }
}

/*----------------------------------------------------------------------------
  Function that outputs value to LEDs
 *----------------------------------------------------------------------------*/
void LED_Out(unsigned int value) {
  int i;

  for (i = 0; i < LED_NUM; i++) {
    if (value & (1<<i)) {
      LED_On (i);
    } else {
      LED_Off(i);
    }
  }
}


/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int main (void) {                       /* Main Program                       */
  int num     = -1; 
  int dir     =  1;
  int pic     =  0;

  SysTick_Config(SystemCoreClock/100);  /* Generate interrupt every 10 ms     */

  LED_init ();

  GLCD_Init   ();
  GLCD_Clear  (White);
  GLCD_Bmp (  0,   0, 320,  69, Bg_16bpp_t+70);
  GLCD_Bmp (  0,  69,   4, 102, Bg_16bpp_l+70);
  GLCD_Bmp (316,  69,   4, 102, Bg_16bpp_r+70);
  GLCD_Bmp (  0, 171, 320,  69, Bg_16bpp_b+70);

  for (;;) {                            /* Loop forever                       */
    if (ClockANI) {
      ClockANI = 0;
      if (pic++ > 8) pic = 0;
        GLCD_Bmp (99, 99, 120, 45, &ARM_Ani_16bpp[pic*(120*45*2)]);
    }

    if (ClockLEDOn) {    /* Blink LED every 1 second (for 0.5s)*/
      ClockLEDOn  = 0;

      /* Calculate 'num': 0,1,...,LED_NUM-1,LED_NUM-1,...,1,0,0,...           */
      num += dir;
      if (num == LED_NUM) { dir = -1; num =  LED_NUM-1; } 
      else if   (num < 0) { dir =  1; num =  0;         }
    
      LED_On (num);
    }
    if (ClockLEDOff) {
      ClockLEDOff = 0;
      LED_Off(num);
    }
  }
}
