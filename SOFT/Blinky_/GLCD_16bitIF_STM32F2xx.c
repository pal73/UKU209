/******************************************************************************/
/* GLCD_16bitIF_STM32F2xx.c: STM32F2xx low level Graphic LCD (320x240 pixels) */
/*                           with 16-bit parallel interface                   */
/******************************************************************************/
/* This file is part of the uVision/ARM development tools.                    */
/* Copyright (c) 2010 Keil - An ARM Company. All rights reserved.             */
/* This software may only be used under the terms of a valid, current,        */
/* end user licence from KEIL for a compatible version of KEIL software       */
/* development tools. Nothing else gives you the right to use this software.  */
/******************************************************************************/


#include "stm32f2xx.h"                  /* STM32F2xx Definitions              */
#include "GLCD.h"
#include "Font_6x8_h.h"
#include "Font_16x24_h.h"

/************************** Orientation  configuration ************************/

#define HORIZONTAL  1                   /* If vertical = 0, if horizontal = 1 */

/*********************** Hardware specific configuration **********************/

/*------------------------- Speed dependant settings -------------------------*/

/* If processor works on high frequency delay has to be increased, it can be 
   increased by factor 2^N by this constant                                   */
#define DELAY_2N    18

/*---------------------- Graphic LCD size definitions ------------------------*/

#if (HORIZONTAL == 1)
#define WIDTH       320                 /* Screen Width (in pixels)           */
#define HEIGHT      240                 /* Screen Hight (in pixels)           */
#else
#define WIDTH       240                 /* Screen Width (in pixels)           */
#define HEIGHT      320                 /* Screen Hight (in pixels)           */
#endif
#define BPP         16                  /* Bits per pixel                     */
#define BYPP        ((BPP+7)/8)         /* Bytes per pixel                    */

/*--------------- Graphic LCD interface hardware definitions -----------------*/

/* Note: LCD /CS is NE3 - Bank 3 of NOR/SRAM Bank 1~4 */
#define LCD_BASE        (0x60000000UL | 0x08000000UL)
#define LCD_REG16  (*((volatile unsigned short *)(LCD_BASE  ))) 
#define LCD_DAT16  (*((volatile unsigned short *)(LCD_BASE+2)))
 
/*---------------------------- Global variables ------------------------------*/

/******************************************************************************/
static volatile unsigned short TextColor = Black, BackColor = White;


/************************ Local auxiliary functions ***************************/

/*******************************************************************************
* Delay in while loop cycles                                                   *
*   Parameter:    cnt:    number of while cycles to delay                      *
*   Return:                                                                    *
*******************************************************************************/

static void delay (int cnt) {

  cnt <<= DELAY_2N;
  while (cnt--);
}


/*******************************************************************************
* Write a command the LCD controller                                           *
*   Parameter:    cmd:    command to be written                                *
*   Return:                                                                    *
*******************************************************************************/

static __inline void wr_cmd (unsigned char cmd) {

  LCD_REG16 = cmd;
}


/*******************************************************************************
* Write data to the LCD controller                                             *
*   Parameter:    dat:    data to be written                                   *
*   Return:                                                                    *
*******************************************************************************/

static __inline void wr_dat (unsigned short dat) {

  LCD_DAT16 = dat;
}


/*******************************************************************************
* Start of data writing to the LCD controller                                  *
*   Parameter:                                                                 *
*   Return:                                                                    *
*******************************************************************************/

static __inline void wr_dat_start (void) {

  /* only used for SPI interface */
}


/*******************************************************************************
* Stop of data writing to the LCD controller                                   *
*   Parameter:                                                                 *
*   Return:                                                                    *
*******************************************************************************/

static __inline void wr_dat_stop (void) {

  /* only used for SPI interface */
}


/*******************************************************************************
* Data writing to the LCD controller                                           *
*   Parameter:    dat:    data to be written                                   *
*   Return:                                                                    *
*******************************************************************************/

static __inline void wr_dat_only (unsigned short dat) {

  LCD_DAT16 = dat;
}


/*******************************************************************************
* Read data from the LCD controller                                            *
*   Parameter:                                                                 *
*   Return:               read data                                            *
*******************************************************************************/

static __inline unsigned short rd_dat (void) {

  return (LCD_DAT16);                                    /* return value */
}


/*******************************************************************************
* Write a value to the to LCD register                                         *
*   Parameter:    reg:    register to be written                               *
*                 val:    value to write to the register                       *
*******************************************************************************/

static __inline void wr_reg (unsigned char reg, unsigned short val) {

  wr_cmd(reg);
  wr_dat(val);
}


/*******************************************************************************
* Read from the LCD register                                                   *
*   Parameter:    reg:    register to be read                                  *
*   Return:               value read from the register                         *
*******************************************************************************/

static unsigned short rd_reg (unsigned char reg) {

  wr_cmd(reg);
  return(rd_dat());
}


/************************ Exported functions **********************************/

/*******************************************************************************
* Initialize the Graphic LCD controller                                        *
*   Parameter:                                                                 *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_Init (void) { 
  static unsigned short driverCode;

/* Configure the LCD Control pins --------------------------------------------*/
  RCC->AHB1ENR  |= ((1UL <<  0) |       /* Enable GPIOA clock                 */
                    (1UL <<  3) |       /* Enable GPIOD clock                 */
                    (1UL <<  4) |       /* Enable GPIOE clock                 */
                    (1UL <<  5) |       /* Enable GPIOF clock                 */
                    (1UL <<  6)  );     /* Enable GPIOG clock                 */

  /* PD.00(D2),  PD.01(D3),  PD.04(NOE), PD.05(NWE)           */ 
  /* PD.08(D13), PD.09(D14), PD.10(D15), PD.14(D0), PD.15(D1) */
  GPIOD->MODER    &= ~0xF03F0F0F;       /* clear Bits                         */
  GPIOD->MODER    |=  0xA02A0A0A;       /* Alternate Function mode            */   
  GPIOD->OSPEEDR  &= ~0xF03F0F0F;       /* clear Bits                         */
  GPIOD->OSPEEDR  |=  0xA02A0A0A;       /* 50 MHz Fast speed                  */   
  GPIOD->AFR[0]   &= ~0x00FF00FF;       /* clear Bits                         */
  GPIOD->AFR[0]   |=  0x00CC00CC;       /* Alternate Function mode AF12       */  
  GPIOD->AFR[1]   &= ~0xFF000FFF;       /* clear Bits                         */
  GPIOD->AFR[1]   |=  0xCC000CCC;       /* Alternate Function mode AF12       */  

  /* PE.07(D4), PE.08(D5),  PE.09(D6),  PE.10(D7), PE.11(D8) */
  /* PE.12(D9), PE.13(D10), PE.14(D11), PE.15(D12)           */
  GPIOE->MODER    &= ~0xFFFFC000;       /* clear Bits                         */
  GPIOE->MODER    |=  0xAAAA8000;       /* Alternate Function mode            */
  GPIOE->OSPEEDR  &= ~0xFFFFC000;       /* clear Bits                         */
  GPIOE->OSPEEDR  |=  0xAAAA8000;       /* 50 MHz Fast speed                  */
  GPIOE->AFR[0]   &= ~0xF0000000;       /* clear Bits                         */
  GPIOE->AFR[0]   |=  0xC0000000;       /* Alternate Function mode AF12       */  
  GPIOE->AFR[1]   &= ~0xFFFFFFFF;       /* clear Bits                         */
  GPIOE->AFR[1]   |=  0xCCCCCCCC;       /* Alternate Function mode AF12       */  

  /* PF.00(A0 (RS)) */ 
  GPIOF->MODER   &= ~0x00000003;        /* clear Bits                         */
  GPIOF->MODER   |=  0x00000002;        /* Alternate Function mode            */
  GPIOF->OSPEEDR &= ~0x00000003;        /* clear Bits                         */
  GPIOF->OSPEEDR |=  0x00000002;        /* 50 MHz Fast speed                  */
  GPIOF->AFR[0]  &= ~0x0000000F;        /* clear Bits                         */
  GPIOF->AFR[0]  |=  0x0000000C;        /* Alternate Function mode AF12       */  

  /* PG.10(NE4 (LCD/CS)) - CE1(LCD /CS) */
  GPIOG->MODER    &= ~0x00300000;         /* clear Bits                       */
  GPIOG->MODER    |=  0x00200000;         /* Alternate Function mode          */
  GPIOG->OSPEEDR  &= ~0x00300000;         /* clear Bits                       */
  GPIOG->OSPEEDR  |=  0x00200000;         /* 50 MHz Fast speed                */
  GPIOG->AFR[1]   &= ~0x00000F00;         /* clear Bits                       */
  GPIOG->AFR[1]   |=  0x00000C00;         /* Alternate Function mode AF12     */  

  /* PA.08(LCD Backlight */
  GPIOA->BSRRH    |=  (1UL <<   8);       /* Backlight off                    */
  GPIOA->MODER    &= ~(3UL << 2*8);       /* clear Bits                       */
  GPIOA->MODER    |=  (1UL << 2*8);       /* PI.9 is output                   */
  GPIOA->OTYPER   &= ~(1UL <<   8);       /* PI.9 is output Push-Pull         */
  GPIOA->OSPEEDR  &= ~(3UL << 2*8);       /* clear Bits                       */
  GPIOA->OSPEEDR  |=  (2UL << 2*8);       /* PI.9 is 50MHz Fast Speed         */
//  GPIOA->PUPDR    &= ~(3UL << 2*8);     /* PI.9 is Pull up                    */
//  GPIOA->PUPDR    |=  (1UL << 2*8); 

/*-- FSMC Configuration ------------------------------------------------------*/
/*----------------------- SRAM Bank 3 ----------------------------------------*/
  RCC->AHB3ENR  |= (1UL << 0);                        /* enable FSMC clock */

  FSMC_Bank1->BTCR[(3-1)*2 + 1] =      /* Bank3 NOR/SRAM timing register configuration */
                          (0 << 28) |  /* FSMC AccessMode A */
                          (0 << 24) |  /* Data Latency */
                          (0 << 20) |  /* CLK Division */
                          (0 << 16) |  /* Bus Turnaround Duration */
                          (9 <<  8) |  /* Data SetUp Time */
                          (0 <<  4) |  /* Address Hold Time */
                          (1 <<  0);   /* Address SetUp Time */
  FSMC_Bank1->BTCR[(3-1)*2 + 0] =      /* Control register */
                          (0 << 19) |  /* Write burst disabled */
                          (0 << 15) |  /* Async wait disabled */
                          (0 << 14) |  /* Extended mode disabled */
                          (0 << 13) |  /* NWAIT signal is disabled */ 
                          (1 << 12) |  /* Write operation enabled */
                          (0 << 11) |  /* NWAIT signal is active one data cycle before wait state */
                          (0 << 10) |  /* Wrapped burst mode disabled */
                          (0 <<  9) |  /* Wait signal polarity active low */
                          (0 <<  8) |  /* Burst access mode disabled */
                          (1 <<  4) |  /* Memory data  bus width is 16 bits */
                          (0 <<  2) |  /* Memory type is SRAM */
                          (0 <<  1) |  /* Address/Data Multiplexing disable */
                          (1 <<  0);   /* Memory Bank enable */

  delay(5);                             /* Delay 50 ms                        */
  driverCode = rd_reg(0x00);
  if (driverCode);                      /* avoid warning */

  /* Start Initial Sequence --------------------------------------------------*/
  wr_reg(0x01, 0x0100);                 /* Set SS bit                         */
  wr_reg(0x02, 0x0700);                 /* Set 1 line inversion               */
  wr_reg(0x04, 0x0000);                 /* Resize register                    */
  wr_reg(0x08, 0x0207);                 /* 2 lines front, 7 back porch        */
  wr_reg(0x09, 0x0000);                 /* Set non-disp area refresh cyc ISC  */
  wr_reg(0x0A, 0x0000);                 /* FMARK function                     */
  wr_reg(0x0C, 0x0000);                 /* RGB interface setting              */
  wr_reg(0x0D, 0x0000);                 /* Frame marker Position              */
  wr_reg(0x0F, 0x0000);                 /* RGB interface polarity             */

  /* Power On sequence -------------------------------------------------------*/
  wr_reg(0x10, 0x0000);                 /* Reset Power Control 1              */
  wr_reg(0x11, 0x0000);                 /* Reset Power Control 2              */
  wr_reg(0x12, 0x0000);                 /* Reset Power Control 3              */
  wr_reg(0x13, 0x0000);                 /* Reset Power Control 4              */
  delay(20);                            /* Discharge cap power voltage (200ms)*/
  wr_reg(0x10, 0x12B0);                 /* SAP, BT[3:0], AP, DSTB, SLP, STB   */
  wr_reg(0x11, 0x0007);                 /* DC1[2:0], DC0[2:0], VC[2:0]        */
  delay(5);                             /* Delay 50 ms                        */
  wr_reg(0x12, 0x01BD);                 /* VREG1OUT voltage                   */
  delay(5);                             /* Delay 50 ms                        */
  wr_reg(0x13, 0x1400);                 /* VDV[4:0] for VCOM amplitude        */
  wr_reg(0x29, 0x000E);                 /* VCM[4:0] for VCOMH                 */
  delay(5);                             /* Delay 50 ms                        */
  wr_reg(0x20, 0x0000);                 /* GRAM horizontal Address            */
  wr_reg(0x21, 0x0000);                 /* GRAM Vertical Address              */

  /* Adjust the Gamma Curve --------------------------------------------------*/
  if (driverCode == 0x5408) {           /* LCD with SPFD5408 LCD Controller   */
    wr_reg(0x30, 0x0B0D);
    wr_reg(0x31, 0x1923);
    wr_reg(0x32, 0x1C26);
    wr_reg(0x33, 0x261C);
    wr_reg(0x34, 0x2419);
    wr_reg(0x35, 0x0D0B);
    wr_reg(0x36, 0x1006);
    wr_reg(0x37, 0x0610);
    wr_reg(0x38, 0x0706);
    wr_reg(0x39, 0x0304);
    wr_reg(0x3A, 0x0E05);
    wr_reg(0x3B, 0x0E01);
    wr_reg(0x3C, 0x010E);
    wr_reg(0x3D, 0x050E);
    wr_reg(0x3E, 0x0403);
    wr_reg(0x3F, 0x0607);
  }
  else {                                /* LCD with other LCD Controller      */
    wr_reg(0x30, 0x0006);
    wr_reg(0x31, 0x0101);
    wr_reg(0x32, 0x0003);
    wr_reg(0x35, 0x0106);
    wr_reg(0x36, 0x0B02);
    wr_reg(0x37, 0x0302);
    wr_reg(0x38, 0x0707);
    wr_reg(0x39, 0x0007);
    wr_reg(0x3C, 0x0600);
    wr_reg(0x3D, 0x020B);
  }

  /* Set GRAM area -----------------------------------------------------------*/
  wr_reg(0x50, 0x0000);                 /* Horizontal GRAM Start Address      */
  wr_reg(0x51, (HEIGHT-1));             /* Horizontal GRAM End   Address      */
  wr_reg(0x52, 0x0000);                 /* Vertical   GRAM Start Address      */
  wr_reg(0x53, (WIDTH-1));              /* Vertical   GRAM End   Address      */
  if (driverCode == 0x5408)             /* LCD with SPFD5408 LCD Controller   */
    wr_reg(0x60, 0xA700);               /* Gate Scan Line                     */
  else                                  /* LCD with other LCD Controller      */
    wr_reg(0x60, 0x2700);               /* Gate Scan Line                     */
  wr_reg(0x61, 0x0001);                 /* NDL,VLE, REV                       */
  wr_reg(0x6A, 0x0000);                 /* Set scrolling line                 */

  /* Partial Display Control -------------------------------------------------*/
  wr_reg(0x80, 0x0000);
  wr_reg(0x81, 0x0000);
  wr_reg(0x82, 0x0000);
  wr_reg(0x83, 0x0000);
  wr_reg(0x84, 0x0000);
  wr_reg(0x85, 0x0000);

  /* Panel Control -----------------------------------------------------------*/
  wr_reg(0x90, 0x0010);
  wr_reg(0x92, 0x0000);
  wr_reg(0x93, 0x0003);
  wr_reg(0x95, 0x0110);
  wr_reg(0x97, 0x0000);
  wr_reg(0x98, 0x0000);

  /* Set GRAM write direction
     I/D=11 (Horizontal : increment, Vertical : increment)                    */

#if (HORIZONTAL == 1)
  /* AM=1   (address is updated in vertical writing direction)                */
  wr_reg(0x03, 0x1038);
#else 
  /* AM=0   (address is updated in horizontal writing direction)              */
  wr_reg(0x03, 0x1030);
#endif

  wr_reg(0x07, 0x0137);                 /* 262K color and display ON          */

  GPIOA->BSRRL    |=  (1UL <<   8);       /* Backlight off                    */
}


/*******************************************************************************
* Set draw window region                                                       *
*   Parameter:      x:        horizontal position                              *
*                   y:        vertical position                                *
*                   w:        window width in pixel                            *
*                   h:        window height in pixels                          *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_SetWindow (unsigned int x, unsigned int y, unsigned int w, unsigned int h) {

  wr_reg(0x50, x);                      /* Horizontal GRAM Start Address      */
  wr_reg(0x51, x+w-1);                  /* Horizontal GRAM End   Address (-1) */
  wr_reg(0x52, y);                      /* Vertical   GRAM Start Address      */
  wr_reg(0x53, y+h-1);                  /* Vertical   GRAM End   Address (-1) */

  wr_reg(0x20, x);
  wr_reg(0x21, y);
}


/*******************************************************************************
* Set draw window region to whole screen                                       *
*   Parameter:                                                                 *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_WindowMax (void) {

#if (HORIZONTAL == 1)
  GLCD_SetWindow (0, 0, HEIGHT, WIDTH);
#else
  GLCD_SetWindow (0, 0, WIDTH,  HEIGHT);
#endif
}


/*******************************************************************************
* Draw a pixel in foreground color                                             *
*   Parameter:      x:        horizontal position                              *
*                   y:        vertical position                                *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_PutPixel (unsigned int x, unsigned int y) {

#if (HORIZONTAL == 1)
  wr_reg(0x20, y);
  wr_reg(0x21, WIDTH-1-x);
#else
  wr_reg(0x20, x);
  wr_reg(0x21, y);
#endif
  wr_cmd(0x22);
  wr_dat(TextColor);
}


/*******************************************************************************
* Set foreground color                                                         *
*   Parameter:      color:    foreground color                                 *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_SetTextColor (unsigned short color) {

  TextColor = color;
}


/*******************************************************************************
* Set background color                                                         *
*   Parameter:      color:    background color                                 *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_SetBackColor (unsigned short color) {

  BackColor = color;
}


/*******************************************************************************
* Clear display                                                                *
*   Parameter:      color:    display clearing color                           *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_Clear (unsigned short color) {
  unsigned int i;

  GLCD_WindowMax();
  wr_cmd(0x22);
  wr_dat_start();
  for(i = 0; i < (WIDTH*HEIGHT); i++)
    wr_dat_only(color);
  wr_dat_stop();
}


/*******************************************************************************
* Draw character on given position                                             *
*   Parameter:      x:        horizontal position                              *
*                   y:        vertical position                                *
*                   cw:       character width in pixel                         *
*                   ch:       character height in pixels                       *
*                   c:        pointer to character bitmap                      *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_DrawChar_U8 (unsigned int x, unsigned int y, unsigned int cw, unsigned int ch, unsigned char *c) {
  int idx = 0, i, j;

#if (HORIZONTAL == 1)
  x = WIDTH-x-cw;
  GLCD_SetWindow(y, x, ch, cw);
#else
  GLCD_SetWindow(x, y, cw, ch);
#endif
  wr_cmd(0x22);
  wr_dat_start();
  for (j = 0; j < ch; j++) {
#if (HORIZONTAL == 1)
    for (i = cw-1; i >= 0; i--) {
#else
    for (i = 0; i <= cw-1; i++) {
#endif
      if((c[idx] & (1 << i)) == 0x00) {
        wr_dat_only(BackColor);
      } else {
        wr_dat_only(TextColor);
      }
    }
    c++;
  }
  wr_dat_stop();
}


/*******************************************************************************
* Draw character on given position                                             *
*   Parameter:      x:        horizontal position                              *
*                   y:        vertical position                                *
*                   cw:       character width in pixel                         *
*                   ch:       character height in pixels                       *
*                   c:        pointer to character bitmap                      *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_DrawChar_U16 (unsigned int x, unsigned int y, unsigned int cw, unsigned int ch, unsigned short *c) {
  int idx = 0, i, j;

#if (HORIZONTAL == 1)
  x = WIDTH-x-cw;
  GLCD_SetWindow(y, x, ch, cw);
#else
  GLCD_SetWindow(x, y, cw, ch);
#endif
  wr_cmd(0x22);
  wr_dat_start();
  for (j = 0; j < ch; j++) {
#if (HORIZONTAL == 1)
    for (i = cw-1; i >= 0; i--) {
#else
    for (i = 0; i <= cw-1; i++) {
#endif
      if((c[idx] & (1 << i)) == 0x00) {
        wr_dat_only(BackColor);
      } else {
        wr_dat_only(TextColor);
      }
    }
    c++;
  }
  wr_dat_stop();
}


/*******************************************************************************
* Disply character on given line                                               *
*   Parameter:      ln:       line number                                      *
*                   col:      column number                                    *
*                   fi:       font index (0 = 6x8, 1 = 16x24)                  *
*                   c:        ascii character                                  *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_DisplayChar (unsigned int ln, unsigned int col, unsigned char fi, unsigned char c) {

  c -= 32;
  switch (fi) {
    case 0:  /* Font 6 x 8 */
      GLCD_DrawChar_U8 (col *  6, ln *  8,  6,  8, (unsigned char  *)&Font_6x8_h  [c * 8]);
      break;
    case 1:  /* Font 16 x 24 */
      GLCD_DrawChar_U16(col * 16, ln * 24, 16, 24, (unsigned short *)&Font_16x24_h[c * 24]);
      break;
  }
}


/*******************************************************************************
* Disply string on given line                                                  *
*   Parameter:      ln:       line number                                      *
*                   col:      column number                                    *
*                   fi:       font index (0 = 6x8, 1 = 16x24)                  *
*                   s:        pointer to string                                *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_DisplayString (unsigned int ln, unsigned int col, unsigned char fi, unsigned char *s) {

  GLCD_WindowMax();
  while (*s) {
    GLCD_DisplayChar(ln, col++, fi, *s++);
  }
}


/*******************************************************************************
* Clear given line                                                             *
*   Parameter:      ln:       line number                                      *
*                   fi:       font index (0 = 6x8, 1 = 16x24)                  *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_ClearLn (unsigned int ln, unsigned char fi) {
  unsigned char i;
  unsigned char buf[60];

  GLCD_WindowMax();
  switch (fi) {
    case 0:  /* Font 6 x 8 */
      for (i = 0; i < (WIDTH+5)/6; i++)
        buf[i] = ' ';
      buf[i+1] = 0;
      break;
    case 1:  /* Font 16 x 24 */
      for (i = 0; i < (WIDTH+15)/16; i++)
        buf[i] = ' ';
      buf[i+1] = 0;
      break;
  }
  GLCD_DisplayString (ln, 0, fi, buf);
}

/*******************************************************************************
* Draw bargraph                                                                *
*   Parameter:      x:        horizontal position                              *
*                   y:        vertical position                                *
*                   w:        maximum width of bargraph (in pixels)            *
*                   val:      value of active bargraph (in 1/1024)             *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_Bargraph (unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int val) {
  int i,j;

  val = (val * w) >> 10;                /* Scale value                        */
#if (HORIZONTAL == 1)
  x = WIDTH-x-w;
  GLCD_SetWindow(y, x, h, w);
#else
  GLCD_SetWindow(x, y, w, h);
#endif
  wr_cmd(0x22);
  wr_dat_start();
  for (i = 0; i < h; i++) {
#if (HORIZONTAL == 1)
    for (j = w-1; j >= 0; j--) {
#else
    for (j = 0; j <= w-1; j++) {
#endif
      if(j >= val) {
        wr_dat_only(BackColor);
      } else {
        wr_dat_only(TextColor);
      }
    }
  }
  wr_dat_stop();
}


/*******************************************************************************
* Display graphical bitmap image at position x horizontally and y vertically   *
* (This function is optimized for 16 bits per pixel format, it has to be       *
*  adapted for any other bits per pixel format)                                *
*   Parameter:      x:        horizontal position                              *
*                   y:        vertical position                                *
*                   w:        width of bitmap                                  *
*                   h:        height of bitmap                                 *
*                   bitmap:   address at which the bitmap data resides         *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_Bitmap (unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned char *bitmap) {
  unsigned int    i, j;
  unsigned short *bitmap_ptr = (unsigned short *)bitmap;

#if (HORIZONTAL == 1)
  x = WIDTH-x-w;
  GLCD_SetWindow(y, x, h, w);
#else
  GLCD_SetWindow(x, y, w, h);
#endif
  wr_cmd(0x22);
  wr_dat_start();
  for (j = 0; j < h; j++) {
#if (HORIZONTAL == 1)
    for (i = 0; i < w; i++) {
      wr_dat_only(*bitmap_ptr++);
    }
#else
    bitmap_ptr += w-1;
    for (i = 0; i < w; i++) {
      wr_dat_only(*bitmap_ptr--);
    }
    bitmap_ptr += w+1;
#endif
  }
  wr_dat_stop();
}


/*******************************************************************************
* Display graphical bmp file image at position x horizontally and y vertically *
* (This function is optimized for 16 bits per pixel format, it has to be       *
*  adapted for any other bits per pixel format)                                *
*   Parameter:      x:        horizontal position                              *
*                   y:        vertical position                                *
*                   w:        width of bitmap                                  *
*                   h:        height of bitmap                                 *
*                   bmp:      address at which the bmp data resides            *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_Bmp (unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned char *bmp) {
  unsigned int    i, j;
  unsigned short *bitmap_ptr = (unsigned short *)bmp;

#if (HORIZONTAL == 1)
  x = WIDTH-x-w;
  GLCD_SetWindow(y, x, h, w);
#else
  GLCD_SetWindow(x, y, w, h);
#endif
  wr_cmd(0x22);
  wr_dat_start();
#if (HORIZONTAL == 1)
  bitmap_ptr += (h*w)-1;
  for (j = 0; j < h; j++) {
    for (i = 0; i < w; i++) {
      wr_dat_only(*bitmap_ptr--);
    }
  }
#else
  bitmap_ptr += ((h-1)*w);
  for (j = 0; j < h; j++) {
    for (i = 0; i < w; i++) {
      wr_dat_only(*bitmap_ptr++);
    }
    bitmap_ptr -= 2*w;
  }
#endif
  wr_dat_stop();
}


/*******************************************************************************
* Scroll content of the whole display for dy pixels vertically                 *
*   Parameter:      dy:       number of pixels for vertical scroll             *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_ScrollVertical (unsigned int dy) {
#if (HORIZONTAL == 0)
  static unsigned int y = 0;

  y = y + dy;
  while (y >= HEIGHT) 
    y -= HEIGHT;

  wr_reg(0x6A, y);
  wr_reg(0x61, 3);
#endif
}

/******************************************************************************/
