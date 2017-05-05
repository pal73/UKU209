/*
*********************************************************************************************************
*                                                uC/GUI
*                        Universal graphic software for embedded applications
*
*                       (c) Copyright 2002, Micrium Inc., Weston, FL
*                       (c) Copyright 2002, SEGGER Microcontroller Systeme GmbH
*
*              �C/GUI is protected by international copyright laws. Knowledge of the
*              source code may not be used to write a similar product. This file may
*              only be used in accordance with a license and should not be redistributed
*              in any way. We appreciate your understanding and fairness.
*
----------------------------------------------------------------------
File        : LCDPM444_12.c
Purpose     : Color conversion routines for LCD-drivers
---------------------------END-OF-HEADER------------------------------
*/

#include "LCD_Protected.h"    /* inter modul definitions */

/*********************************************************************
*
*       Public code,
*
*       LCD_FIXEDPALETTE == 444, 4096 colors, RRRRGGGGBBBB
*
**********************************************************************
*/
/*********************************************************************
*
*       LCD_Color2Index_M444_12
*/
unsigned LCD_Color2Index_M444_12(LCD_COLOR Color) {
  unsigned int r,g,b;
  r = Color         & 255;
  g = (Color >> 8)  & 255;
  b = (Color >> 16) & 255;
  r = (r + 8) / 17;
  g = (g + 8) / 17;
  b = (b + 8) / 17;
  return b + (g << 4) + (r << 8);
}

/*********************************************************************
*
*       LCD_Index2Color_M444_12
*/
LCD_COLOR LCD_Index2Color_M444_12(int Index) {
  unsigned int r,g,b;
  /* Seperate the color masks */
  b = Index & 0xf;
  g = (Index >> 4) & 0xf;
  r = ((unsigned)Index >> 8) & 0xf;
  /* Convert the color masks */
  r = r * 17;
  g = g * 17;
  b = b * 17;
  return r + (g<<8) + (((U32)b)<<16);
}

/*********************************************************************
*
*       LCD_GetIndexMask_M444_12
*/
unsigned LCD_GetIndexMask_M444_12(void) {
  return 0x0fff;
}

/*************************** End of file ****************************/
