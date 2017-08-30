/**
  ******************************************************************************
  * @file    menu.h
  * @author  MCD Application Team
  * @version V2.0.0
  * @date    11-July-2011
  * @brief   This file contains the functions prototypes for the menu section.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MENU_H
#define __MENU_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "graphicObjectTypes.h"

/* Exported types ------------------------------------------------------------*/
typedef enum {
    MAN = 0, 
    WOMAN
}GenderType;
/* Exported constants --------------------------------------------------------*/
#define PAGE_HOME       0xFF
#define PAGE_0          0x00
#define PAGE_1          0x01
#define PAGE_2          0x02
#define PAGE_3          0x03
#define PAGE_4          0x04
#define PAGE_5          0x05
#define PAGE_6          0x06
#define PAGE_7          0x07
#define PAGE_8          0x08
#define PAGE_9          0x09
#define PAGE_LIBRARY    0xFE
#define PAGE_D0         0x10
#define PAGE_D1         0x11
#define PAGE_D2         0x12
#define PAGE_D3         0x13
#define PAGE_D4         0x14
#define PAGE_D5         0x15
#define PAGE_D6         0x16
#define PAGE_D7         0x17
#define PAGE_DEMO       0x18
/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */ 
void Show_HomeScreen(void);


#ifdef __cplusplus
}
#endif

#endif /* __MENU_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

