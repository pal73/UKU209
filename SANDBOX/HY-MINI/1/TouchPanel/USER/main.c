/****************************************Copyright (c)****************************************************
**                                      
**                                 http://www.powermcu.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               main.c
** Descriptions:            The TouchPanel application function
**
**--------------------------------------------------------------------------------------------------------
** Created by:              AVRman
** Created date:            2010-11-7
** Version:                 v1.0
** Descriptions:            The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:           
** Version:                 
** Descriptions:            
**
*********************************************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "TouchPanel.h"
#include "systick.h"
#include "GLCD.h"

/*******************************************************************************
* Function Name  : Delay
* Description    : Delay Time
* Input          : - nCount: Delay Time
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void  Delay (uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}

/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configure GPIO Pin
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB , ENABLE); 						 
/**
 *	LED1 -> PB0   LED2 -> PB1
 */					 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/*******************************************************************************
* Function Name  : main
* Description    : Main program
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
int main(void)
{
  LCD_Initializtion();
  LCD_BackLight_Init();

  GPIO_Configuration();
  delay_init();
  TP_Init();
  //TouchPanel_Calibrate();
  /* Infinite loop */
  LCD_BackLight(10);

  while (1)	
  {
    //getDisplayPoint(&display, Read_Ads7846(), &matrix ) ;
    //TP_DrawPoint(display.x,display.y);
		/* LED1-ON LED2-OFF */
		GPIO_SetBits(GPIOB , GPIO_Pin_0);
		GPIO_ResetBits(GPIOB , GPIO_Pin_1);
		Delay(0xfffff);
		Delay(0xfffff);
		Delay(0x5ffff);	

		/* LED1-OFF LED2-ON */
		GPIO_ResetBits(GPIOB , GPIO_Pin_0);
		GPIO_SetBits(GPIOB , GPIO_Pin_1);
		Delay(0xfffff);
		Delay(0xfffff);
		Delay(0x5ffff);
  }
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif


/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
