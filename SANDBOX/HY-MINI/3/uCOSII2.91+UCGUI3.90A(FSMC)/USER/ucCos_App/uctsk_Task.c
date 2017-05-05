/****************************************Copyright (c)****************************************************
**                                      
**                                 http://www.powermcu.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               uctsk_Task.c
** Descriptions:            The uctsk_Task application function
**
**--------------------------------------------------------------------------------------------------------
** Created by:              AVRman
** Created date:            2010-11-24
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
#include <includes.h>            



/* Private variables ---------------------------------------------------------*/
static  OS_STK         App_Task_UCGUI_Stk        [APP_TASK_UCGUI_STK_SIZE];
static  OS_STK         App_Task_TouchPanel_Stk   [APP_TASK_TouchPanel_STK_SIZE];
static  OS_STK         App_TaskBlinkStk          [APP_TASK_BLINK_STK_SIZE];

/* Private function prototypes -----------------------------------------------*/
       void MainTask         (void);
static void uctsk_UCGUI      (void);
static void uctsk_TouchPanel (void);
static void uctsk_Blink      (void);
extern void _ExecCalibration (void);
 
void  App_UCGUI_TaskCreate (void)
{
    CPU_INT08U  os_err;

	os_err = os_err; 

	os_err = OSTaskCreate((void (*)(void *)) uctsk_UCGUI,				
                          (void          * ) 0,							
                          (OS_STK        * )&App_Task_UCGUI_Stk[APP_TASK_UCGUI_STK_SIZE - 1],		
                          (INT8U           ) APP_TASK_UCGUI_PRIO  );							

	#if OS_TASK_NAME_EN > 0
    	OSTaskNameSet(APP_TASK_UCGUI_PRIO, "Task UCGUI", &os_err);
	#endif
   
    os_err = OSTaskCreate((void (*)(void *)) uctsk_TouchPanel,				
                          (void          * ) 0,							
                          (OS_STK        * )&App_Task_TouchPanel_Stk[APP_TASK_TouchPanel_STK_SIZE - 1],		
                          (INT8U           ) APP_TASK_TouchPanel_PRIO  );

	#if OS_TASK_NAME_EN > 0
    	OSTaskNameSet(APP_TASK_TouchPanel_PRIO, "TASK TouchPanel Messages", &os_err);
	#endif


	os_err = OSTaskCreate((void (*)(void *)) uctsk_Blink,				
                          (void          * ) 0,							
                          (OS_STK        * )&App_TaskBlinkStk[APP_TASK_BLINK_STK_SIZE - 1],		
                          (INT8U           ) APP_TASK_BLINK_PRIO  );							

	#if OS_TASK_NAME_EN > 0
    	OSTaskNameSet(APP_TASK_BLINK_PRIO, "Task LED Blink", &os_err);
	#endif


}							 

static void uctsk_UCGUI (void) 
{   
   GUI_Init();
   _ExecCalibration();	  /* ´¥ÃþÆÁÐ£×¼   */
   	for(;;)
   	{  
	  MainTask(); 
    }
}

static void uctsk_Blink (void) 
{                 
    for(;;)
   	{  
     	OSTimeDlyHMSM(0, 0, 0, 250);	 /* 500 MS  */
		/* LED1-ON LED2-OFF */
		GPIO_SetBits(GPIOB , GPIO_Pin_0);
		GPIO_ResetBits(GPIOB , GPIO_Pin_1);

      	OSTimeDlyHMSM(0, 0, 0, 500);	 /* 500 MS  */
	    /* LED1-OFF LED2-ON */
		GPIO_ResetBits(GPIOB , GPIO_Pin_0);
		GPIO_SetBits(GPIOB , GPIO_Pin_1);


    }
}

static void uctsk_TouchPanel (void) 
{  
    TP_Init();			             /* ´¥ÃþÆÁ³õÊ¼»¯ */ 
    
   	for(;;)
   	{  
	   GUI_TOUCH_Exec();
	   OSTimeDlyHMSM(0, 0, 0, 20);	 /* 20 MS  */
    }
}

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
