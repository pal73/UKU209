

#include <stdio.h>
#include "stm32f2xx.h"                  /* STM32F2xx Definitions              */
#include "GLCD.h"
#include "main.h"
#include "tsc2046.h"

#define __FI        1                   /* Font index 16x24                   */
#if (__FI == 1)                         /* Font index  6x8                    */                         
  #define __FONT_WIDTH  16
  #define __FONT_HEIGHT 24
#else                                   /* Font index 16x24                   */
  #define __FONT_WIDTH   6
  #define __FONT_HEIGHT  8
#endif


#define SSD1963_SetArea	  GLCD_SetWindow
#define	SSD1963_WriteCommand  wr_cmd
#define	SSD1963_WriteData	wr_dat

#define LED_NUM     4                   /* Number of user LEDs                */
const unsigned long led_mask[] = { 1UL<<6, 1UL<<8, 1UL<<9, 1UL<< 7 };

               char  text[40];
			   short plazma;
/* Import external functions from Serial.c file                               */
extern void SER_init (void);

/* Import external variables from IRQ.c file                                  */
extern volatile unsigned short AD_last;
extern volatile unsigned char  clock_1s;

/* variable to trace in LogicAnalyzer (should not read to often)              */
       volatile unsigned short AD_dbg;          


char b1Hz=0;
char b1000Hz=0;
short sycTickCnt=0;


extern  __inline void wr_cmd (unsigned char cmd) ;

extern  __inline void wr_dat (unsigned short dat) ;


//***********************************************
//Таймер
char b10000Hz,b1000Hz,b2000Hz,b100Hz,b50Hz,b10Hz,b5Hz,b2Hz,b1Hz,b1min;
short t0cnt,t0cnt0,t0cnt1,t0cnt2,t0cnt3,t0cnt4,t0cnt5,t0cnt6,t0_cnt7,t0_cnt_min,t0cntMin;
char bFL5,bFL2,bFL,bFL_,bTPS;


/*----------------------------------------------------------------------------
  Function that initializes ADC
 *----------------------------------------------------------------------------*/
void ADC_init (void) {

  /* Setup and initialize ADC converter                                       */
  RCC->APB2ENR  |=  (1UL <<  8);        /* Enable ADC1 clock                  */
  RCC->AHB1ENR  |=  (1UL <<  2);        /* Enable GPIOC clock                 */
  GPIOC->MODER  |=  (3UL << 2*2);       /* PC.2 is Analog mode                */
  
#ifndef __USE_IRQ
  /* DMA2 stream0 configuration ----------------------------------------------*/
  RCC->AHB1ENR  |=  (1UL << 22);        /* Enable DMA2 clock                  */

  DMA2_Stream0->M0AR  = (uint32_t)&AD_last;    /* Set memory     address      */
  DMA2_Stream0->PAR   = (uint32_t)&(ADC1->DR); /* Set peripheral address      */
  DMA2_Stream0->NDTR  = 1;              /* Transmit 1 data item               */
  DMA2_Stream0->CR    = 0x00022900;     /* configure DMA stream               */
  DMA2_Stream0->CR   |= (1UL << 0);     /* Enable stream                      */
#endif //__USE_IRQ

  ADC1->SQR1   =   0;           
  ADC1->SQR2   =   0;           
  ADC1->SQR3   =  (12UL <<  0);         /* SQ1 = channel 12                   */
  ADC1->SMPR1  =  ( 7UL <<  6);         /* Channel 12 sample time is 480 cyc. */
  ADC1->SMPR2  =   0;                   /* Clear register                     */
  ADC1->CR1    =  ( 1UL <<  8);         /* Scan mode on                       */

  ADC1->CR2   |=  ( 1UL <<  3);         /* Initialize calibration registers   */
  while (ADC1->CR2 & (1UL << 3));       /* Wait for initialization to finish  */
  ADC1->CR2   |=  ( 1UL <<  2);         /* Start calibration                  */
  while (ADC1->CR2 & (1UL << 2));       /* Wait for calibration to finish     */

#ifndef __USE_IRQ
  ADC1->CR2   |=  ( 1UL <<  8);         /* DMA mode enable                    */
  ADC1->CR2   |=  ( 1UL <<  9);         /* continuous DMA requests            */
#else
  ADC1->CR1   |=  ( 1UL <<  5);         /* enable EOC interrupt               */
  NVIC_EnableIRQ(ADC_IRQn);             /* enable ADC Interrupt               */
#endif //__USE_IRQ

  ADC1->CR2   |=  ( 1UL <<  0);         /* ADC enable                         */
}


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
  unsigned short AD_value, AD_print = 0;

  SysTick_Config(SystemCoreClock/1000);  /* Generate interrupt each 1 ms      */

  //LED_init();                           /* LED Initialization                 */
  //SER_init();                           /* UART#1 Initialization              */
  //ADC_init();                           /* ADC Initialization                 */
RCC->AHB1ENR  |= ((1UL <<  2) |       /* Enable GPIOC clock                 */
                    (1UL <<  6) |       /* Enable GPIOG clock                 */
                    (1UL <<  8)  );     /* Enable GPIOI clock                 */
GPIOC->MODER  = 0x55555555;

#ifdef __USE_LCD
  GLCD_Init();                          /* Initialize graphical LCD display   */

  GLCD_Clear(Cyan);                    /* Clear graphical LCD display        */
  GLCD_SetBackColor(Olive);
  GLCD_SetTextColor(White);
  //GLCD_DisplayChar(0,0,1,'c');
  
  //GLCD_DisplayString(0, 0, __FI, " STM3220F-EVAL Dem1 "); 
  //GLCD_DisplayString(0, 0, __FI, "1234567");
  //GLCD_DisplayString(1, 0, __FI, "       Blinky       ");
  //GLCD_DisplayString(2, 0, __FI, "    www.keil.com    ");
  //GLCD_SetBackColor(White);
  //GLCD_SetTextColor(Blue);
#endif // __USE_LCD

tsc2046_init();

while (1) 

	{
	if(b1000Hz)
		{
		b1000Hz=0;
		GPIOC->ODR^= 0xffff;
		}

	if(b100Hz)
		{
		b100Hz=0;
		tsc2046_drv();
		//GPIOA->ODR^= 0xffff;
		}
	if(b10Hz)
		{
		b10Hz=0;		
		sprintf(text, "%7d", d_in_x);
		GLCD_DisplayString(0, 0, __FI,  (unsigned char *)text);
		sprintf(text, "%7d", d_in_y);
		GLCD_DisplayString(1, 0, __FI,  (unsigned char *)text);
		}   
	if(b1Hz)
		{
		b1Hz=0; 
		
		

		sprintf(text, "ddad = %4d", plazma++);
		GLCD_DisplayString(5, 5, __FI,  (unsigned char *)text);

		}
  	}
}


		//GLCD_Clear(Olive);
		
	/*	{
  		unsigned int x,y;
  		SSD1963_SetArea(50, 100 , 100, 150);
  		SSD1963_WriteCommand(0x002c);
  		for(x=0;x<6000;x++)
			{
      		SSD1963_WriteData(Blue);
   			}
		} */

/*		{
  		unsigned int x,y;
  		SSD1963_SetArea(101, 151 , 50, 100);
  		SSD1963_WriteCommand(0x002c);
  		for(x=0;x<2600;x++)
			{
      		SSD1963_WriteData(0xf800);
  			} 
		} */
		
//		}                      /* Loop forever                       */
    //AD_value = AD_last;                 /* Read AD_last value                 */
    //if (AD_value != AD_last)            /* Make sure that AD interrupt did    */
      //AD_value = AD_last;               /* not interfere with value reading   */

    //if (AD_value != AD_print) {        /* Make sure that AD interrupt did    */
#ifdef __USE_LCD
      //GLCD_SetTextColor(Red);
      //GLCD_Bargraph (9 * __FONT_WIDTH, 6 * __FONT_HEIGHT, 10 * __FONT_WIDTH, __FONT_HEIGHT - 2, (AD_value>>2));
      //GLCD_SetTextColor(White);
#endif // __USE_LCD

      //AD_print = AD_value;              /* Get unscaled value for printout    */
      //AD_dbg   = AD_value;
    //}

    /* Printf message with AD value to serial port every 1 second */
    /////if (clock_1s) {
      /////clock_1s = 0;
      /////sprintf(text, "AD value = 0x%04X", AD_print);

#ifdef __USE_LCD
      //GLCD_SetTextColor(Blue);
      /////GLCD_DisplayString(5, 0, __FI,  (unsigned char *)text);
#endif // __USE_LCD
      /////printf("%s\r\n", text);
    /////}