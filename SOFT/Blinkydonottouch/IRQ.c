#include "stm32f2xx.h"                   
#include "main.h" 

extern short sycTickCnt;

volatile unsigned short AD_last;          /* Last converted value             */
volatile unsigned int   AD_avg = 0;
volatile unsigned char  clock_1s;         /* Flag activated each second       */

/* Import function for turning LEDs on or off                                 */
extern void LED_Out (unsigned int num);


/*----------------------------------------------------------------------------
  SysTick IRQ: Executed periodically (10ms) 
 *----------------------------------------------------------------------------*/
void SysTick_Handler (void) {
  static unsigned long ticks;
  static unsigned long timetick;
  static unsigned char leds = 0x01;


  if (ticks++ >= 99) {                    /* Set Clock1s to 1 every 1 second  */
    ticks    = 0;
    clock_1s = 1;
  }

  /* Blink the LEDs depending on ADC_ConvertedValue                           */
  if (timetick++ >= (AD_last >> 8)) {
    timetick   = 0;
    leds     <<= 1;
    if (leds == 0) leds = 0x01;
    LED_Out (leds);
  }

  ADC1->CR2    |=  1 << 30;               /* Start conversion                 */ 

b1000Hz=1;

if(++t0cnt0>=10)
     {
     t0cnt0=0;
     b100Hz=1;
     }

if(++t0cnt1>=20)
     {
     t0cnt1=0;
     b50Hz=1;
     }
     
if(++t0cnt2>=100)
	{
    t0cnt2=0;
    b10Hz=1;
	 
	if(++t0cnt3>=5)
		{
	 	t0cnt3=0;
	    b2Hz=1;
		}        

	if(++t0cnt4>=10)
	    {
	    t0cnt4=0;
	    b1Hz=1;
     	}
	}
}


#ifdef __USE_IRQ
/*----------------------------------------------------------------------------
  calculate an average over 16 AD values
 *----------------------------------------------------------------------------*/
static unsigned short CalcAverage (unsigned short val)  {
  static unsigned short idx      =  0;
  static unsigned short aval[32] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  static unsigned int   sum      =  0;

  sum = sum - aval[idx] + val;
  aval[idx] = val;         
  idx = (idx +1 ) & 0x1F;

  return (sum >> 5);
}


/*----------------------------------------------------------------------------
  ADC Interrupt Handler
 *----------------------------------------------------------------------------*/
void ADC_IRQHandler (void) {

  if (ADC1->SR & (1<<1)) {                /* ADC1 EOC interrupt?              */
//    ADC_ConvertedValue = ADC1->DR;
    AD_last  = CalcAverage (ADC1->DR);    /* calculate average                */

    ADC1->SR &= ~(1<<1);                  /* clear EOC interrupt              */
  }

  b1000Hz=1;



}
#endif //__USE_IRQ
