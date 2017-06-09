
#include "stm32f2xx.h" 

//***********************************************
//***********************************************
//***********************************************
//***********************************************
//***********************************************
void SysTick_Handler (void) 	 /* SysTick Interrupt Handler (1ms)    */
{

GPIOC->ODR^= 0xFFFFFFFF;
}


void main (void)
{
SysTick_Config(SystemCoreClock/100);  /* Generate interrupt each 10 ms      */

RCC->AHB1ENR  |= ((1UL <<  2) |       /* Enable GPIOC clock                 */
                    (1UL <<  6) |       /* Enable GPIOG clock                 */
                    (1UL <<  8)  );     /* Enable GPIOI clock                 */

GPIOC->MODER  = 0x55555555;

while(1)
	{
	}
};