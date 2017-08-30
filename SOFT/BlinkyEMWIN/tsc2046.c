#include "stm32f2xx.h"                  /* STM32F2xx Definitions              */
#include "GLCD.h"
#include "main.h"
#include "tsc2046.h"

int d_in_x;
int d_in_y;

static void DelayUS(uint32_t cnt)
{
  uint16_t i;
  for(i = 0;i<cnt;i++)
  {
     uint8_t us = 12; /* ÉèÖÃÖµÎª12£¬´óÔ¼ÑÓ1Î¢Ãë */    
     while (us--)     /* ÑÓ1Î¢Ãë	*/
     {
       ;   
     }
  }
}

//-----------------------------------------------
void tsc2046_init(void)
{
/* Configure the LCD Control pins --------------------------------------------*/
RCC->AHB1ENR  |= (1UL <<  0);       /* Enable GPIOA clock                 */


//in-out config
//PA5 - SCL     - Alternative func. push-pull     
//PA6 - MOSI Alternative func.  push-pull- OUT  
//PA7 - MISO Input floating / Input pull-up - 
//PA3 - CS - GPIO - soft        
GPIOA->MODER &= 0xffff033f;
GPIOA->MODER |= 0x0000a840;
GPIOA->OTYPER &=0x00000000;

GPIOA->AFR[0]&=~(0x00000fff<<(4*5));
GPIOA->AFR[0]|=(0x00000555<<(4*5));

//SPI1 CR1 configure
RCC->APB2ENR |= (1<<12);     				//Enable oscil SPI1		RCC_APB2ENR_SPI1EN;

SPI1->CR1 |= SPI_CR1_BR;                	//Baud rate = Fpclk/256
SPI1->CR1 &= ~SPI_CR1_CPOL;             	//Polarity cls signal CPOL = 0;
SPI1->CR1 &= ~SPI_CR1_CPHA;             	//Phase cls signal    CPHA = 0;
SPI1->CR1 &= ~SPI_CR1_DFF;              	//8 bit data
SPI1->CR1 &= ~SPI_CR1_LSBFIRST;         	//MSB will be first
SPI1->CR1 |= SPI_CR1_SSM;               	// Program mode NSS
SPI1->CR1 |= SPI_CR1_SSI;               	// àííàëîãè÷íî ñîñòîÿíèþ, êîãäà NSS 1
SPI1->CR2 &= ~SPI_CR2_SSOE;              	//âûâîä NSS -  âûêëþ÷åí
SPI1->CR1 |= SPI_CR1_MSTR;              	//Mode Master
SPI1->CR1 |= SPI_CR1_SPE;               	//Enable SPI1
SPI1->CR1 &= ~SPI_CR1_BIDIMODE;               
SPI1->CR1 |= SPI_CR1_BIDIOE;                     

}

//-----------------------------------------------
void tsc2046_drv(void)
{
int d_in;                //zero data in
//CS_TOUCH_ON;             //chip select TOUCH
GPIOA->ODR&=~(1<<3);
DelayUS(10);
SPI1->DR = 0x90;//command;                   //send command
while(!(SPI1->SR & SPI_SR_RXNE));       //wait
d_in = SPI1->DR;                        //ignore this enter
SPI1->DR = 0;      //send zero bytes - cont receiving data from module
while(!(SPI1->SR & SPI_SR_RXNE));  //wait
d_in = SPI1->DR;        //receive high byte
d_in <<= 8;             //move it 8 
SPI1->DR = 0;           //send zero bytes - cont receiving data from module
while(!(SPI1->SR & SPI_SR_RXNE));       //wait
d_in |= SPI1->DR;        //receive low byte
//CS_TOUCH_OFF;
//Delay (10);
d_in_x=d_in>>4;
GPIOA->ODR|=(1<<3);
//return d_in;

GPIOA->ODR&=~(1<<3);
//Delay (10);
SPI1->DR = 0xD0;//command;                   //send command
while(!(SPI1->SR & SPI_SR_RXNE));       //wait
d_in = SPI1->DR;                        //ignore this enter
SPI1->DR = 0;      //send zero bytes - cont receiving data from module
while(!(SPI1->SR & SPI_SR_RXNE));  //wait
d_in = SPI1->DR;        //receive high byte
d_in <<= 8;             //move it 8 
SPI1->DR = 0;           //send zero bytes - cont receiving data from module
while(!(SPI1->SR & SPI_SR_RXNE));       //wait
d_in |= SPI1->DR;        //receive low byte
//CS_TOUCH_OFF;
//Delay (10);
d_in_y=d_in>>4;
GPIOA->ODR|=(1<<3);	
}