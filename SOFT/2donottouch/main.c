
#include "stm32f2xx.h"

#define DATAPIXELWIDTH 	16
#define TFT_WIDTH		320
#define TFT_HEIGHT		240

#define LCD_RST_PORT 	GPIOF
#define LCD_RST		 	1
#define LCD_RDS_PORT 	GPIOF
#define LCD_RDS		 	0
#define LCD_WR_PORT 	GPIOD
#define LCD_WR		 	5
#define LCD_RD_PORT 	GPIOD
#define LCD_RD		 	4
#define LCD_CS_PORT 	GPIOG
#define LCD_CS		 	10

#define SET_LCD_RDS 		LCD_RDS_PORT->BSRRL = (1<<LCD_RDS)
#define RESET_LCD_RDS 		LCD_RDS_PORT->BSRRH = (1<<LCD_RDS)
#define SET_LCD_CS 			LCD_CS_PORT->BSRRL = (1<<LCD_CS)
#define RESET_LCD_CS 		LCD_CS_PORT->BSRRH = (1<<LCD_CS)
#define SET_LCD_RD 			LCD_RD_PORT->BSRRL = (1<<LCD_RD)
#define RESET_LCD_RD 		LCD_RD_PORT->BSRRH = (1<<LCD_RD)
#define SET_LCD_WR 			LCD_WR_PORT->BSRRL = (1<<LCD_WR)
#define RESET_LCD_WR 		LCD_WR_PORT->BSRRH = (1<<LCD_WR)
#define SET_LCD_RST 		LCD_RST_PORT->BSRRL = (1<<LCD_RST)
#define RESET_LCD_RST 		LCD_RST_PORT->BSRRH = (1<<LCD_RST)

char b1Hz=0;
char b1000Hz=0;
short sycTickCnt=0;
short short_plazma;

void SSD1963_WriteCommand(uint16_t commandToWrite);
void SSD1963_WriteData(uint16_t dataToWrite);
void delay_ms(uint16_t del);

void lcd_port_init(void)
{
RCC->AHB1ENR  |= (	(1UL <<  3) |       // Enable GPIOD clock                 
                    (1UL <<  4) |       // Enable GPIOE clock                 
					(1UL <<  5) |       // Enable GPIOF clock                 
                    (1UL <<  6)  );     // Enable GPIOG clock 
					
GPIOD->MODER=	0x50150505;
GPIOD->OTYPER=	0x00000000; 
GPIOD->OSPEEDR=	0xffffffff; 

GPIOE->MODER=	0x55554000;
GPIOE->OTYPER=	0x00000000; 
GPIOE->OSPEEDR=	0xffffffff; 

GPIOF->MODER=	0x00000005;
GPIOF->OTYPER=	0x00000000; 
GPIOF->OSPEEDR=	0xffffffff; 

GPIOG->MODER=	0x00100000;
GPIOG->OTYPER=	0x00000000; 
GPIOG->OSPEEDR=	0xffffffff;               
}



void SSD1963_Init (void)
{
  uint16_t  HDP=799;
  uint16_t  HT=928;
  uint16_t  HPS=46;
  uint16_t  LPS=15;
  uint8_t   HPW=48;

  uint16_t  VDP=479;
  uint16_t  VT=525;
  uint16_t  VPS=16;
  uint16_t  FPS=8;
  uint8_t   VPW=16;

  RESET_LCD_RST;
  delay_ms(5);
  SET_LCD_RST;
  delay_ms(5);


  SSD1963_WriteCommand(0x0001);  // software reset
  delay_ms(5);
  SSD1963_WriteCommand(0x00E0);
  SSD1963_WriteData(0x0001);
  delay_ms(3);
  SSD1963_WriteCommand(0x00E0);
  SSD1963_WriteData(0x0003);
  delay_ms(3);


	SSD1963_WriteCommand(0xb0);  //SET LCD MODE  SET TFT 18Bits MODE
	SSD1963_WriteData(0x20);   //SET TFT MODE & hsync+Vsync+DEN MODE
	SSD1963_WriteData(0x80);   //SET TFT MODE & hsync+Vsync+DEN MODE
	SSD1963_WriteData(0x01);   //SET horizontal size=320-1 HightByte
	SSD1963_WriteData(0x3f);      //SET horizontal size=320-1 LowByte
	SSD1963_WriteData(0x00);   //SET vertical size=240-1 HightByte
	SSD1963_WriteData(0xef);   //SET vertical size=240-1 LowByte
	SSD1963_WriteData(0x00);   //SET even/odd line RGB seq.=RGB
 
	SSD1963_WriteCommand(0xf0);  //SET LCD MODE  SET TFT 18Bits MODE
	SSD1963_WriteData(0x03);   //SET TFT MODE & hsync+Vsync+DEN MODE
	
	SSD1963_WriteCommand(0x36);  //SET LCD MODE  SET TFT 18Bits MODE
	SSD1963_WriteData(0x08);   //SET TFT MODE & hsync+Vsync+DEN MODE
	 
	SSD1963_WriteCommand(0xe2);  //SET LCD MODE  SET TFT 18Bits MODE
	SSD1963_WriteData(0x1d);   //SET TFT MODE & hsync+Vsync+DEN MODE		
	SSD1963_WriteData(0x02);			
	SSD1963_WriteData(0x54);		

	SSD1963_WriteCommand(0xe6);  //SET LCD MODE  SET TFT 18Bits MODE
	SSD1963_WriteData(0x01);   //SET TFT MODE & hsync+Vsync+DEN MODE
	SSD1963_WriteData(0xdd);		//ce	
	SSD1963_WriteData(0xde);		//94
 
	SSD1963_WriteCommand(0xb4);		//SET HSYNC 
	SSD1963_WriteData(0x01);			
	SSD1963_WriteData(0x98);			//SET HT = 408(10)=0198(16)
	SSD1963_WriteData(0x00);			
	SSD1963_WriteData(0x44);			//SET HBP = 68(10)=44(16)
	SSD1963_WriteData(0x14);			//SET HPW = 20(10)=14(16)
	SSD1963_WriteData(0x00);			//SET LPS = 0
	SSD1963_WriteData(0x00);
	SSD1963_WriteData(0x00);			
	
	SSD1963_WriteCommand(0xb6); 		//SET VSYNC
	SSD1963_WriteData(0x01);			
	SSD1963_WriteData(0x06);			//SET HT = 262(10)=408(!6)
	SSD1963_WriteData(0x00);			
	SSD1963_WriteData(0x12);			//SET VBP = 18(10)=18(16)
	SSD1963_WriteData(0x04);			//SET VPW = 4(10)=4(16)PS = 0Vsync pulse 8 = 7 + 1
	SSD1963_WriteData(0x00);			//SET FPS = 0
	SSD1963_WriteData(0x00);
 
	SSD1963_WriteCommand(0x2a);  //SET column address
	SSD1963_WriteData(0x00);   //SET start column address=0
	SSD1963_WriteData(0x00);
	SSD1963_WriteData(0x01);   //SET end column address=320
	SSD1963_WriteData(0x3f);
 
	SSD1963_WriteCommand(0x2b);  //SET page address
	SSD1963_WriteData(0x00);   //SET start page address=0
	SSD1963_WriteData(0x00);
	SSD1963_WriteData(0x00);   //SET end page address=240
	SSD1963_WriteData(0xef);
 
	SSD1963_WriteCommand(0xb8);   //SET GPIO
	SSD1963_WriteData(0x0f);      //SET I/O
	SSD1963_WriteData(0x01);
	SSD1963_WriteCommand(0xba);   //SET GPIO
    SSD1963_WriteData(0x01);      //SET I/O

 
	SSD1963_WriteCommand(0x29);  //SET display on
	SSD1963_WriteCommand(0x2c);

/*
  SSD1963_WriteCommand(0x00E2);     //PLL multiplier, set PLL clock to 120M
  SSD1963_WriteData(0x0023);        //N=0x36 for 6.5M, 0x23 for 10M crystal
  SSD1963_WriteData(0x0002);
  SSD1963_WriteData(0x0004);
  SSD1963_WriteCommand(0x00E0);  // PLL enable
  SSD1963_WriteData(0x0001);
  delay_ms(1);
  SSD1963_WriteCommand(0x00E0);
  SSD1963_WriteData(0x0003);
  delay_ms(5);
  SSD1963_WriteCommand(0x0001);  // software reset
  delay_ms(5);
  SSD1963_WriteCommand(0x00E6);     //PLL setting for PCLK, depends on resolution
  SSD1963_WriteData(0x0003);
  SSD1963_WriteData(0x00ff);
  SSD1963_WriteData(0x00ff);

  SSD1963_WriteCommand(0x00B0);     //LCD SPECIFICATION
  SSD1963_WriteData(0x0000);
  SSD1963_WriteData(0x0000);
  SSD1963_WriteData((HDP>>8)&0X00FF);  //Set HDP
  SSD1963_WriteData(HDP&0X00FF);
  SSD1963_WriteData((VDP>>8)&0X00FF);  //Set VDP
  SSD1963_WriteData(VDP&0X00FF);
  SSD1963_WriteData(0x0000);

  SSD1963_WriteCommand(0x00B4);     //HSYNC
  SSD1963_WriteData((HT>>8)&0X00FF);  //Set HT
  SSD1963_WriteData(HT&0X00FF);
  SSD1963_WriteData((HPS>>8)&0X00FF);  //Set HPS
  SSD1963_WriteData(HPS&0X00FF);
  SSD1963_WriteData(HPW);                          //Set HPW
  SSD1963_WriteData((LPS>>8)&0X00FF);  //Set HPS
  SSD1963_WriteData(LPS&0X00FF);
  SSD1963_WriteData(0x0000);

  SSD1963_WriteCommand(0x00B6);     //VSYNC
  SSD1963_WriteData((VT>>8)&0X00FF);   //Set VT
  SSD1963_WriteData(VT&0X00FF);
  SSD1963_WriteData((VPS>>8)&0X00FF);  //Set VPS
  SSD1963_WriteData(VPS&0X00FF);
  SSD1963_WriteData(VPW);                          //Set VPW
  SSD1963_WriteData((FPS>>8)&0X00FF);  //Set FPS
  SSD1963_WriteData(FPS&0X00FF);

  SSD1963_WriteCommand(0x00BA);
  SSD1963_WriteData(0x0005);    //GPIO[3:0] out 1

  SSD1963_WriteCommand(0x00B8);
  SSD1963_WriteData(0x0007);    //GPIO3=input, GPIO[2:0]=output
  SSD1963_WriteData(0x0001);    //GPIO0 normal

  SSD1963_WriteCommand(0x0036); //rotation
  SSD1963_WriteData(0x0000);

  SSD1963_WriteCommand(0x00F0); //pixel data interface
 // #if DATAPIXELWIDTH==16
  SSD1963_WriteData(0x0003); //16 bit (565)
 // #endif
 // #if DATAPIXELWIDTH==9
 // SSD1963_WriteData(0x0006); // 9 bit
 // #endif
 // #if DATAPIXELWIDTH==8
 // SSD1963_WriteData(0x0000); // 8 bit
 // #endif

  delay_ms(5);

  SSD1963_WriteCommand(0x0029); //display on

  SSD1963_WriteCommand(0x00d0);
  SSD1963_WriteData(0x000d); */
}

void SSD1963_WriteCommand(uint16_t commandToWrite)
{
short tempS;

GPIOE->ODR &=		0x007F;
GPIOE->ODR |=		(commandToWrite&0x1FF0)<<3;
GPIOD->ODR &=		0x38fc;
tempS= ((commandToWrite&0x0003)<<14)|((commandToWrite&0x000c)>>2)|((commandToWrite&0xe000)>>5);
GPIOD->ODR |= tempS;		

SET_LCD_RD;

//GPIOG->ODR &= ~(1<<10);
RESET_LCD_RDS;
RESET_LCD_WR;
RESET_LCD_CS;
SET_LCD_CS;
SET_LCD_WR;

}

void SSD1963_WriteData(uint16_t dataToWrite)
{
GPIOE->ODR &=		0x007F;
GPIOE->ODR |=		(dataToWrite&0x1FF0)<<3;
GPIOD->ODR &=		0x38fc;
GPIOD->ODR |=		((dataToWrite&0x0003)<<14)|((dataToWrite&0x000c)>>2)|((dataToWrite&0xe000)>>5);

SET_LCD_RD;
SET_LCD_RDS;
RESET_LCD_WR;
RESET_LCD_CS;
SET_LCD_CS;
SET_LCD_WR;

} 

void delay_ms(uint16_t del)
{
uint32_t del_cnt;
del_cnt= (uint32_t)del*10000;
while(--del_cnt)
	{
	};
}

// Fills whole screen specified color
void SSD1963_SetArea(uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2)
{
  SSD1963_WriteCommand(0x002a);
  SSD1963_WriteData((x1 >> 8) & 0xff);
  SSD1963_WriteData(x1 & 0xff);
  SSD1963_WriteData((x2 >> 8) & 0xff);
  SSD1963_WriteData(x2 & 0xff);

  SSD1963_WriteCommand(0x002b);
  SSD1963_WriteData((y1 >> 8) & 0xff);
  SSD1963_WriteData(y1 & 0xff);
  SSD1963_WriteData((y2 >> 8) & 0xff);
  SSD1963_WriteData(y2 & 0xff);	 
}

#if DATAPIXELWIDTH==16
  void SSD1963_WriteDataPix(uint16_t pixdata)
  {
	GPIOE->ODR &=		0x007F;
	GPIOE->ODR |=		(pixdata&0x1FF0)<<3;
	GPIOD->ODR &=		0x38fc;
	GPIOD->ODR |=		((pixdata&0x0003)<<14)|((pixdata&0x000c)>>2)|((pixdata&0xe000)>>5);

  	//LCD_DATA_PORT->ODR  = pixdata;
  //SET_LCD_RDS;
  //RESET_LCD_WR;
  //SET_LCD_WR;
  SET_LCD_RD;
	RESET_LCD_CS;
	SET_LCD_RDS;
	RESET_LCD_WR;
	SET_LCD_WR;
	SET_LCD_CS;
   }
#endif

#if DATAPIXELWIDTH==9
void SSD1963_WriteDataPix(uint16_t pixdata)
{
  LCD_DATA_PORT->ODR  = (LCD_DATA_PORT->ODR & 0xfe00) | ((pixdata >>  8) & 0x000f) | ((pixdata >>  7) & 0x01f0);
  SET_LCD_RDS;
  RESET_LCD_WR;
  SET_LCD_WR;

  LCD_DATA_PORT->ODR  = (LCD_DATA_PORT->ODR & 0xfe00) | ((pixdata << 1) & 0x01f7) | (pixdata & 0x0001);
  RESET_LCD_WR;
  SET_LCD_WR;
}
#endif

#if DATAPIXELWIDTH==8
void SSD1963_WriteDataPix(uint16_t pixdata)
{
  LCD_DATA_PORT->ODR  = (LCD_DATA_PORT->ODR & 0xff00) | ((pixdata >>  8) & 0x00f8) | ((pixdata >>  9) & 0x0004);
  SET_LCD_RDS;
  RESET_LCD_WR;
  SET_LCD_WR;

  LCD_DATA_PORT->ODR  = (LCD_DATA_PORT->ODR & 0xff00) | ((pixdata >> 3) & 0x00fc);
  RESET_LCD_WR;
  SET_LCD_WR;

  LCD_DATA_PORT->ODR  = (LCD_DATA_PORT->ODR & 0xff00) | ((pixdata <<  3) & 0x00f8) | ((pixdata <<  2) & 0x0004);
  RESET_LCD_WR;
  SET_LCD_WR;
}
#endif

void SSD1963_ClearScreen(uint16_t color)
{
  unsigned int x,y;
  SSD1963_SetArea(0, TFT_WIDTH-1 , 0, TFT_HEIGHT-1);
  SSD1963_WriteCommand(0x002c);
  for(x=0;x<TFT_WIDTH;x++){
    for(y= 0;y<TFT_HEIGHT;y++){
      SSD1963_WriteData(color);
    }
  }
} 

//***********************************************
//***********************************************
//***********************************************
//***********************************************
//***********************************************
void SysTick_Handler (void) 	 /* SysTick Interrupt Handler (1ms)    */
{

b1000Hz=1;

sycTickCnt++;
if(sycTickCnt>=2000)
	{
	b1Hz=1;
	sycTickCnt=0;
	}
}


void main (void)
{
SysTick_Config(SystemCoreClock/1000);  /* Generate interrupt each 10 ms      */

RCC->AHB1ENR  |= ((1UL <<  2) |       /* Enable GPIOC clock                 */
                    (1UL <<  6) |       /* Enable GPIOG clock                 */
                    (1UL <<  8)  );     /* Enable GPIOI clock                 */

GPIOC->MODER  = 0x55555555;


//tick_init(); // ????????????? ?????????? ???????
lcd_port_init(); // ????????????? ?????? ?????-??????
SSD1963_Init(); // ????????????? ???????

//GPIOG->MODER=	0x00100000;
//GPIOG->OTYPER=	0x00000000; 
//GPIOG->OSPEEDR=	0xffffffff; 


while(1)
	{
	if(b1Hz)
		{
		b1Hz=0;
		GPIOC->ODR&= 0xfffe;
		GPIOC->ODR^= 0x0008;
		//lcd_port_init(); // ????????????? ?????? ?????-??????
		//SSD1963_Init(); // ????????????? ???????
		//SSD1963_WriteData(short_plazma++);
		SSD1963_ClearScreen(0x07e0);				//80ms
		  //SSD1963_WriteCommand(0x5555); //pixel data interface
 		  //SSD1963_WriteData(0xaaaa); //16 bit (565)
		GPIOC->ODR|= 0x0001;

/*		{
  		unsigned int x,y;
  		SSD1963_SetArea(50, 100 , 50, 150);
  		SSD1963_WriteCommand(0x002c);
  		for(x=0;x<6000;x++){
    //for(y= 0;y<TFT_HEIGHT;y++){
      	SSD1963_WriteData(0x001f);
    //}
  		}
} 

		{
  		unsigned int x,y;
  		SSD1963_SetArea(101, 151 , 50, 100);
  		SSD1963_WriteCommand(0x002c);
  		for(x=0;x<2600;x++){
    //for(y= 0;y<TFT_HEIGHT;y++){
      	SSD1963_WriteData(0xf800);
    //}
  		} 
} 		 */

		}
	}
};