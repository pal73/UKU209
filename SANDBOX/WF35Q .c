#include "C8051F340.h"
#include <INTRINS.H>  


//-----------------------------------------------------------------
// Bit Definitions
#define        Data_BUS         P1		  // D15 ~ D8
sbit           IC_A0            =P2^6;    // Data/command select L:Command,H:Data
sbit           IC_WR            =P0^6; 	  // L: Write, H: Read
sbit           IC_RD            =P0^7;	  // Data Enable H-->L
sbit           IC_CS            =P2^4;    // L: Chip select
sbit           IC_RST           =P2^1;    // L: RESET
sbit		   IC_DIR			=P2^7;
sbit		   IC_HOLD			=P0^3;
sbit		   MD				=P2^2;
sbit		   CS2				=P2^3;
sbit		   FS				=P2^0;
/// SW //////////////////////////
sbit 			S_S             =P0^1;	//SW1
sbit 			STP	        	=P0^0;	//SW2
sbit 			SW3	        	=P0^2;	//SW3
sbit 			SW4	        	=P0^4;	//SW4
sbit 			SW5	        	=P0^5;	//SW5
//------------------------------------------------------------------------------

unsigned int GATE;
unsigned int SOURCE;
unsigned long color_1,color_2,color_3,color_4;





//------------------------------------------------------------------------------
void Write_Command(unsigned char command)
{
	IC_RD = 1;
	IC_A0 = 0;
	IC_WR = 0;
	IC_CS = 0;
	FS = 0;
	Data_BUS = command;
	IC_CS = 1;
	FS = 1;
	IC_WR = 1;
}

//------------------------------------------------------------------------------
void Write_Data(unsigned char data1)
{
	IC_RD = 1;
	IC_A0 = 1;
	IC_WR = 0;
	IC_CS = 0;
	FS = 0;
	Data_BUS = data1;
	IC_CS = 1;
	FS = 1;
	IC_WR = 1;
}

//======================================================
void Write_Data_16BIT(unsigned long data1)
{
	IC_RD = 1;
	IC_A0 = 1;
	IC_WR = 0;
	IC_CS = 0;

	IC_HOLD	=1;
	Data_BUS = ((data1)>>8);
	IC_HOLD	=0;	
	Data_BUS = data1;
	
	IC_CS = 1;
	IC_WR = 1;
}
//------------------------------------------------------------------------------
void Command_Write(unsigned char command,unsigned char data1)
{
	Write_Command(command);
	Write_Data(data1);
}
//------------------------------------------------------------------------------
void SendData(unsigned long color)
{
	if(S_S == 1)
	{
		//8bit
		Write_Data((color)>>16);  // color is red
		Write_Data((color)>>8);  	// color is green
		Write_Data(color);	  		// color is blue
	}
	else
	{
		//16bit
		unsigned long color_1,color_2,color_3,color_4;
		color_1 = ((color)>>3) & 0x00001f;	// color is red
		color_2 = ((color)>>5) & 0x0007e0;	// color is green
		color_3 = ((color)>>8) & 0x00f800;	// color is blue
		color_4 = (color_1 ^ color_2 ^ color_3); 
		Write_Data_16BIT(color_4);
	}
	
}

/*************************************************/
//Set SSD1963 Ram address
/*************************************************/
void SSD1963_RAM_Address()
{
	Write_Command(0x2a);	//SET column address
	Write_Data(0x00);	//SET start column address=0
	Write_Data(0x00);
	Write_Data((SOURCE-1)>>8);	//SET end column address
	Write_Data(SOURCE-1);

	Write_Command(0x2b);	//SET page address
	Write_Data(0x00);	//SET start page address=0
	Write_Data(0x00);
	Write_Data((GATE-1)>>8);	//SET end page address
	Write_Data(GATE-1);

    Write_Command(0x2c);

}

/*******************************************/
//Test for all white or black or red or blue or green
/*******************************************/
void SSD1963_Show_RGB(unsigned char dat1,unsigned char dat2,unsigned char dat3)
{
	 unsigned int i,j;

	SSD1963_RAM_Address();
	//WindowSet(0x0000,0x013f,0x0000,0x00ef);

	for(i=0;i<GATE;i++)
	{
		for(j=0;j<SOURCE;j++)
		{
			if(S_S == 1)
			{
				//8bit
				Write_Data(dat1);
				Write_Data(dat2);
				Write_Data(dat3);
			}
			else
			{
				//16bit
				color_1 = dat1;	// color is blue
				color_1 = color_1<<16;
				color_2 = dat2;	// color is green
				color_2 = color_2<<8;
				color_3 = dat3;	// color is red		
				color_4 = (color_1 ^ color_2 ^ color_3); 									
				SendData(color_4);
			}
		}

	}

}
/////////////////////////////////////////////
void delay(unsigned long delay_value)
{
	while(delay_value!=0)
	delay_value--;
}
/////////////////////////////////////////////
/////////////////////////////////////////////
void SW2_SC(void)
{
		while(1)
		{
			if(STP ==0)
			break;
		}
}
//----------------------------------------------
void SW3_SC(void)
{
		while(1)
		{
			if(SW3 ==0)
			break;
		}
}
//----------------------------------------------
void SW4_SC(void)
{
		while(1)
		{
			if(SW4 ==0)
			break;
		}
}
//----------------------------------------------
void SW5_SC(void)
{
		while(1)
		{
			if(SW5 ==0)
			break;
		}
}
//////////////////////////////////////////////////////////////////
//------------------------------------------------------------------------------
//  WF35Q initial
//------------------------------------------------------------------------------
void WF35_Initial_SSD1963(void)
{
	GATE = 240;
    SOURCE = 320;

	IC_RST = 0;
	_nop_();
	_nop_();
	_nop_();
	IC_RST = 1;
	_nop_();
	_nop_();
	_nop_();
	Write_Command(0x01);     //Software Reset
	_nop_();
	_nop_();
	_nop_();
	_nop_();
	_nop_();
	_nop_();
	Command_Write(0xe0,0x01);    //START PLL
	_nop_();
	_nop_();
	_nop_();
	Command_Write(0xe0,0x03);    //LOCK PLL
	_nop_();
	_nop_();
	_nop_(); 

	Write_Command(0xb0);  //SET LCD MODE  SET TFT 18Bits MODE
	Write_Data(0x20);   //SET TFT MODE & hsync+Vsync+DEN MODE
	Write_Data(0x80);   //SET TFT MODE & hsync+Vsync+DEN MODE
	Write_Data(0x01);   //SET horizontal size=320-1 HightByte
	Write_Data(0x3f);      //SET horizontal size=320-1 LowByte
	Write_Data(0x00);   //SET vertical size=240-1 HightByte
	Write_Data(0xef);   //SET vertical size=240-1 LowByte
	Write_Data(0x00);   //SET even/odd line RGB seq.=RGB
 
 	if(S_S == 1)
	{
		//8bit(666)
		Command_Write(0xf0,0x00); 	//SET pixel data I/F format=8bit
	}
	else
	{
		//16bit(565)
		Command_Write(0xf0,0x03);	//SET pixel data I/F format=16bit(565 format)
	}
	
	Command_Write(0x36,0x08);   // SET read from frame buffer to the display is RGB  
 
	Write_Command(0xe2);    
	Write_Data(0x1d);		
	Write_Data(0x02);			
	Write_Data(0x54);		

	Write_Command(0xe6);     //SET PCLK freq=4.94MHz  ; pixel clock frequency
	Write_Data(0x01);		//00
	Write_Data(0xdd);		//ce	
	Write_Data(0xde);		//94
 
	Write_Command(0xb4);		//SET HSYNC 
	Write_Data(0x01);			
	Write_Data(0x98);			//SET HT = 408(10)=0198(16)
	Write_Data(0x00);			
	Write_Data(0x44);			//SET HBP = 68(10)=44(16)
	Write_Data(0x14);			//SET HPW = 20(10)=14(16)
	Write_Data(0x00);			//SET LPS = 0
	Write_Data(0x00);
	Write_Data(0x00);			
	
	Write_Command(0xb6); 		//SET VSYNC
	Write_Data(0x01);			
	Write_Data(0x06);			//SET HT = 262(10)=408(!6)
	Write_Data(0x00);			
	Write_Data(0x12);			//SET VBP = 18(10)=18(16)
	Write_Data(0x04);			//SET VPW = 4(10)=4(16)PS = 0Vsync pulse 8 = 7 + 1
	Write_Data(0x00);			//SET FPS = 0
	Write_Data(0x00);
 
	Write_Command(0x2a);  //SET column address
	Write_Data(0x00);   //SET start column address=0
	Write_Data(0x00);
	Write_Data(0x01);   //SET end column address=320
	Write_Data(0x3f);
 
	Write_Command(0x2b);  //SET page address
	Write_Data(0x00);   //SET start page address=0
	Write_Data(0x00);
	Write_Data(0x00);   //SET end page address=240
	Write_Data(0xef);
 
	Write_Command(0xb8);   //SET GPIO
	Write_Data(0x0f);      //SET I/O
	Write_Data(0x01);
	Write_Command(0xba);   //SET GPIO
    Write_Data(0x01);      //SET I/O

 
	Write_Command(0x29);  //SET display on
	Write_Command(0x2c);
 
}


//////////////////////////////////////////////////////////////////
void main(void)
{	

	//////////////////////////////////////////
	IC_RST = 0;
	delay(255);//47us
	delay(255);//110us
	delay(255);//120us~160us
	IC_RST = 1;
	_nop_();
	_nop_();
	_nop_(); 
	//////////////////////////////////////////

	if(SW4 == 0) //SW4 for WF57F Demo
	{
		WF35_Initial_SSD1963();
		_nop_(); _nop_(); _nop_(); _nop_(); _nop_();
		_nop_(); _nop_(); _nop_(); _nop_(); _nop_();
		_nop_(); _nop_(); _nop_(); _nop_(); _nop_();
        
        SSD1963_Show_RGB(0xFF, 0xFF, 0xFF);
	}
}

