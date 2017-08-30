/**
  ******************************************************************************
  * @file    menu.c
  * @author  MCD Application Team
  * @version V2.0.0
  * @date    11-July-2011
  * @brief   This file contains all the menu functions.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "menu.h"
#include "float.h"
#include "LcdHal.h"
#include "TscHal.h"
#include "graphicObject.h"
#include "pictures.h"

/** @addtogroup Embedded_GUI_Example
  * @{
  */

/** @defgroup Menu
  * @brief Menu main functions
  * @{
  */

/** @defgroup Menu_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */


/** @defgroup Menu_Private_Defines
  * @{
  */
#if defined(USE_STM3210C_EVAL) || defined(USE_STM32100E_EVAL) || defined(USE_STM322xG_EVAL)
#define INPUT_SOURCE    (uint8_t*)"the pen or the joystick to select"
#else
#define INPUT_SOURCE    (uint8_t*)"the joystick to select"
#endif
/**
  * @}
  */

/** @defgroup Menu_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup Menu_Private_Variables
  * @{
  */

__IO uint8_t vu8_gPageCurrent = 0;
GL_Page_TypeDef *pageStart;
GL_Page_TypeDef *page0, *page1, *page2, *page3, *page4, *page5, *page6, *page7, *page8, *page9;
GL_Page_TypeDef *pageD0, *pageD1, *pageD2, *pageD3, *pageD4, *pageD5, *pageD6, *pageD7;
GL_bool stEmployee = GL_TRUE;
GL_bool drivingLicense = GL_FALSE;
GenderType user_gender = MAN;
uint8_t travellers = 0x00;
__IO uint16_t favourite_speed = 0x00;

/**
  * @}
  */

/** @defgroup Menu_Private_FunctionPrototypes
  * @{
  */

static void HomeFuncLibrary(void);
static void NextFunc(void);
static void NextFunc1(void);
static void NextFunc2(void);
static void NextFunc3(void);
static void NextFunc4(void);
static void NextFunc5(void);
static void NextFunc5(void);
static void NextFunc6(void);
static void NextFunc7(void);
static void NextFunc8(void);
static void BackFunc(void);
static void BackFunc1(void);
static void BackFunc2(void);
static void BackFunc3(void);
static void BackFunc4(void);
static void BackFunc5(void);
static void BackFunc6(void);
static void BackFunc7(void);
static void BackFunc8(void);
static void GeneralBtnFunc(void);
static void EnDisBtnFunc(void);
static void CheckboxFunc(void);
static void RButtonFunc1(void);
static void RButtonFunc2(void);
static void RButtonFunc3(void);
static void ComboboxFunc(void);
static void SlidebarFunc(void);
static void SlidebarLeftFunc(void);
static void SlidebarRightFunc(void);
static void IconFunc(void);
static void HomeFuncDemo(void);
static void StartDemoFunc(void);
static void NextFuncD1(void);
static void NextFuncD2(void);
static void NextFuncD3(void);
static void NextFuncD4(void);
static void NextFuncD5(void);
static void BackFuncD1(void);
static void BackFuncD2(void);
static void BackFuncD3(void);
static void BackFuncD4(void);
static void BackFuncD5(void);
static void SwitchDemoBtnFunc(void);
static void RButtonFuncD1(void);
static void RButtonFuncD2(void);
static void CheckboxDemoFunc(void);
static void ComboboxDemoFunc(void);
static void SlidebarDemoFunc(void);
static void FinishBtnFunc(void);
static void MENU_LibPresentation_Dummy(void);
static void MENU_Demo_Dummy(void);
static void MENU_LibPresentation_SwitchPage(GL_Page_TypeDef* PreviousPage, uint8_t PageIndex);
static void MENU_DemoPresentation_SwitchPage(GL_Page_TypeDef* PreviousPage, uint8_t PageIndex);

/**
  * @}
  */

/** @defgroup Menu_Private_Functions
  * @{
  */

/**
  * @brief  Initializes the MEnu with all the menu_variables de-initialized
  * @param  None
  * @retval None
  */
void Show_HomeScreen(void)
{
  GL_PageControls_TypeDef* pageLabel_0a;
  GL_PageControls_TypeDef* pageLabel_0b;
  GL_PageControls_TypeDef* PresentationBtn;
  GL_PageControls_TypeDef* DemoBtn;
  GL_PageControls_TypeDef* TopBannerIcon;
  /*Resetting all the menu parameters*/
  vu8_gPageCurrent = PAGE_HOME;

  /* ************** GRAPHIC LIBRARY - PAGE 0 ************** */
  pageStart = malloc(sizeof(GL_Page_TypeDef));
  Create_PageObj( pageStart );

  TopBannerIcon = NewIcon( 1, (uint8_t*) TopBanner, 320, 38, NullFunc );
  AddPageControlObj( LCD_Width - 1, 0, TopBannerIcon, pageStart );

  pageLabel_0a = NewLabel( 2, (uint8_t*)"Graphic Library", GL_HORIZONTAL, GL_FONT_BIG, GL_Blue );
  AddPageControlObj( (uint16_t)((LCD_Width / 11)*9.3), (uint16_t)((LCD_Height / 10)*1.8), pageLabel_0a, pageStart );

  pageLabel_0b = NewLabel( 3, (uint8_t*)"Presentation & Demo", GL_HORIZONTAL, GL_FONT_BIG, GL_Blue );
  AddPageControlObj( (uint16_t)((LCD_Width / 11)*10.4), (uint16_t)((LCD_Height / 10)*2.8), pageLabel_0b, pageStart );

  PresentationBtn = NewButton( 4, (uint8_t*)"LIBRARY", MENU_LibPresentation_Dummy );
  AddPageControlObj( (uint16_t)((LCD_Width / 10)*6.2), (uint16_t)((LCD_Height / 9)*3.8), PresentationBtn, pageStart );

  DemoBtn = NewButton( 5, (uint8_t*)"DEMO", MENU_Demo_Dummy );
  AddPageControlObj( (uint16_t)((LCD_Width / 10)*5.8), (uint16_t)((LCD_Height / 9)*5.3), DemoBtn, pageStart );

  GL_Clear(White);
  GL_SetTextColor(GL_Blue);
  pageStart->ShowPage( pageStart, GL_TRUE );

}


/**
  * @brief  Start the Library Presentation
  * @param  None
  * @retval None
  */
static void MENU_LibPresentation_Dummy(void)
{
  MENU_LibPresentation_SwitchPage(pageStart, PAGE_0);
}

/**
  * @brief  Build library presentation page 0
  * @param  None
  * @retval None
  */
static void LibPresentation_InitPage0(void)
{
  GL_PageControls_TypeDef* TopBannerIcon;
  GL_PageControls_TypeDef* pageLabel_0;
  GL_PageControls_TypeDef* HorizontalLabel;
  GL_PageControls_TypeDef* VerticalLabel1;
  GL_PageControls_TypeDef* VerticalLabel2;
  GL_PageControls_TypeDef* GenLabel1;
  GL_PageControls_TypeDef* GenLabel2;
  GL_PageControls_TypeDef* HomeButton0;
  GL_PageControls_TypeDef* NextButton0;

  /* ************** GRAPHIC LIBRARY PRESENTATION - PAGE 0 ************** */
  page0 = malloc(sizeof(GL_Page_TypeDef));
  Create_PageObj( page0 );
  TopBannerIcon = NewIcon( 10, (uint8_t*) TopBanner, 320, 38, NullFunc );
  AddPageControlObj( LCD_Width - 1, 0, TopBannerIcon, page0 );

  pageLabel_0 = NewLabel( 11, (uint8_t*)"Graphic Library Page0", GL_HORIZONTAL, GL_FONT_BIG, GL_Blue );
  AddPageControlObj( (uint16_t)((LCD_Width / 11)*10.9), (uint16_t)((LCD_Height / 10)*1.8), pageLabel_0, page0 );

  HorizontalLabel = NewLabel( 12, (uint8_t*)"Horizontal", GL_HORIZONTAL, GL_FONT_SMALL, GL_Black );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*5.5), (uint16_t)((LCD_Height / 9)*2.5), HorizontalLabel, page0 );

  VerticalLabel1 = NewLabel( 13, (uint8_t*)"Vertical1", GL_LEFT_VERTICAL, GL_FONT_SMALL, GL_Black );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*6.1), (uint16_t)((LCD_Height / 10)*3.2), VerticalLabel1, page0 );

  VerticalLabel2 = NewLabel( 14, (uint8_t*)"Vertical2", GL_RIGHT_VERTICAL, GL_FONT_SMALL, GL_Black );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*3),   (uint16_t)((LCD_Height / 10)*3.2), VerticalLabel2, page0 );

  GenLabel1 = NewLabel( 15, (uint8_t*)"These're 3 Labels printed", GL_HORIZONTAL, GL_FONT_SMALL, GL_Black );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*7.1), (uint16_t)((LCD_Height / 9)*6.1), GenLabel1, page0 );
  GenLabel2 = NewLabel( 16, (uint8_t*)"in various directions", GL_HORIZONTAL, GL_FONT_SMALL, GL_Black );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*7.1), (uint16_t)((LCD_Height / 9)*6.7), GenLabel2, page0 );

  HomeButton0 = NewButton( 17, (uint8_t*)"HOME", HomeFuncLibrary );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*5.18), (uint16_t)((LCD_Height / 9)*8), HomeButton0, page0 );

  NextButton0 = NewButton( 18, (uint8_t*)"NEXT>", NextFunc );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*3.3), (uint16_t)((LCD_Height / 9)*8), NextButton0, page0 );
}

/**
  * @brief  Build library presentation page 1
  * @param  None
  * @retval None
  */
static void LibPresentation_InitPage1(void)
{
  GL_PageControls_TypeDef* TopBannerIcon;
  GL_PageControls_TypeDef* pageLabel_1;
  GL_PageControls_TypeDef* GeneralBtn;
  GL_PageControls_TypeDef* GenBtnLabel1;
  GL_PageControls_TypeDef* GenBtnLabel2;
  GL_PageControls_TypeDef* GenBtnLabel3;
  GL_PageControls_TypeDef* BackButton1;
  GL_PageControls_TypeDef* HomeButton1;
  GL_PageControls_TypeDef* NextButton1;

  /* ************** GRAPHIC LIBRARY PRESENTATION - PAGE 1 ************** */
  page1 = malloc(sizeof(GL_Page_TypeDef));
  Create_PageObj( page1 );
  TopBannerIcon = NewIcon( 19, (uint8_t*) TopBanner, 320, 38, NullFunc );
  AddPageControlObj( LCD_Width - 1, 0, TopBannerIcon, page1 );

  pageLabel_1 = NewLabel( 20, (uint8_t*)"Graphic Library Page1", GL_HORIZONTAL, GL_FONT_BIG, GL_Blue );
  AddPageControlObj( (uint16_t)((LCD_Width / 11)*10.9), (uint16_t)((LCD_Height / 10)*1.8), pageLabel_1, page1 );

  GeneralBtn = NewButton( 21, (uint8_t*)"GenBtn", GeneralBtnFunc );
  AddPageControlObj( (uint16_t)((LCD_Width / 10)*6), (uint16_t)((LCD_Height / 9)*3), GeneralBtn, page1 );

  GenBtnLabel1 = NewLabel( 22, (uint8_t*)"This is a Simple Button, use", GL_HORIZONTAL, GL_FONT_SMALL, GL_Black );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*7.4), (uint16_t)((LCD_Height / 9)*5.4), GenBtnLabel1, page1 );
  GenBtnLabel2 = NewLabel( 23, INPUT_SOURCE , GL_HORIZONTAL, GL_FONT_SMALL, GL_Black );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*7.4), (uint16_t)((LCD_Height / 9)*5.9), GenBtnLabel2, page1 );
  GenBtnLabel3 = NewLabel( 24, (uint8_t*)"the button.", GL_HORIZONTAL, GL_FONT_SMALL, GL_Black );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*7.4), (uint16_t)((LCD_Height / 9)*6.5), GenBtnLabel3, page1 );

  BackButton1 = NewButton( 25, (uint8_t*)"<BACK", BackFunc );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*7.3), (uint16_t)((LCD_Height / 9)*8), BackButton1, page1 );

  HomeButton1 = NewButton( 26, (uint8_t*)"HOME", HomeFuncLibrary );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*5.18), (uint16_t)((LCD_Height / 9)*8), HomeButton1, page1 );

  NextButton1 = NewButton( 27, (uint8_t*)"NEXT>", NextFunc1 );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*3.3), (uint16_t)((LCD_Height / 9)*8), NextButton1, page1 );

}

/**
  * @brief  Build library presentation page 2
  * @param  None
  * @retval None
  */
static void LibPresentation_InitPage2(void)
{
  GL_PageControls_TypeDef* TopBannerIcon;
  GL_PageControls_TypeDef* pageLabel_2;
  GL_PageControls_TypeDef* EnDisBtn;
  GL_PageControls_TypeDef* SwitchLabel1;
  GL_PageControls_TypeDef* SwitchLabel2;
  GL_PageControls_TypeDef* SwitchLabel3;
  GL_PageControls_TypeDef* BackButton2;
  GL_PageControls_TypeDef* HomeButton2;
  GL_PageControls_TypeDef* NextButton2;
  /* ************** GRAPHIC LIBRARY PRESENTATION - PAGE 2 ************** */
  page2 = malloc(sizeof(GL_Page_TypeDef));
  Create_PageObj( page2 );

  TopBannerIcon = NewIcon( 28, (uint8_t*) TopBanner, 320, 38, NullFunc );
  AddPageControlObj( LCD_Width - 1, 0, TopBannerIcon, page2 );

  pageLabel_2 = NewLabel( 29, (uint8_t*)"Graphic Library Page2", GL_HORIZONTAL, GL_FONT_BIG, GL_Blue );
  AddPageControlObj( (uint16_t)((LCD_Width / 11)*10.9), (uint16_t)((LCD_Height / 10)*1.8), pageLabel_2, page2 );

  EnDisBtn = NewSwitch( 30, (uint8_t*)"Enable", (uint8_t*)"Disable", EnDisBtnFunc );
  AddPageControlObj( (uint16_t)((LCD_Width / 10)*6.2), (uint16_t)((LCD_Height / 9)*3), EnDisBtn, page2 );

  SwitchLabel1 = NewLabel( 31, (uint8_t*)"This is a Switch Button, use", GL_HORIZONTAL, GL_FONT_SMALL, GL_Black );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*7.4), (uint16_t)((LCD_Height / 9)*5.4), SwitchLabel1, page2 );
  SwitchLabel2 = NewLabel( 32, INPUT_SOURCE, GL_HORIZONTAL, GL_FONT_SMALL, GL_Black );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*7.4), (uint16_t)((LCD_Height / 9)*5.9), SwitchLabel2, page2 );
  SwitchLabel3 = NewLabel( 33, (uint8_t*)"the button status.", GL_HORIZONTAL, GL_FONT_SMALL, GL_Black );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*7.4), (uint16_t)((LCD_Height / 9)*6.5), SwitchLabel3, page2 );

  BackButton2 = NewButton( 34, (uint8_t*)"<BACK", BackFunc1 );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*7.3), (uint16_t)((LCD_Height / 9)*8), BackButton2, page2 );

  HomeButton2 = NewButton( 35, (uint8_t*)"HOME", HomeFuncLibrary );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*5.18), (uint16_t)((LCD_Height / 9)*8), HomeButton2, page2 );

  NextButton2 = NewButton( 36, (uint8_t*)"NEXT>", NextFunc2 );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*3.3), (uint16_t)((LCD_Height / 9)*8), NextButton2, page2 );

}

/**
  * @brief  Build library presentation page 3
  * @param  None
  * @retval None
  */
static void LibPresentation_InitPage3(void)
{
  GL_PageControls_TypeDef* TopBannerIcon;
  GL_PageControls_TypeDef* pageLabel_3;
  GL_PageControls_TypeDef* CheckBox;
  GL_PageControls_TypeDef* CheckboxLabel1;
  GL_PageControls_TypeDef* CheckboxLabel2;
  GL_PageControls_TypeDef* CheckboxLabel3;
  GL_PageControls_TypeDef* BackButton3;
  GL_PageControls_TypeDef* HomeButton3;
  GL_PageControls_TypeDef* NextButton3;

  /* ************** GRAPHIC LIBRARY PRESENTATION - PAGE 3 ************** */
  page3 = malloc(sizeof(GL_Page_TypeDef));
  Create_PageObj( page3 );

  TopBannerIcon = NewIcon( 37, (uint8_t*) TopBanner, 320, 38, NullFunc );
  AddPageControlObj( LCD_Width - 1, 0, TopBannerIcon, page3 );

  pageLabel_3 = NewLabel( 38, (uint8_t*)"Graphic Library Page3", GL_HORIZONTAL, GL_FONT_BIG, GL_Blue );
  AddPageControlObj( (uint16_t)((LCD_Width / 11)*10.9), (uint16_t)((LCD_Height / 10)*1.8), pageLabel_3, page3 );

  CheckBox = NewCheckbox( 39, (uint8_t*)"CheckBox Option", CheckboxFunc );
  AddPageControlObj( (uint16_t)((LCD_Width / 10)*7.3), (uint16_t)((LCD_Height / 9)*3), CheckBox, page3 );

  CheckboxLabel1 = NewLabel( 40, (uint8_t*)"This is a Checkbox, use", GL_HORIZONTAL, GL_FONT_SMALL, GL_Black );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*7.3), (uint16_t)((LCD_Height / 9)*5.4), CheckboxLabel1, page3 );
  CheckboxLabel2 = NewLabel( 41, INPUT_SOURCE, GL_HORIZONTAL, GL_FONT_SMALL, GL_Black );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*7.3), (uint16_t)((LCD_Height / 9)*5.9), CheckboxLabel2, page3 );
  CheckboxLabel3 = NewLabel( 42, (uint8_t*)"the Checkbox.", GL_HORIZONTAL, GL_FONT_SMALL, GL_Black );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*7.3), (uint16_t)((LCD_Height / 9)*6.5), CheckboxLabel3, page3 );

  BackButton3 = NewButton( 43, (uint8_t*)"<BACK", BackFunc2 );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*7.3), (uint16_t)((LCD_Height / 9)*8), BackButton3, page3 );

  HomeButton3 = NewButton( 44, (uint8_t*)"HOME", HomeFuncLibrary );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*5.18), (uint16_t)((LCD_Height / 9)*8), HomeButton3, page3 );

  NextButton3 = NewButton( 45, (uint8_t*)"NEXT>", NextFunc3 );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*3.3), (uint16_t)((LCD_Height / 9)*8), NextButton3, page3 );
}

/**
  * @brief  Build library presentation page 4
  * @param  None
  * @retval None
  */
static void LibPresentation_InitPage4(void)
{
  GL_PageControls_TypeDef* TopBannerIcon;
  GL_PageControls_TypeDef* pageLabel_4;
  GL_RadioButtonGrp_TypeDef* RadioButtonGrp;
  GL_PageControls_TypeDef* RadioOption_a;
  GL_PageControls_TypeDef* RadioOption_b;
  GL_PageControls_TypeDef* RadioOption_c;
  GL_PageControls_TypeDef* RadioButtonLabel1;
  GL_PageControls_TypeDef* RadioButtonLabel2;
  GL_PageControls_TypeDef* RadioButtonLabel3;
  GL_PageControls_TypeDef* BackButton4;
  GL_PageControls_TypeDef* HomeButton4;
  GL_PageControls_TypeDef* NextButton4;

  /* ************** GRAPHIC LIBRARY PRESENTATION - PAGE 4 ************** */
  page4 = malloc(sizeof(GL_Page_TypeDef));
  Create_PageObj( page4 );

  TopBannerIcon = NewIcon( 46, (uint8_t*) TopBanner, 320, 38, NullFunc );
  AddPageControlObj( LCD_Width - 1, 0, TopBannerIcon, page4 );

  pageLabel_4 = NewLabel( 47, (uint8_t*)"Graphic Library Page4", GL_HORIZONTAL, GL_FONT_BIG, GL_Blue );
  AddPageControlObj( (uint16_t)((LCD_Width / 11)*10.9), (uint16_t)((LCD_Height / 10)*1.8), pageLabel_4, page4 );

  RadioButtonGrp = NewRadioButtonGrp(48);
  RadioOption_a = RADIO_BUTTON_ADD(RadioButtonGrp, (uint8_t*)"Option1", RButtonFunc1);
  RadioOption_b = RADIO_BUTTON_ADD(RadioButtonGrp, (uint8_t*)"Option2", RButtonFunc2);
  RadioOption_c = RADIO_BUTTON_ADD(RadioButtonGrp, (uint8_t*)"Option3", RButtonFunc3);
  AddPageControlObj( (uint16_t)((LCD_Width / 10)*6.3), (uint16_t)((LCD_Height / 9)*2.5), RadioOption_a, page4 );
  AddPageControlObj( (uint16_t)((LCD_Width / 10)*6.3), (uint16_t)((LCD_Height / 9)*3.4), RadioOption_b, page4 );
  AddPageControlObj( (uint16_t)((LCD_Width / 10)*6.3), (uint16_t)((LCD_Height / 9)*4.3), RadioOption_c, page4 );

  RadioButtonLabel1 = NewLabel( 49, (uint8_t*)"This is a RadioButton, use", GL_HORIZONTAL, GL_FONT_SMALL, GL_Black );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*7.2), (uint16_t)((LCD_Height / 9)*6.1), RadioButtonLabel1, page4 );
  RadioButtonLabel2 = NewLabel( 50, INPUT_SOURCE, GL_HORIZONTAL, GL_FONT_SMALL, GL_Black );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*7.2), (uint16_t)((LCD_Height / 9)*6.6), RadioButtonLabel2, page4 );
  RadioButtonLabel3 = NewLabel( 51, (uint8_t*)"the desired option.", GL_HORIZONTAL, GL_FONT_SMALL, GL_Black );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*7.2), (uint16_t)((LCD_Height / 9)*7.2), RadioButtonLabel3, page4 );

  BackButton4 = NewButton( 52, (uint8_t*)"<BACK", BackFunc3 );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*7.3), (uint16_t)((LCD_Height / 9)*8), BackButton4, page4 );

  HomeButton4 = NewButton( 53, (uint8_t*)"HOME", HomeFuncLibrary );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*5.18), (uint16_t)((LCD_Height / 9)*8), HomeButton4, page4 );

  NextButton4 = NewButton( 54, (uint8_t*)"NEXT>", NextFunc4 );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*3.3), (uint16_t)((LCD_Height / 9)*8), NextButton4, page4 );
}

/**
  * @brief  Build library presentation page 5
  * @param  None
  * @retval None
  */
static void LibPresentation_InitPage5(void)
{
  GL_PageControls_TypeDef* TopBannerIcon;
  GL_PageControls_TypeDef* pageLabel_5;
  GL_PageControls_TypeDef* ComboBoxGrp1;
  GL_PageControls_TypeDef* ComboboxLabel1;
  GL_PageControls_TypeDef* ComboboxLabel2;
  GL_PageControls_TypeDef* ComboboxLabel3;
  GL_PageControls_TypeDef* BackButton5;
  GL_PageControls_TypeDef* HomeButton5;
  GL_PageControls_TypeDef* NextButton5;

  /* ************** GRAPHIC LIBRARY PRESENTATION - PAGE 5 ************** */
  page5 = malloc(sizeof(GL_Page_TypeDef));
  Create_PageObj( page5 );


  TopBannerIcon = NewIcon( 55, (uint8_t*) TopBanner, 320, 38, NullFunc );
  AddPageControlObj( LCD_Width - 1, 0, TopBannerIcon, page5 );

  pageLabel_5 = NewLabel( 56, (uint8_t*)"Graphic Library Page5", GL_HORIZONTAL, GL_FONT_BIG, GL_Blue );
  AddPageControlObj( (uint16_t)((LCD_Width / 11)*10.9), (uint16_t)((LCD_Height / 10)*1.8), pageLabel_5, page5 );

  ComboBoxGrp1 = NewComboBoxGrp(57);
  AddComboOption (ComboBoxGrp1->objPTR, (uint8_t*)"Option1", ComboboxFunc);
  AddComboOption (ComboBoxGrp1->objPTR, (uint8_t*)"Option2", ComboboxFunc);
  AddComboOption (ComboBoxGrp1->objPTR, (uint8_t*)"Option3", ComboboxFunc);
  AddComboOption (ComboBoxGrp1->objPTR, (uint8_t*)"Option4", ComboboxFunc);
  AddPageControlObj( (uint16_t)((LCD_Width / 10)*7.5), (uint16_t)((LCD_Height / 9)*3.1), ComboBoxGrp1, page5 );

  ComboboxLabel1 = NewLabel( 58, (uint8_t*)"This is a Combobox, use", GL_HORIZONTAL, GL_FONT_SMALL, GL_Black );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*7.2), (uint16_t)((LCD_Height / 9)*5.3), ComboboxLabel1, page5 );
  ComboboxLabel2 = NewLabel( 59, INPUT_SOURCE, GL_HORIZONTAL, GL_FONT_SMALL, GL_Black );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*7.2), (uint16_t)((LCD_Height / 9)*5.8), ComboboxLabel2, page5 );
  ComboboxLabel3 = NewLabel( 60, (uint8_t*)"the desired option.", GL_HORIZONTAL, GL_FONT_SMALL, GL_Black );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*7.2), (uint16_t)((LCD_Height / 9)*6.4), ComboboxLabel3, page5 );

  BackButton5 = NewButton( 61, (uint8_t*)"<BACK", BackFunc4 );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*7.3), (uint16_t)((LCD_Height / 9)*8), BackButton5, page5 );

  HomeButton5 = NewButton( 62, (uint8_t*)"HOME", HomeFuncLibrary );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*5.18), (uint16_t)((LCD_Height / 9)*8), HomeButton5, page5 );

  NextButton5 = NewButton( 63, (uint8_t*)"NEXT>", NextFunc5 );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*3.3), (uint16_t)((LCD_Height / 9)*8), NextButton5, page5 );
}

/**
  * @brief  Build library presentation page 6
  * @param  None
  * @retval None
  */
static void LibPresentation_InitPage6(void)
{
  GL_PageControls_TypeDef* TopBannerIcon;
  GL_PageControls_TypeDef* pageLabel_6;
  GL_PageControls_TypeDef* Slidebar;
  GL_PageControls_TypeDef* SlidebarLeftVert;
  GL_PageControls_TypeDef* SlidebarRightVert;
  GL_PageControls_TypeDef* SlideBarLabel1;
  GL_PageControls_TypeDef* SlideBarLabel2;
  GL_PageControls_TypeDef* HomeButton6;
  GL_PageControls_TypeDef* BackButton6;
  GL_PageControls_TypeDef* NextButton6;

  /* ************** GRAPHIC LIBRARY PRESENTATION - PAGE 6 ************** */
  page6 = malloc(sizeof(GL_Page_TypeDef));
  Create_PageObj( page6 );

  TopBannerIcon = NewIcon( 64, (uint8_t*) TopBanner, 320, 38, NullFunc );
  AddPageControlObj( LCD_Width - 1, 0, TopBannerIcon, page6 );

  pageLabel_6 = NewLabel( 65, (uint8_t*)"Graphic Library Page6", GL_HORIZONTAL, GL_FONT_BIG, GL_Blue );
  AddPageControlObj( (uint16_t)((LCD_Width / 11)*10.9), (uint16_t)((LCD_Height / 10)*1.8), pageLabel_6, page6 );

  Slidebar = NewSlidebar( 66, (uint8_t*)"Volume", GL_HORIZONTAL, SlidebarFunc );
  AddPageControlObj( (uint16_t)((LCD_Width / 10)*6.7), (uint16_t)((LCD_Height / 9)*2.7), Slidebar, page6 );

  SlidebarLeftVert = NewSlidebar( 67, (uint8_t*)"Left Vertical", GL_LEFT_VERTICAL, SlidebarLeftFunc );
  AddPageControlObj( (uint16_t)((LCD_Width / 10)*9.5), (uint16_t)((LCD_Height / 9)*3), SlidebarLeftVert, page6 );

  SlidebarRightVert = NewSlidebar( 68, (uint8_t*)"Right Vertical", GL_RIGHT_VERTICAL, SlidebarRightFunc );
  AddPageControlObj( (uint16_t)(LCD_Width / 10), (uint16_t)((LCD_Height / 9)*3), SlidebarRightVert, page6 );

  SlideBarLabel1 = NewLabel( 69, (uint8_t*)"This is a SlideBar, move", GL_HORIZONTAL, GL_FONT_SMALL, GL_Black );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*7.2), (uint16_t)((LCD_Height / 9)*5.4), SlideBarLabel1, page6 );
  SlideBarLabel2 = NewLabel( 70, (uint8_t*)"the cursor to left or right", GL_HORIZONTAL, GL_FONT_SMALL, GL_Black );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*7.2), (uint16_t)((LCD_Height / 9)*5.9), SlideBarLabel2, page6 );

  HomeButton6 = NewButton( 71, (uint8_t*)"HOME", HomeFuncLibrary );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*5.18), (uint16_t)((LCD_Height / 9)*8), HomeButton6, page6 );

  BackButton6 = NewButton( 72, (uint8_t*)"<BACK", BackFunc5 );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*7.3), (uint16_t)((LCD_Height / 9)*8), BackButton6, page6 );

  NextButton6 = NewButton( 73, (uint8_t*)"NEXT>", NextFunc6 );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*3.3), (uint16_t)((LCD_Height / 9)*8), NextButton6, page6 );

}

/**
  * @brief  Build library presentation page 7
  * @param  None
  * @retval None
  */
static void LibPresentation_InitPage7(void)
{

  GL_PageControls_TypeDef* Histogram;
  GL_PageControls_TypeDef* HomeButton7;
  GL_PageControls_TypeDef* BackButton7;
  GL_PageControls_TypeDef* NextButton7;
  /* ************** GRAPHIC LIBRARY PRESENTATION - PAGE 7 ************** */
  const int16_t data_points[] =
    {
      300, 355, 323, 700, 650, 458, 622, 555, 412, 538, 712, 804, 932, 614, 278, 199, 387, 399, 423, 888, 917, 601, 753, 377, 112
    };
  page7 = malloc(sizeof(GL_Page_TypeDef));
  Create_PageObj( page7 );


  Histogram = NewHistogram( 74, (uint8_t*)"Hours", (uint8_t*)"KWhs", (int16_t*)data_points, 24 );
  AddPageControlObj( (uint16_t)((LCD_Width / 10)*6.7), (uint16_t)((LCD_Height / 9)*3), Histogram, page7 );

  HomeButton7 = NewButton( 75, (uint8_t*)"HOME", HomeFuncLibrary );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*5.18), (uint16_t)((LCD_Height / 9)*8), HomeButton7, page7 );

  BackButton7 = NewButton( 76, (uint8_t*)"<BACK", BackFunc6 );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*7.3), (uint16_t)((LCD_Height / 9)*8), BackButton7, page7 );

  NextButton7 = NewButton( 77, (uint8_t*)"NEXT>", NextFunc7 );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*3.3), (uint16_t)((LCD_Height / 9)*8), NextButton7, page7 );
}

/**
  * @brief  Build library presentation page 8
  * @param  None
  * @retval None
  */
static void LibPresentation_InitPage8(void)
{
  GL_PageControls_TypeDef* GraphChart;
  GL_PageControls_TypeDef* HomeButton8;
  GL_PageControls_TypeDef* BackButton8;
  GL_PageControls_TypeDef* NextButton8;

  /* ************** GRAPHIC LIBRARY PRESENTATION - PAGE 8 ************** */
  const int16_t data_points2[] =
    {
      300, 700, 320, 650, 288, 622, 167, 512, 218, 932, 314, 804, 199, 787, 299, 923, 388, 917, 201, 753, 277, 912, 326, 799
    };
  page8 = malloc(sizeof(GL_Page_TypeDef));
  Create_PageObj( page8 );

  GraphChart = NewGraphChart( 78, (uint8_t*)"Hours", (uint8_t*)"KWhs", (int16_t*)data_points2, 24, GL_TRUE );

  AddPageControlObj( (uint16_t)((LCD_Width / 10)*6.7), (uint16_t)((LCD_Height / 9)*3), GraphChart, page8 );

  HomeButton8 = NewButton( 79, (uint8_t*)"HOME", HomeFuncLibrary );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*5.18), (uint16_t)((LCD_Height / 9)*8), HomeButton8, page8 );

  BackButton8 = NewButton( 80, (uint8_t*)"<BACK", BackFunc7 );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*7.3), (uint16_t)((LCD_Height / 9)*8), BackButton8, page8 );

  NextButton8 = NewButton( 81, (uint8_t*)"NEXT>", NextFunc8 );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*3.3), (uint16_t)((LCD_Height / 9)*8), NextButton8, page8 );
}

/**
  * @brief  Build library presentation page 9
  * @param  None
  * @retval None
  */
static void LibPresentation_InitPage9(void)
{
  GL_PageControls_TypeDef* TopBannerIcon;
  GL_PageControls_TypeDef* pageLabel_9;
  GL_PageControls_TypeDef* IconLabel1;
  GL_PageControls_TypeDef* IconLabel2;
  GL_PageControls_TypeDef* Icon;

  GL_PageControls_TypeDef* HomeButton9;
  GL_PageControls_TypeDef* BackButton9;

  /* ************** GRAPHIC LIBRARY PRESENTATION - PAGE 9 ************** */
  page9 = malloc(sizeof(GL_Page_TypeDef));
  Create_PageObj( page9 );

  TopBannerIcon = NewIcon( 82, (uint8_t*) TopBanner, 320, 38, NullFunc );
  AddPageControlObj( LCD_Width - 1, 0, TopBannerIcon, page9 );

  pageLabel_9 = NewLabel( 83, (uint8_t*)"Graphic Library Page9", GL_HORIZONTAL, GL_FONT_BIG, GL_Blue );
  AddPageControlObj( (uint16_t)((LCD_Width / 11)*10.9), (uint16_t)((LCD_Height / 10)*1.8), pageLabel_9, page9 );

#if !defined(STM32F10X_MD) && !defined(STM32F10X_MD_VL) && !defined(STM32L1XX_MD)
  Icon = NewIcon( 84, (uint8_t*) STM32Logo, 100, 100, IconFunc );
  AddPageControlObj( (uint16_t)((LCD_Width / 10)*6.6), (uint16_t)((LCD_Height / 9)*2.5), Icon, page9 );
#elif defined(STM32F10X_MD) || defined(STM32F10X_MD_VL) || defined(STM32L1XX_MD)
  Icon = NewIcon( 84, (uint8_t*) STM32Logo, 50, 50, IconFunc );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*5), (uint16_t)((LCD_Height / 9)*3.2), Icon, page9 );
#endif

  IconLabel1 = NewLabel( 85, (uint8_t*)"This is an Icon object. You can", GL_HORIZONTAL, GL_FONT_SMALL, GL_Black );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*7.5), (uint16_t)((LCD_Height / 9)*6.6), IconLabel1, page9 );
  IconLabel2 = NewLabel( 86, (uint8_t*)"associate a function to its touch", GL_HORIZONTAL, GL_FONT_SMALL, GL_Black );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*7.5), (uint16_t)((LCD_Height / 9)*7), IconLabel2, page9 );

  HomeButton9 = NewButton( 87, (uint8_t*)"HOME", HomeFuncLibrary );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*5.18), (uint16_t)((LCD_Height / 9)*8), HomeButton9, page9 );

  BackButton9 = NewButton( 88, (uint8_t*)"<BACK", BackFunc8 );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*7.3), (uint16_t)((LCD_Height / 9)*8), BackButton9, page9 );
}

/**
  * @brief  Start the Library Presentation
  * @param  None
  * @retval None
  */
void MENU_LibPresentation_Init(uint8_t InitPage)
{
  GL_SetBackColor( GL_White );
  GL_SetTextColor( GL_Blue );

  switch (InitPage)
  {
    case PAGE_0:
    {
      LibPresentation_InitPage0();
    }
    break;
    case PAGE_1:
    {
      LibPresentation_InitPage1();
    }
    break;
    case PAGE_2:
    {
      LibPresentation_InitPage2();
    }
    break;
    case PAGE_3:
    {
      LibPresentation_InitPage3();
    }
    break;
    case PAGE_4:
    {
      LibPresentation_InitPage4();
    }
    break;
    case PAGE_5:
    {
      LibPresentation_InitPage5();
    }
    break;
    case PAGE_6:
    {
      LibPresentation_InitPage6();
    }
    break;
    case PAGE_7:
    {
      LibPresentation_InitPage7();
    }
    break;
    case PAGE_8:
    {
      LibPresentation_InitPage8();
    }
    break;
    case PAGE_9:
    {
      LibPresentation_InitPage9();
    }
    break;
    default:
      break;
  }
  /* ********** End of Pages Creation for GRAPHIC LIBRARY PRESENTATION ********** */
}

/**
  * @brief  Start the Demo Application
  * @param  None
  * @retval None
  */
static void MENU_Demo_Dummy(void)
{
  MENU_DemoPresentation_SwitchPage(pageStart, PAGE_D0);
}

/**
  * @brief  Build Demo Page 0
  * @param  None
  * @retval None
  */
static GL_Page_TypeDef *  Demo_InitPage0(void)
{
  GL_PageControls_TypeDef* TopBannerIcon;
  GL_PageControls_TypeDef* pageLabel_D0;
  GL_PageControls_TypeDef* DemoStartLabel1;
  GL_PageControls_TypeDef* DemoStartLabel2;
  GL_PageControls_TypeDef* DemoStartLabel3;
  GL_PageControls_TypeDef* DemoStartLabel4;
  GL_PageControls_TypeDef* DemoStartLabel5;
  GL_PageControls_TypeDef* StartButton;
  GL_PageControls_TypeDef* HomeButtonD0;
  /* ************** DEMO APPLICATION - PAGE 0 ************** */
  pageD0 = malloc(sizeof(GL_Page_TypeDef));
  Create_PageObj( pageD0 );

  TopBannerIcon = NewIcon( 89, (uint8_t*) TopBanner, 320, 38, NullFunc );
  AddPageControlObj( LCD_Width - 1, 0, TopBannerIcon, pageD0 );

  pageLabel_D0 = NewLabel( 90, (uint8_t*)"Graphic Library Demo", GL_HORIZONTAL, GL_FONT_BIG, GL_Blue );
  AddPageControlObj( (uint16_t)((LCD_Width / 11)*10.6), (uint16_t)((LCD_Height / 10)*1.8), pageLabel_D0, pageD0 );

  DemoStartLabel1 = NewLabel( 91, (uint8_t*)"Hello! This is a little Demo Application!", GL_HORIZONTAL, GL_FONT_SMALL, GL_Black );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*8.85), (uint16_t)((LCD_Height / 9)*3), DemoStartLabel1, pageD0 );

  DemoStartLabel2 = NewLabel( 92, (uint8_t*)"You will be asked for a few question to let", GL_HORIZONTAL, GL_FONT_SMALL, GL_Black );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*8.85), (uint16_t)((LCD_Height / 9)*3.6), DemoStartLabel2, pageD0 );

  DemoStartLabel3 = NewLabel( 93, (uint8_t*)"you find out the better transportation way", GL_HORIZONTAL, GL_FONT_SMALL, GL_Black );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*8.85), (uint16_t)((LCD_Height / 9)*4.2), DemoStartLabel3, pageD0 );

  DemoStartLabel4 = NewLabel( 94, (uint8_t*)"for you. Now click on BEGIN and let's start", GL_HORIZONTAL, GL_FONT_SMALL, GL_Black );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*8.85), (uint16_t)((LCD_Height / 9)*4.8), DemoStartLabel4, pageD0 );

  DemoStartLabel5 = NewLabel( 95, (uint8_t*)"the Demo!", GL_HORIZONTAL, GL_FONT_SMALL, GL_Black );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*8.85), (uint16_t)((LCD_Height / 9)*5.5), DemoStartLabel5, pageD0 );

  StartButton = NewButton( 96, (uint8_t*)"BEGIN", StartDemoFunc );
  AddPageControlObj( (uint16_t)((LCD_Width / 10)*5.8), (uint16_t)((LCD_Height / 9)*6), StartButton, pageD0 );

  HomeButtonD0 = NewButton( 97, (uint8_t*)"HOME", HomeFuncDemo );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*5.18), (uint16_t)((LCD_Height / 9)*8), HomeButtonD0, pageD0 );
  return pageD0;
}

/**
  * @brief  Build Demo Page 1
  * @param  None
  * @retval None
  */
static GL_Page_TypeDef *  Demo_InitPage1(void)
{
  GL_PageControls_TypeDef* TopBannerIcon;
  GL_PageControls_TypeDef* HomeButtonD1;
  GL_PageControls_TypeDef* NextButtonD1;
  GL_PageControls_TypeDef* pageLabel_D1;
  GL_PageControls_TypeDef* SwitchDemoBtn;
  GL_PageControls_TypeDef* SwitchLabelD1;
  /* ************** DEMO APPLICATION - PAGE 1 ************** */
  pageD1 = malloc(sizeof(GL_Page_TypeDef));
  Create_PageObj( pageD1 );

  TopBannerIcon = NewIcon( 98, (uint8_t*) TopBanner, 320, 38, NullFunc );
  AddPageControlObj( LCD_Width - 1, 0, TopBannerIcon, pageD1 );

  pageLabel_D1 = NewLabel( 99, (uint8_t*)"Graphic Library Demo", GL_HORIZONTAL, GL_FONT_BIG, GL_Blue );
  AddPageControlObj( (uint16_t)((LCD_Width / 11)*10.6), (uint16_t)((LCD_Height / 10)*1.8), pageLabel_D1, pageD1 );

  SwitchDemoBtn = NewSwitch( 100, (uint8_t*)"YES!", (uint8_t*)"NO!", SwitchDemoBtnFunc );
  AddPageControlObj( (uint16_t)((LCD_Width / 10)*5.8), (uint16_t)((LCD_Height / 9)*3), SwitchDemoBtn, pageD1 );

  SwitchLabelD1 = NewLabel( 101, (uint8_t*)"Are you an ST Employee?", GL_HORIZONTAL, GL_FONT_SMALL, GL_Black );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*6.7), (uint16_t)((LCD_Height / 9)*4.5), SwitchLabelD1, pageD1 );

  HomeButtonD1 = NewButton( 102, (uint8_t*)"HOME", HomeFuncDemo );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*5.18), (uint16_t)((LCD_Height / 9)*8), HomeButtonD1, pageD1 );

  NextButtonD1 = NewButton( 103, (uint8_t*)"NEXT>", NextFuncD1 );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*3.3), (uint16_t)((LCD_Height / 9)*8), NextButtonD1, pageD1 );
  return pageD1;
}

/**
  * @brief  Build Demo Page 2
  * @param  None
  * @retval None
  */
static GL_Page_TypeDef *  Demo_InitPage2(void)
{
  GL_PageControls_TypeDef* TopBannerIcon;
  GL_PageControls_TypeDef* HomeButtonD2;
  GL_PageControls_TypeDef* NextButtonD2;

  GL_PageControls_TypeDef* BackButtonD2;
  GL_PageControls_TypeDef* pageLabel_D2;
  GL_RadioButtonGrp_TypeDef* RadioButtonDemoGrp;
  GL_PageControls_TypeDef* RadioOptionD_a;
  GL_PageControls_TypeDef* RadioOptionD_b;
  GL_PageControls_TypeDef* RadioButtonLabelD1;
  GL_PageControls_TypeDef* RadioButtonLabelD2;

  /* ************** DEMO APPLICATION - PAGE 2 ************** */
  pageD2 = malloc(sizeof(GL_Page_TypeDef));
  Create_PageObj( pageD2 );

  TopBannerIcon = NewIcon( 104, (uint8_t*) TopBanner, 320, 38, NullFunc );
  AddPageControlObj( LCD_Width - 1, 0, TopBannerIcon, pageD2 );

  pageLabel_D2 = NewLabel( 105, (uint8_t*)"Graphic Library Demo", GL_HORIZONTAL, GL_FONT_BIG, GL_Blue );
  AddPageControlObj( (uint16_t)((LCD_Width / 11)*10.6), (uint16_t)((LCD_Height / 10)*1.8), pageLabel_D2, pageD2 );

  RadioButtonDemoGrp = NewRadioButtonGrp(106);
  RadioOptionD_a = RADIO_BUTTON_ADD(RadioButtonDemoGrp, (uint8_t*)"Man", RButtonFuncD1);
  RadioOptionD_b = RADIO_BUTTON_ADD(RadioButtonDemoGrp, (uint8_t*)"Woman", RButtonFuncD2);
  AddPageControlObj( (uint16_t)((LCD_Width / 10)*6), (uint16_t)((LCD_Height / 9)*2.9), RadioOptionD_a, pageD2 );
  AddPageControlObj( (uint16_t)((LCD_Width / 10)*6), (uint16_t)((LCD_Height / 9)*3.9), RadioOptionD_b, pageD2 );

  RadioButtonLabelD1 = NewLabel( 107, (uint8_t*)"Select your gender:", GL_HORIZONTAL, GL_FONT_SMALL, GL_Black );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*6.3), (uint16_t)((LCD_Height / 9)*5.2), RadioButtonLabelD1, pageD2 );
  RadioButtonLabelD2 = NewLabel( 108, (uint8_t*)"Man or Woman?", GL_HORIZONTAL, GL_FONT_SMALL, GL_Black );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*6.3), (uint16_t)((LCD_Height / 9)*5.7), RadioButtonLabelD2, pageD2 );

  BackButtonD2 = NewButton( 109, (uint8_t*)"<BACK", BackFuncD1 );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*7.3), (uint16_t)((LCD_Height / 9)*8), BackButtonD2, pageD2 );

  HomeButtonD2 = NewButton( 110, (uint8_t*)"HOME", HomeFuncDemo );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*5.18), (uint16_t)((LCD_Height / 9)*8), HomeButtonD2, pageD2 );

  NextButtonD2 = NewButton( 111, (uint8_t*)"NEXT>", NextFuncD2 );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*3.3), (uint16_t)((LCD_Height / 9)*8), NextButtonD2, pageD2 );
  return pageD2;
}

/**
  * @brief  Build Demo Page 3
  * @param  None
  * @retval None
  */
static GL_Page_TypeDef *  Demo_InitPage3(void)
{
  GL_PageControls_TypeDef* TopBannerIcon;

  GL_PageControls_TypeDef* HomeButtonD3;

  GL_PageControls_TypeDef* NextButtonD3;

  GL_PageControls_TypeDef* BackButtonD3;

  GL_PageControls_TypeDef* pageLabel_D3;

  GL_PageControls_TypeDef* CheckBoxDemo;
  GL_PageControls_TypeDef* CheckboxLabelD1;
  GL_PageControls_TypeDef* CheckboxLabelD2;

  /* ************** DEMO APPLICATION - PAGE 3 ************** */
  pageD3 = malloc(sizeof(GL_Page_TypeDef));
  Create_PageObj( pageD3 );

  TopBannerIcon = NewIcon( 112, (uint8_t*) TopBanner, 320, 38, NullFunc );
  AddPageControlObj( LCD_Width - 1, 0, TopBannerIcon, pageD3 );

  pageLabel_D3 = NewLabel( 113, (uint8_t*)"Graphic Library Demo", GL_HORIZONTAL, GL_FONT_BIG, GL_Blue );
  AddPageControlObj( (uint16_t)((LCD_Width / 11)*10.6), (uint16_t)((LCD_Height / 10)*1.8), pageLabel_D3, pageD3 );

  CheckBoxDemo = NewCheckbox( 114, (uint8_t*)"I own a Driving license", CheckboxDemoFunc );
  AddPageControlObj( (uint16_t)((LCD_Width / 10)*7.9), (uint16_t)((LCD_Height / 9)*3.2), CheckBoxDemo, pageD3 );

  CheckboxLabelD1 = NewLabel( 115, (uint8_t*)"Select the Checkbox if you", GL_HORIZONTAL, GL_FONT_SMALL, GL_Black );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*7.1), (uint16_t)((LCD_Height / 9)*4.6), CheckboxLabelD1, pageD3 );
  CheckboxLabelD2 = NewLabel( 116, (uint8_t*)"have the driving license", GL_HORIZONTAL, GL_FONT_SMALL, GL_Black );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*7.1), (uint16_t)((LCD_Height / 9)*5.1), CheckboxLabelD2, pageD3 );

  BackButtonD3 = NewButton( 117, (uint8_t*)"<BACK", BackFuncD2 );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*7.3), (uint16_t)((LCD_Height / 9)*8), BackButtonD3, pageD3 );

  HomeButtonD3 = NewButton( 118, (uint8_t*)"HOME", HomeFuncDemo );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*5.18), (uint16_t)((LCD_Height / 9)*8), HomeButtonD3, pageD3 );

  NextButtonD3 = NewButton( 119, (uint8_t*)"NEXT>", NextFuncD3 );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*3.3), (uint16_t)((LCD_Height / 9)*8), NextButtonD3, pageD3 );
  return pageD3;
}

/**
  * @brief  Build Demo Page 4
  * @param  None
  * @retval None
  */
static GL_Page_TypeDef *  Demo_InitPage4(void)
{
  GL_PageControls_TypeDef* TopBannerIcon;

  GL_PageControls_TypeDef* HomeButtonD4;

  GL_PageControls_TypeDef* NextButtonD4;

  GL_PageControls_TypeDef* BackButtonD4;

  GL_PageControls_TypeDef* pageLabel_D4;
  GL_PageControls_TypeDef* ComboBoxDemoGrp1;
  GL_PageControls_TypeDef* ComboboxLabelD1;
  GL_PageControls_TypeDef* ComboboxLabelD2;

  /* ************** DEMO APPLICATION - PAGE 4 ************** */
  pageD4 = malloc(sizeof(GL_Page_TypeDef));
  Create_PageObj( pageD4 );

  TopBannerIcon = NewIcon( 120, (uint8_t*) TopBanner, 320, 38, NullFunc );
  AddPageControlObj( LCD_Width - 1, 0, TopBannerIcon, pageD4 );

  pageLabel_D4 = NewLabel( 121, (uint8_t*)"Graphic Library Demo", GL_HORIZONTAL, GL_FONT_BIG, GL_Blue );
  AddPageControlObj( (uint16_t)((LCD_Width / 11)*10.6), (uint16_t)((LCD_Height / 10)*1.8), pageLabel_D4, pageD4 );

  ComboBoxDemoGrp1 = NewComboBoxGrp(122);
  AddComboOption (ComboBoxDemoGrp1->objPTR, (uint8_t*)"Alone", ComboboxDemoFunc);
  AddComboOption (ComboBoxDemoGrp1->objPTR, (uint8_t*)"1 People", ComboboxDemoFunc);
  AddComboOption (ComboBoxDemoGrp1->objPTR, (uint8_t*)"2-4 People", ComboboxDemoFunc);
  AddComboOption (ComboBoxDemoGrp1->objPTR, (uint8_t*)"More than 4", ComboboxDemoFunc);
  AddPageControlObj( (uint16_t)((LCD_Width / 10)*7.4), (uint16_t)((LCD_Height / 9)*3.2), ComboBoxDemoGrp1, pageD4 );

  ComboboxLabelD1 = NewLabel( 123, (uint8_t*)"How many people do", GL_HORIZONTAL, GL_FONT_SMALL, GL_Black );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*6.2), (uint16_t)((LCD_Height / 9)*4.6), ComboboxLabelD1, pageD4 );
  ComboboxLabelD2 = NewLabel( 124, (uint8_t*)"You travel with?", GL_HORIZONTAL, GL_FONT_SMALL, GL_Black );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*6.2), (uint16_t)((LCD_Height / 9)*5.1), ComboboxLabelD2, pageD4 );

  BackButtonD4 = NewButton( 125, (uint8_t*)"<BACK", BackFuncD3 );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*7.3), (uint16_t)((LCD_Height / 9)*8), BackButtonD4, pageD4 );

  HomeButtonD4 = NewButton( 126, (uint8_t*)"HOME", HomeFuncDemo );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*5.18), (uint16_t)((LCD_Height / 9)*8), HomeButtonD4, pageD4 );

  NextButtonD4 = NewButton( 127, (uint8_t*)"NEXT>", NextFuncD4 );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*3.3), (uint16_t)((LCD_Height / 9)*8), NextButtonD4, pageD4 );
  return pageD4;
}

/**
  * @brief  Build Demo Page 5
  * @param  None
  * @retval None
  */
static GL_Page_TypeDef *  Demo_InitPage5(void)
{
  GL_PageControls_TypeDef* TopBannerIcon;

  GL_PageControls_TypeDef* HomeButtonD5;

  GL_PageControls_TypeDef* NextButtonD5;

  GL_PageControls_TypeDef* BackButtonD5;

  GL_PageControls_TypeDef* pageLabel_D5;

  GL_PageControls_TypeDef* SlidebarDemo;
  GL_PageControls_TypeDef* SlideBarLabelD1;
  GL_PageControls_TypeDef* SlideBarLabelD2;
  /* ************** DEMO APPLICATION - PAGE 5 ************** */
  pageD5 = malloc(sizeof(GL_Page_TypeDef));
  Create_PageObj( pageD5 );

  TopBannerIcon = NewIcon( 128, (uint8_t*) TopBanner, 320, 38, NullFunc );
  AddPageControlObj( LCD_Width - 1, 0, TopBannerIcon, pageD5 );

  pageLabel_D5 = NewLabel( 129, (uint8_t*)"Graphic Library Demo", GL_HORIZONTAL, GL_FONT_BIG, GL_Blue );
  AddPageControlObj( (uint16_t)((LCD_Width / 11)*10.6), (uint16_t)((LCD_Height / 10)*1.8), pageLabel_D5, pageD5 );

  SlidebarDemo = NewSlidebar( 130, (uint8_t*)"Speed:0-300 KMh", GL_HORIZONTAL, SlidebarDemoFunc );
  AddPageControlObj( (uint16_t)((LCD_Width / 10)*6.7), (uint16_t)((LCD_Height / 9)*2.9), SlidebarDemo, pageD5 );

  SlideBarLabelD1 = NewLabel( 131, (uint8_t*)"Move the cursor to select", GL_HORIZONTAL, GL_FONT_SMALL, GL_Black );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*7.1), (uint16_t)((LCD_Height / 9)*5.6), SlideBarLabelD1, pageD5 );
  SlideBarLabelD2 = NewLabel( 132, (uint8_t*)"your favourite cruise speed", GL_HORIZONTAL, GL_FONT_SMALL, GL_Black );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*7.1), (uint16_t)((LCD_Height / 9)*6.1), SlideBarLabelD2, pageD5 );

  BackButtonD5 = NewButton( 133, (uint8_t*)"<BACK", BackFuncD4 );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*7.3), (uint16_t)((LCD_Height / 9)*8), BackButtonD5, pageD5 );

  HomeButtonD5 = NewButton( 134, (uint8_t*)"HOME", HomeFuncDemo );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*5.18), (uint16_t)((LCD_Height / 9)*8), HomeButtonD5, pageD5 );

  NextButtonD5 = NewButton( 135, (uint8_t*)"NEXT>", NextFuncD5 );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*3.3), (uint16_t)((LCD_Height / 9)*8), NextButtonD5, pageD5 );
  return pageD5;
}

/**
  * @brief  Build Demo Page 6
  * @param  None
  * @retval None
  */
static GL_Page_TypeDef *  Demo_InitPage6(void)
{
  GL_PageControls_TypeDef* TopBannerIcon;
  GL_PageControls_TypeDef* BackButtonD6;

  GL_PageControls_TypeDef* pageLabel_D6;

  GL_PageControls_TypeDef* FinishBtn;
  GL_PageControls_TypeDef* FinishBtnLabel1;
  GL_PageControls_TypeDef* FinishBtnLabel2;

  /* ************** DEMO APPLICATION - PAGE 6 ************** */
  pageD6 = malloc(sizeof(GL_Page_TypeDef));
  Create_PageObj( pageD6 );

  TopBannerIcon = NewIcon( 136, (uint8_t*) TopBanner, 320, 38, NullFunc );
  AddPageControlObj( LCD_Width - 1, 0, TopBannerIcon, pageD6 );

  pageLabel_D6 = NewLabel( 137, (uint8_t*)"Graphic Library Demo", GL_HORIZONTAL, GL_FONT_BIG, GL_Blue );
  AddPageControlObj( (uint16_t)((LCD_Width / 11)*10.6), (uint16_t)((LCD_Height / 10)*1.8), pageLabel_D6, pageD6 );

  FinishBtn = NewButton( 138, (uint8_t*)"FINISH", FinishBtnFunc );
  AddPageControlObj( (uint16_t)((LCD_Width / 10)*6.2), (uint16_t)((LCD_Height / 9)*3.2), FinishBtn, pageD6 );

  FinishBtnLabel1 = NewLabel( 139, (uint8_t*)"Click on Finish to find out the", GL_HORIZONTAL, GL_FONT_SMALL, GL_Black );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*7.8), (uint16_t)((LCD_Height / 9)*4.6), FinishBtnLabel1, pageD6 );
  FinishBtnLabel2 = NewLabel( 140, (uint8_t*)"better transportation for you", GL_HORIZONTAL, GL_FONT_SMALL, GL_Black );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*7.8), (uint16_t)((LCD_Height / 9)*5.1), FinishBtnLabel2, pageD6 );

  BackButtonD6 = NewButton( 141, (uint8_t*)"<BACK", BackFuncD5 );
  AddPageControlObj( (uint16_t)((LCD_Width / 9)*7.2), (uint16_t)((LCD_Height / 9)*8), BackButtonD6, pageD6 );

  return pageD6;

}

/**
  * @brief  Build Demo Page 7
  * @param  None
  * @retval None
  */
static GL_Page_TypeDef *  Demo_InitPage7(void)
{
  GL_PageControls_TypeDef* TopBannerIcon;

  GL_PageControls_TypeDef* pageLabel_D7a;
  GL_PageControls_TypeDef* pageLabel_D7b;
#if defined(STM32F10X_MD) || defined(STM32F10X_MD_VL) || defined(STM32L1XX_MD)
  GL_PageControls_TypeDef* pageLabel_D7c;
#endif
#if !defined(STM32F10X_MD) && !defined(STM32F10X_MD_VL) && !defined(STM32L1XX_MD)
  GL_PageControls_TypeDef* TransportationIcon;
#endif
  GL_PageControls_TypeDef* RestartBtn;
  /* ************** DEMO APPLICATION - PAGE 7 ************** */
  pageD7 = malloc(sizeof(GL_Page_TypeDef));
  Create_PageObj( pageD7 );

  TopBannerIcon = NewIcon( 142, (uint8_t*) TopBanner, 320, 38, NullFunc );
  AddPageControlObj( LCD_Width - 1, 0, TopBannerIcon, pageD7 );

  pageLabel_D7a = NewLabel( 143, (uint8_t*)"The better transportation", GL_HORIZONTAL, GL_FONT_SMALL, GL_Black );
  AddPageControlObj( (uint16_t)((LCD_Width / 11)*8.4), (uint16_t)((LCD_Height / 10)*2.2), pageLabel_D7a, pageD7 );

  pageLabel_D7b = NewLabel( 144, (uint8_t*)"for You, is the following:", GL_HORIZONTAL, GL_FONT_SMALL, GL_Black );
  AddPageControlObj( (uint16_t)((LCD_Width / 11)*8.4), (uint16_t)((LCD_Height / 10)*2.8), pageLabel_D7b, pageD7 );

  RestartBtn = NewButton( 145, (uint8_t*)"RESTART", HomeFuncDemo );
  AddPageControlObj( (uint16_t)((LCD_Width / 10)*6.2), (uint16_t)((LCD_Height / 9)*8), RestartBtn, pageD7 );

#if defined(STM32F10X_MD) || defined(STM32F10X_MD_VL) || defined(STM32L1XX_MD)
  pageLabel_D7c = NewLabel( 146, (uint8_t*)"", GL_HORIZONTAL, GL_FONT_BIG, GL_Red );
  AddPageControlObj(  (uint16_t)((LCD_Width / 2) + ((5*GL_FONT_BIG_WIDTH) / 2)), (uint16_t)((LCD_Height / 2) - 10), pageLabel_D7c, pageD7 );
#else
  TransportationIcon = NewIcon (146, GL_NULL, 90, 90, NullFunc);
  AddPageControlObj( (uint16_t)((LCD_Width / 2) + 45), (uint16_t)((LCD_Height / 2) - 40), TransportationIcon, pageD7 );
#endif
  return pageD7;
}


/**
  * @brief  Start the Demo Application
  * @param  InitPage: page index to init
  * @retval pointer to GL_Page_TypeDef object type
  */
GL_Page_TypeDef * MENU_Demo_Init(uint8_t InitPage)
{
  GL_Page_TypeDef * tmp = NULL;
  switch (InitPage)
  {
    case PAGE_D0:
    {
      tmp = Demo_InitPage0();

    }
    break;
    case PAGE_D1:
    {
      tmp = Demo_InitPage1();
    }
    break;
    case PAGE_D2:
    {
      tmp = Demo_InitPage2();
    }
    break;
    case PAGE_D3:
    {
      tmp = Demo_InitPage3();
    }
    break;
    case PAGE_D4:
    {
      tmp = Demo_InitPage4();

    }
    break;
    case PAGE_D5:
    {
      tmp = Demo_InitPage5();

    }
    break;
    case PAGE_D6:
    {
      tmp = Demo_InitPage6();
    }
    break;
    case PAGE_D7:
    {

      tmp =  Demo_InitPage7();
    }
    break;
    default:
      break;
  }

  return tmp;
  /* ********** End of Page Creation for GRAPHIC LIBRARY DEMO TEST APPLICATION ********** */
}

/**
  * @brief  HomeFuncLibrary
  * @param  None
  * @retval None
  */
static void HomeFuncLibrary(void)
{

  /* check the page to destroy */
  switch (vu8_gPageCurrent)
  {
    case PAGE_0:
    {
      page0->ShowPage(page0, GL_FALSE);
      DestroyPage(page0);
      free(page0);
    }
    break;
    case PAGE_1:
    {
      page1->ShowPage(page1, GL_FALSE);
      DestroyPage(page1);
      free(page1);
    }
    break;
    case PAGE_2:
    {
      page2->ShowPage(page2, GL_FALSE);
      DestroyPage(page2);
      free(page2);
    }
    break;
    case PAGE_3:
    {
      page3->ShowPage(page3, GL_FALSE);
      DestroyPage(page3);
      free(page3);
    }
    break;
    case PAGE_4:
    {
      page4->ShowPage(page4, GL_FALSE);
      DestroyPage(page4);
      free(page4);
    }
    break;
    case PAGE_5:
    {
      page5->ShowPage(page5, GL_FALSE);
      DestroyPage(page5);
      free(page5);
    }
    break;
    case PAGE_6:
    {
      page6->ShowPage(page6, GL_FALSE);
      DestroyPage(page6);
      free(page6);
    }
    break;
    case PAGE_7:
    {
      page7->ShowPage(page7, GL_FALSE);
      DestroyPage(page7);
      free(page7);
    }
    break;
    case PAGE_8:
    {
      page8->ShowPage(page8, GL_FALSE);
      DestroyPage(page8);
      free(page8);
    }
    break;
    case PAGE_9:
    {
      page9->ShowPage(page9, GL_FALSE);
      DestroyPage(page9);
      free(page9);
    }
    break;
    case PAGE_HOME:
    {
      pageStart->ShowPage(pageStart, GL_FALSE);
      DestroyPage(pageStart);
      free(pageStart);
    }
    break;
    default:
      break;
  }

  Show_HomeScreen();
  vu8_gPageCurrent = PAGE_HOME;
}

/**
  * @brief  This function allows to switch from one page to an other
  * @param  PreviousPage: pointer to the current page
  * @param  PageIndex: Index of the next page
  * @retval None
  */
static void MENU_LibPresentation_SwitchPage(GL_Page_TypeDef* PreviousPage, uint8_t PageIndex)
{
  /* Switch to new page, and free previous one. */
  GL_Page_TypeDef* NextPage = NULL;

  (*PreviousPage).ShowPage(PreviousPage, GL_FALSE);
  DestroyPage(PreviousPage);
  free(PreviousPage);
  PreviousPage = NULL;
  MENU_LibPresentation_Init(PageIndex);
  vu8_gPageCurrent = PageIndex;
  switch (PageIndex)
  {
    case PAGE_0:
      NextPage = page0;
      break;
    case PAGE_1:
      NextPage = page1;
      break;
    case PAGE_2:
      NextPage = page2;
      break;
    case PAGE_3:
      NextPage = page3;
      break;
    case PAGE_4:
      NextPage = page4;
      break;
    case PAGE_5:
      NextPage = page5;
      break;
    case PAGE_6:
      NextPage = page6;
      break;
    case PAGE_7:
      NextPage = page7;
      break;
    case PAGE_8:
      NextPage = page8;
      break;
    case PAGE_9:
      NextPage = page9;
      break;
    default:
      break;
  }
  (*NextPage).ShowPage(NextPage, GL_TRUE);
}

/**
  * @brief  NextFunc
  * @param  None
  * @retval None
  */
static void NextFunc(void)
{
  MENU_LibPresentation_SwitchPage(page0, PAGE_1);
}

/**
  * @brief  NextFunc1
  * @param  None
  * @retval None
  */
static void NextFunc1(void)
{
  MENU_LibPresentation_SwitchPage(page1, PAGE_2);
}

/**
  * @brief  NextFunc2
  * @param  None
  * @retval None
  */
static void NextFunc2(void)
{
  MENU_LibPresentation_SwitchPage(page2, PAGE_3);
}

/**
  * @brief  NextFunc3
  * @param  None
  * @retval None
  */
static void NextFunc3(void)
{
  MENU_LibPresentation_SwitchPage(page3, PAGE_4);
}

/**
  * @brief  NextFunc4
  * @param  None
  * @retval None
  */
static void NextFunc4(void)
{
  MENU_LibPresentation_SwitchPage(page4, PAGE_5);
}

/**
  * @brief  NextFunc5
  * @param  None
  * @retval None
  */
static void NextFunc5(void)
{
  MENU_LibPresentation_SwitchPage(page5, PAGE_6);
}

/**
  * @brief  NextFunc6
  * @param  None
  * @retval None
  */
static void NextFunc6(void)
{
  MENU_LibPresentation_SwitchPage(page6, PAGE_7);
}

/**
  * @brief  NextFunc7
  * @param  None
  * @retval None
  */
static void NextFunc7(void)
{
  MENU_LibPresentation_SwitchPage(page7, PAGE_8);
}

/**
  * @brief  NextFunc8
  * @param  None
  * @retval None
  */
static void NextFunc8(void)
{
  MENU_LibPresentation_SwitchPage(page8, PAGE_9);
}

/**
  * @brief  BackFunc
  * @param  None
  * @retval None
  */
static void BackFunc(void)
{
  MENU_LibPresentation_SwitchPage(page1, PAGE_0);
}

/**
  * @brief  BackFunc1
  * @param  None
  * @retval None
  */
static void BackFunc1(void)
{

  MENU_LibPresentation_SwitchPage(page2, PAGE_1);
}

/**
  * @brief  BackFunc2
  * @param  None
  * @retval None
  */
static void BackFunc2(void)
{
  MENU_LibPresentation_SwitchPage(page3, PAGE_2);
}

/**
  * @brief  BackFunc3
  * @param  None
  * @retval None
  */
static void BackFunc3(void)
{

  MENU_LibPresentation_SwitchPage(page4, PAGE_3);
}

/**
  * @brief  BackFunc4
  * @param  None
  * @retval None
  */
static void BackFunc4(void)
{

  MENU_LibPresentation_SwitchPage(page5, PAGE_4);
}

/**
  * @brief  BackFunc5
  * @param  None
  * @retval None
  */
static void BackFunc5(void)
{

  MENU_LibPresentation_SwitchPage(page6, PAGE_5);
}

/**
  * @brief  BackFunc6
  * @param  None
  * @retval None
  */
static void BackFunc6(void)
{

  MENU_LibPresentation_SwitchPage(page7, PAGE_6);
}

/**
  * @brief  BackFunc7
  * @param  None
  * @retval None
  */
static void BackFunc7(void)
{

  MENU_LibPresentation_SwitchPage(page8, PAGE_7);
}

/**
  * @brief  BackFunc8
  * @param  None
  * @retval None
  */
static void BackFunc8(void)
{

  MENU_LibPresentation_SwitchPage(page9, PAGE_8);
}

/**
  * @brief  GeneralBtnFunc
  * @param  None
  * @retval None
  */
static void GeneralBtnFunc(void)
{
  uint8_t* empty_string = (uint8_t*)"                  ";
  GL_SetTextColor(GL_Red);
  GL_DisplayAdjStringLine( (uint16_t)((LCD_Height / 9)*4.2), (uint16_t)((LCD_Width / 9)*8.4), (uint8_t*)"You hit the button", GL_FALSE );
  GL_Delay(80);
  GL_DisplayAdjStringLine( (uint16_t)((LCD_Height / 9)*4.2), (uint16_t)((LCD_Width / 9)*8.4), empty_string, GL_FALSE );
  GL_SetTextColor(GL_Blue);
}

/**
  * @brief  EnDisBtnFunc
  * @param  None
  * @retval None
  */
static void EnDisBtnFunc(void)
{
  uint8_t* empty_string = (uint8_t*)"                    ";
  GL_SetTextColor(GL_Red);
  if ( GetObjStatus( page2, 30) == GL_FALSE )
  {
    GL_DisplayAdjStringLine( (uint16_t)((LCD_Height / 9)*4.2), (uint16_t)((LCD_Width / 9)*8.7), empty_string, GL_FALSE );
    GL_DisplayAdjStringLine( (uint16_t)((LCD_Height / 9)*4.2), (uint16_t)((LCD_Width / 9)*8.7), (uint8_t*)"You have disabled it", GL_FALSE );
  }
  else
  {
    GL_DisplayAdjStringLine( (uint16_t)((LCD_Height / 9)*4.2), (uint16_t)((LCD_Width / 9)*8.7), empty_string, GL_FALSE );
    GL_DisplayAdjStringLine( (uint16_t)((LCD_Height / 9)*4.2), (uint16_t)((LCD_Width / 9)*8.7), (uint8_t*)"You have enabled it", GL_FALSE );
  }
  GL_SetTextColor(GL_Blue);
}

/**
  * @brief  CheckboxFunc
  * @param  None
  * @retval None
  */
static void CheckboxFunc(void)
{
  uint8_t* empty_string = (uint8_t*)"                    ";
  GL_SetTextColor(GL_Red);
  if ( GetObjStatus( page3, 39 ) == GL_FALSE )
  {
    GL_DisplayAdjStringLine( (uint16_t)((LCD_Height / 9)*4.2), (uint16_t)((LCD_Width / 9)*8.7), empty_string, GL_FALSE );
    GL_DisplayAdjStringLine( (uint16_t)((LCD_Height / 9)*4.2), (uint16_t)((LCD_Width / 9)*8.7), (uint8_t*)"You have disabled it", GL_FALSE );
  }
  else
  {
    GL_DisplayAdjStringLine( (uint16_t)((LCD_Height / 9)*4.2), (uint16_t)((LCD_Width / 9)*8.7), empty_string, GL_FALSE );
    GL_DisplayAdjStringLine( (uint16_t)((LCD_Height / 9)*4.2), (uint16_t)((LCD_Width / 9)*8.7), (uint8_t*)"You have enabled it", GL_FALSE );
  }
  GL_SetTextColor(GL_Blue);
}

/**
  * @brief  RButtonFunc1
  * @param  None
  * @retval None
  */
static void RButtonFunc1(void)
{
  uint8_t* empty_string = (uint8_t*)"                      ";
  GL_SetTextColor(GL_Red);
  GL_DisplayAdjStringLine( (uint16_t)((LCD_Height / 10)*5.6), (uint16_t)((LCD_Width / 11)*10.9), empty_string, GL_FALSE );
  GL_DisplayAdjStringLine( (uint16_t)((LCD_Height / 10)*5.6), (uint16_t)((LCD_Width / 11)*10.9), (uint8_t*)"You've chosen option1 ", GL_TRUE );
  GL_SetTextColor(GL_Blue);
}

/**
  * @brief  RButtonFunc2
  * @param  None
  * @retval None
  */
static void RButtonFunc2(void)
{
  uint8_t* empty_string = (uint8_t*)"                      ";
  GL_SetTextColor(GL_Red);
  GL_DisplayAdjStringLine( (uint16_t)((LCD_Height / 10)*5.6), (uint16_t)((LCD_Width / 11)*10.9), empty_string, GL_FALSE );
  GL_DisplayAdjStringLine( (uint16_t)((LCD_Height / 10)*5.6), (uint16_t)((LCD_Width / 11)*10.9), (uint8_t*)"You've chosen option2", GL_TRUE );
  GL_SetTextColor(GL_Blue);
}

/**
  * @brief  RButtonFunc3
  * @param  None
  * @retval None
  */
static void RButtonFunc3(void)
{
  uint8_t* empty_string = (uint8_t*)"                      ";
  GL_SetTextColor(GL_Red);
  GL_DisplayAdjStringLine( (uint16_t)((LCD_Height / 10)*5.6), (uint16_t)((LCD_Width / 11)*10.9), empty_string, GL_FALSE );
  GL_DisplayAdjStringLine( (uint16_t)((LCD_Height / 10)*5.6), (uint16_t)((LCD_Width / 11)*10.9), (uint8_t*)"You've chosen option3", GL_TRUE );
  GL_SetTextColor(GL_Blue);
}

/**
  * @brief  ComboboxFunc
  * @param  None
  * @retval None
  */
static void ComboboxFunc(void)
{
  uint8_t* empty_string = (uint8_t*)"                      ";
  GL_SetTextColor(GL_Red);
  if ( GetComboOptionActive( page5, 57 ) == 1 )
  {
    GL_DisplayAdjStringLine( (uint16_t)((LCD_Height / 9)*4.3), (uint16_t)((LCD_Width / 11)*10.9), empty_string, GL_FALSE );
    GL_DisplayAdjStringLine( (uint16_t)((LCD_Height / 9)*4.3), (uint16_t)((LCD_Width / 11)*10.9), (uint8_t*)"You've chosen option1", GL_TRUE );
  }
  else if ( GetComboOptionActive( page5, 57 ) == 2 )
  {
    GL_DisplayAdjStringLine( (uint16_t)((LCD_Height / 9)*4.3), (uint16_t)((LCD_Width / 11)*10.9), empty_string, GL_FALSE );
    GL_DisplayAdjStringLine( (uint16_t)((LCD_Height / 9)*4.3), (uint16_t)((LCD_Width / 11)*10.9), (uint8_t*)"You've chosen option2", GL_TRUE );
  }
  else if ( GetComboOptionActive( page5, 57 ) == 3 )
  {
    GL_DisplayAdjStringLine( (uint16_t)((LCD_Height / 9)*4.3), (uint16_t)((LCD_Width / 11)*10.9), empty_string, GL_FALSE );
    GL_DisplayAdjStringLine( (uint16_t)((LCD_Height / 9)*4.3), (uint16_t)((LCD_Width / 11)*10.9), (uint8_t*)"You've chosen option3", GL_TRUE );
  }
  else if ( GetComboOptionActive( page5, 57 ) == 4 )
  {
    GL_DisplayAdjStringLine( (uint16_t)((LCD_Height / 9)*4.3), (uint16_t)((LCD_Width / 11)*10.9), empty_string, GL_FALSE );
    GL_DisplayAdjStringLine( (uint16_t)((LCD_Height / 9)*4.3), (uint16_t)((LCD_Width / 11)*10.9), (uint8_t*)"You've chosen option4", GL_TRUE );
  }

  GL_SetTextColor(GL_Blue);
}

/**
  * @brief  SlidebarFunc
  * @param  None
  * @retval None
  */
static void SlidebarFunc(void)
{
  uint8_t* empty_string = (uint8_t*)"    ";
  const uint8_t percentage_suffix[3] =
    {'%', 0
    };
  uint8_t percentage_string[4];
  uint8_t percentage_value = Get_SlidebarValue ( page6, 66 );
  sprintf((char*)percentage_string, "%d", percentage_value);
  strcat( (char*)percentage_string, (char*)percentage_suffix );
  GL_SetTextColor(GL_Red);
  GL_DisplayAdjStringLine( (uint16_t)((LCD_Height / 9)*4.2), (uint16_t)((LCD_Width / 9)*5.2), empty_string, GL_FALSE );
  GL_DisplayAdjStringLine( (uint16_t)((LCD_Height / 9)*4.2), (uint16_t)((LCD_Width / 9)*5.2), percentage_string, GL_TRUE );
  GL_SetTextColor(GL_Blue);
}

/**
* @brief  SlidebarLeftRightFunc common function
* @param  None
* @retval None
*/
static void SlidebarLeftRightFunc(uint8_t ID)
{
  const uint8_t* empty_string = (uint8_t*)"    ";
  const uint8_t percentage_suffix[3] =
    {'%', 0
    };
  uint8_t percentage_string[4];
  uint8_t percentage_value = Get_SlidebarValue ( page6, ID );

  sprintf((char*)percentage_string, "%d", percentage_value);
  strcat( (char*)percentage_string, (char*)percentage_suffix );
  GL_SetTextColor(GL_Red);
  GL_DisplayAdjStringLine( (uint16_t)((LCD_Height / 9)*7), (uint16_t)((LCD_Width / 9)*5.2), (uint8_t*)empty_string, GL_FALSE );
  GL_DisplayAdjStringLine( (uint16_t)((LCD_Height / 9)*7), (uint16_t)((LCD_Width / 9)*5.2), percentage_string, GL_TRUE );
  GL_SetTextColor(GL_Blue);
}

/**
  * @brief  SlidebarLeftFunc
  * @param  None
  * @retval None
  */
static void SlidebarLeftFunc(void)
{
  SlidebarLeftRightFunc(67);
}

/**
  * @brief  SlidebarRightFunc
  * @param  None
  * @retval None
  */
static void SlidebarRightFunc(void)
{
  SlidebarLeftRightFunc(68);
}

/**
  * @brief  IconFunc
  * @param  None
  * @retval None
  */
static void IconFunc(void)
{
  if (page9->Page_Active == GL_TRUE)
  {}
}

/**
  * @brief  HomeFuncDemo
  * @param  None
  * @retval None
  */
static void HomeFuncDemo(void)
{
  /* check the page to be cleared */
  switch (vu8_gPageCurrent)
  {
    case PAGE_D0:
    {
      pageD0->ShowPage(pageD0, GL_FALSE);
      DestroyPage(pageD0);
      free(pageD0);
    }
    break;
    case PAGE_D1:
    {
      pageD1->ShowPage(pageD1, GL_FALSE);
      DestroyPage(pageD1);
      free(pageD1);
    }
    break;
    case PAGE_D2:
    {
      pageD2->ShowPage(pageD2, GL_FALSE);
      DestroyPage(pageD2);
      free(pageD2);
    }
    break;
    case PAGE_D3:
    {
      pageD3->ShowPage(pageD3, GL_FALSE);
      DestroyPage(pageD3);
      free(pageD3);
    }
    break;
    case PAGE_D4:
    {
      pageD4->ShowPage(pageD4, GL_FALSE);
      DestroyPage(pageD4);
      free(pageD4);
    }
    break;
    case PAGE_D5:
    {
      pageD5->ShowPage(pageD5, GL_FALSE);
      DestroyPage(pageD5);
      free(pageD5);
    }
    break;
    case PAGE_D6:
    {
      pageD6->ShowPage(pageD6, GL_FALSE);
      DestroyPage(pageD6);
      free(pageD6);
    }
    break;
    case PAGE_D7:
    {
      pageD7->ShowPage(pageD7, GL_FALSE);
      DestroyPage(pageD7);
      free(pageD7);
    }
    break;
    default:
      break;
  }
  Show_HomeScreen();
  vu8_gPageCurrent = PAGE_HOME;
  //pageStart->ShowPage( pageStart, GL_TRUE );
}

/**
  * @brief  This function allows to switch from one page to an other
  * @param  PreviousPage: pointer to the current page
  * @param  PageIndex: Index of the next page
  * @retval None
  */
void MENU_DemoPresentation_SwitchPage(GL_Page_TypeDef* PreviousPage , uint8_t PageIndex)
{
  GL_Page_TypeDef* NextPage = NULL;

  (*PreviousPage).ShowPage(PreviousPage, GL_FALSE);
  DestroyPage(PreviousPage);
  free(PreviousPage);
  PreviousPage = NULL;
  vu8_gPageCurrent = PageIndex;
  NextPage = MENU_Demo_Init(PageIndex);

  NextPage->ShowPage(NextPage, GL_TRUE);
}

/**
  * @brief  StartDemoFunc
  * @param  None
  * @retval None
  */
static void StartDemoFunc(void)
{
  MENU_DemoPresentation_SwitchPage(pageD0, PAGE_D1);
}

/**
  * @brief  BackFuncD1
  * @param  None
  * @retval None
  */
static void BackFuncD1(void)
{
  MENU_DemoPresentation_SwitchPage(pageD2, PAGE_D1);
}

/**
  * @brief  BackFuncD2
  * @param  None
  * @retval None
  */
static void BackFuncD2(void)
{
  MENU_DemoPresentation_SwitchPage(pageD3, PAGE_D2);
}

/**
  * @brief  BackFuncD3
  * @param  None
  * @retval None
  */
static void BackFuncD3(void)
{
  MENU_DemoPresentation_SwitchPage(pageD4, PAGE_D3);
}

/**
  * @brief  BackFuncD4
  * @param  None
  * @retval None
  */
static void BackFuncD4(void)
{
  MENU_DemoPresentation_SwitchPage(pageD5, PAGE_D4);
}

/**
  * @brief  BackFuncD5
  * @param  None
  * @retval None
  */
static void BackFuncD5(void)
{
  MENU_DemoPresentation_SwitchPage(pageD6, PAGE_D5);
}

/**
  * @brief  NextFuncD1
  * @param  None
  * @retval None
  */
static void NextFuncD1(void)
{
  MENU_DemoPresentation_SwitchPage(pageD1, PAGE_D2);
}

/**
  * @brief  NextFuncD2
  * @param  None
  * @retval None
  */
static void NextFuncD2(void)
{
  MENU_DemoPresentation_SwitchPage(pageD2, PAGE_D3);
}

/**
  * @brief  NextFuncD3
  * @param  None
  * @retval None
  */
static void NextFuncD3(void)
{
  MENU_DemoPresentation_SwitchPage(pageD3, PAGE_D4);
}

/**
  * @brief  NextFuncD4
  * @param  None
  * @retval None
  */
static void NextFuncD4(void)
{
  MENU_DemoPresentation_SwitchPage(pageD4, PAGE_D5);
}

/**
  * @brief  NextFuncD5
  * @param  None
  * @retval None
  */
static void NextFuncD5(void)
{
  MENU_DemoPresentation_SwitchPage(pageD5, PAGE_D6);
}

/**
  * @brief  SwitchDemoBtnFunc
  * @param  None
  * @retval None
  */
static void SwitchDemoBtnFunc(void)
{
  if ( GetObjStatus( pageD1, 100 ) == GL_FALSE )
  {
    stEmployee = GL_FALSE;
  }
  else
  {
    stEmployee = GL_TRUE;
  }
}

/**
  * @brief  RButtonFuncD1
  * @param  None
  * @retval None
  */
static void RButtonFuncD1(void)
{
  user_gender = MAN;
}

/**
  * @brief  RButtonFuncD2
  * @param  None
  * @retval None
  */
static void RButtonFuncD2(void)
{
  user_gender = WOMAN;
}

/**
  * @brief  CheckboxDemoFunc
  * @param  None
  * @retval None
  */
static void CheckboxDemoFunc(void)
{
  if ( GetObjStatus( pageD3, 114 ) == GL_FALSE )
  {
    drivingLicense = GL_FALSE;
  }
  else
  {
    drivingLicense = GL_TRUE;
  }
}

/**
  * @brief  ComboboxDemoFunc
  * @param  None
  * @retval None
  */
static void ComboboxDemoFunc(void)
{
  if ( GetComboOptionActive( pageD4, 122 ) == 1 )
  {
    travellers = 1;
  }
  else if ( GetComboOptionActive( pageD4, 122 ) == 2 )
  {
    travellers = 2;
  }
  else if ( GetComboOptionActive( pageD4, 122 ) == 3 )
  {
    travellers = 5;
  }
  else if ( GetComboOptionActive( pageD4, 122 ) == 4 )
  {
    travellers = 6;
  }
}

/**
  * @brief  SlidebarDemoFunc
  * @param  None
  * @retval None
  */
static void SlidebarDemoFunc(void)
{
  uint8_t* empty_string = (uint8_t*)"         ";
  const uint8_t percentage_suffix[] =
    {'K', 'm', '/', 'h', 0
    };
  uint16_t percentage_value = 0;
  uint8_t percentage_string[4 + sizeof(percentage_suffix)];
  percentage_value = Get_SlidebarValue ( pageD5, 130 );
  percentage_value = percentage_value * 3;
  favourite_speed = percentage_value;
  sprintf((char*)percentage_string, "%d", percentage_value);
  strcat ( (char*)percentage_string, (char*)percentage_suffix );
  GL_SetTextColor(GL_Red);
  GL_DisplayAdjStringLine( (uint16_t)((LCD_Height / 9)*4.2), (uint16_t)((LCD_Width / 9)*5.9), empty_string, GL_FALSE );
  GL_DisplayAdjStringLine( (uint16_t)((LCD_Height / 9)*4.2), (uint16_t)((LCD_Width / 9)*5.9), percentage_string, GL_TRUE );
  GL_SetTextColor(GL_Blue);
}

/**
  * @brief  FinishBtnFunc
  * @param  None
  * @retval None
  */
static void FinishBtnFunc(void)
{
#if !defined(STM32F10X_MD) && !defined(STM32F10X_MD_VL) && !defined(STM32L1XX_MD)
  const uint8_t* ptrBitmap = GL_NULL;
#endif
  GL_SetBackColor( GL_White );
  GL_SetTextColor( GL_Blue );

  pageD6->ShowPage(pageD6, GL_FALSE);
  DestroyPage(pageD6);
  free(pageD6);
  MENU_Demo_Init(PAGE_D7);

#if defined(STM32F10X_MD) || defined(STM32F10X_MD_VL) || defined(STM32L1XX_MD)

  if ( favourite_speed > 150 )
  {
    Set_Label(pageD7, 146, (uint8_t*)"Airplane");
  }
  else if ( (travellers == 1 || travellers == 2) && user_gender == MAN )
  {
    Set_Label(pageD7, 146, (uint8_t*)"Motorbike") ;
  }
  else if (travellers == 5 && user_gender == MAN && favourite_speed <= 150 && stEmployee == GL_TRUE && drivingLicense == GL_TRUE)
  {
    Set_Label(pageD7, 146, (uint8_t*)"Car");
  }
  else if ( travellers == 6 || stEmployee == GL_FALSE || drivingLicense == GL_FALSE || user_gender == WOMAN)
  {
    Set_Label(pageD7, 146, (uint8_t*)"Bus") ;
  }
  else
  {
    Set_Label(pageD7, 146, (uint8_t*)"Airplane");
  }

#else /* !defined(STM32F10X_MD) && !defined(STM32F10X_MD_VL) */

  if ( favourite_speed > 150 )
  {
    ptrBitmap = Airplane;
  }
  else if ( (travellers == 1 || travellers == 2) && user_gender == MAN )
  {
    ptrBitmap = Motorbike;
  }
  else if (travellers == 5 && user_gender == MAN && (favourite_speed <= 150) && stEmployee == GL_TRUE && drivingLicense == GL_TRUE)
  {
    ptrBitmap = Car;
  }
  else if ( travellers == 6 || stEmployee == GL_FALSE || drivingLicense == GL_FALSE || user_gender == WOMAN)
  {
    ptrBitmap = Bus;
  }
  else
  {
    ptrBitmap = Airplane;
  }
  SetIconImage( pageD7, 146, ptrBitmap, 90, 90);
#endif /* defined(STM32F10X_MD) || defined(STM32F10X_MD_VL) */

  vu8_gPageCurrent = PAGE_D7;
  pageD7->ShowPage(pageD7, GL_TRUE);
}
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
