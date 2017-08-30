/**
  @page TrueSTUDIO TrueSTUDIO Project Template for Embedded GUI Library Example
 
  @verbatim
  ******************** (C) COPYRIGHT 2011 STMicroelectronics *******************
  * @file    readme.txt
  * @author  MCD Application Team
  * @version V2.0.0
  * @date    11-July-2011
  * @brief   This sub directory contains all the user modifiable files 
  *          needed to create a new project linked with STM32 Embedded GUI
  *          Library and working with TrueSTUDIO software toolchain 
  *          (Version 2.0.1 and later)
  ******************************************************************************
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
  * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
  * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
  * CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
  * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  ******************************************************************************
  @endverbatim
 
 @par Directory contents
 
 - project .cproject/.project: A pre-configured project file with the provided 
                               library structure that produces an executable 
                               image with TrueSTUDIO.

 - stm32*.ld:    	       This file is the TrueSTUDIO linker script used to 
                               place program code (readonly) in internal FLASH 
                               and data (readwrite, Stack and Heap)in internal 
                               SRAM. 
                               You can customize this file to your need.
                           
 @par How to use it ?

 - Open the TrueSTUDIO toolchain.
 - Click on File->Switch Workspace->Other and browse to TrueSTUDIO workspace 
   directory.
 - Click on File->Import, select General->'Existing Projects into Workspace' 
   and then click "Next". 
 - Browse to the TrueSTUDIO workspace directory and select the appropriate project 
   to your device.
 - Under Windows->Preferences->General->Workspace->Linked Resources, add 
   a variable path named "CurPath" which points to the folder containing
   "Libraries", "Project" and "Utilities" folders.
 - Rebuild all project files: Select the project in the "Project explorer" 
   window then click on Project->build project menu.
 - Run program: Select the project in the "Project explorer" window then click 
   Run->Debug (F11)

 * <h3><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h3>
 */
