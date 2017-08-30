/**
  @page tasking TASKING Template Project for Embedded Resource Editor
 
  @verbatim
  ******************** (C) COPYRIGHT 2011 STMicroelectronics *******************
  * @file    readme.txt
  * @author  MCD Application Team
  * @version V2.0.0
  * @date    11-July-2011
  * @brief   This sub-directory contains all the user-modifiable files needed to
  *          create a new project linked with the STM32 Embedded GUI Library
  *          and working with TASKING VX-toolset for ARM Cortex-M V3.2r1.
  ******************************************************************************
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
  * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
  * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
  * CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
  * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  ******************************************************************************
  @endverbatim
 
 @par Directories contents
 
 - .cproject/.project: A pre-configured project file with the provided library
                       structure that produces an executable image with TASKING for your device.

 - TASKING                     : This folder contains
 
     - stm32*.lsl              : Linker script file for TASKING linker that contains 
                                 vector table and used to place program code (readonly) 
                                 in internal FLASH and data (readwrite, Stack and Heap)in
                                 internal SRAM. 
                                 You can customize these files to your need.

     - cstart_thumb2.asm       : This file initializes the stack pointer and copy initialized
                                 sections from ROM to RAM

                           
 @par How to use it ?

 - Open TASKING toolchain.
 - Click on File->Import, select General->'Existing Projects into Workspace' 
   and then click "Next". 
 - Browse to TASKING workspace directory and select the appropriate project
   for your target. 
 - Under Windows->Preferences->General->Workspace->Linked Resources, add 
   a variable path named "Cur_Path" which points to the folder containing
   "Libraries", "Project" and "Utilities" folders.
 - Rebuild all project files: Select the project in the "Project explorer" 
   window then click on Project->build project menu.
 - Run program: Select the project in the "Project explorer" window then click 
   Run->Debug (F11)


 @note The needed define symbols for this config are already declared in the
       preprocessor section. 

 * <h3><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h3>
 */
