/*
 ******************************************************************************
 * $File    TC_Config.h
 * $Brief   Header file of CMSIS RTOS Test Configuration.
 * $Created on: Jan 18, 2023
 ******************************************************************************
 *
 * Copyright (c) 2023 Digital Museum. All rights reserved.
 *
 * This software component is licensed by Digital Museum under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 *	https://www.youtube.com/@digitalmuseum6400
 *	https://github.com/KwangHyuk73
 *	e-mail:ponytail2k@gmail.com
 ******************************************************************************
 */


#ifndef INCLUDE_TC_CONFIG_H_
#define INCLUDE_TC_CONFIG_H_

/*-----------------------------------------------------------------------------
 *      Name:         TC_Config.h
 *      Purpose:      TC Config header
 *----------------------------------------------------------------------------
 *      Copyright(c) KEIL - An ARM Company
 *----------------------------------------------------------------------------*/

#if defined(__ARM_EABI__)
#include "cmsis/cmsis_device.h"
#else

typedef int IRQn_Type;
void
NVIC_EnableIRQ(IRQn_Type);
void
NVIC_DisableIRQ(IRQn_Type);
void
NVIC_SetPendingIRQ(IRQn_Type);

#endif

//#include <cmsis-plus/diag/trace.h>

// [ILG]
// #define PARAM_CHECKING (1)

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------

// <h> Common Test Settings
// <o> Print Output Format <0=> Plain Text <1=> XML
// <i> Set the test results output format to plain text or XML
#ifndef PRINT_XML_REPORT
#define PRINT_XML_REPORT            0
#endif
// <o> Buffer size for assertions results
// <i> Set the buffer size for assertions results buffer
#define BUFFER_ASSERTIONS           128
// </h>

#if defined(__ARM_EABI__)
#define HW_PRESENT  (1)
#endif

extern void EnterInterrupt(void);
extern void ExitInterrupt(void);

#endif /* INCLUDE_TC_CONFIG_H_ */
