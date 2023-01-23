/*
 ******************************************************************************
 * $File    TC_GenWait.c
 * $Brief   C Source file of CMSIS RTOS GenWait Test Case.
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


/*-----------------------------------------------------------------------------
 *      Name:         RV_GenWait.c
 *      Purpose:      CMSIS RTOS validation tests implementation
 *-----------------------------------------------------------------------------
 *      Copyright(c) KEIL - An ARM Company
 *----------------------------------------------------------------------------*/
#include <string.h>
#include "cmsis/TC_Framework.h"
#include "cmsis/cmsis_rv.h"
#include "cmsis/cmsis_os.h"

/*-----------------------------------------------------------------------------
 *      Test implementation
 *----------------------------------------------------------------------------*/
osStatus Stat_Isr;
// [ILG]
// Required as return value for osWait
osEvent Event_Isr;

/*-----------------------------------------------------------------------------
 *      Default IRQ Handler
 *----------------------------------------------------------------------------*/
void GenWait_IRQHandler (void) {

	switch (ISR_ExNum) {
	case 0: Stat_Isr = osDelay (10); break;
	// [ILG]
	// `osFeatureWait` does not exist in CMSIS RTOS, so this
	// case was not tested. Enable it using the correct macro.
#if (osFeature_Wait)
	// #if (osFeatureWait)

	// [ILG]
	// The correct return value is of type osEvent.
	case 1: Event_Isr = osWait  (10); break;
	// case 1: Stat_Isr = osWait  (10); break;
#endif
	}
}

/*-----------------------------------------------------------------------------
 *      Test cases
 *----------------------------------------------------------------------------*/

/*=======0=========1=========2=========3=========4=========5=========6=========7=========8=========9=========0=========1====*/
/**
\defgroup genwait_funcs Generic Wait Functions
\brief Generic Wait Functions Test Cases
\details
The Generic Wait function group in CMSIS-RTOS provides means for a time delay and allow to wait for unspecified events. The
test cases check the functions osDelay and osWait and call the generic wait functions from an ISR.

@{
*/

/*=======0=========1=========2=========3=========4=========5=========6=========7=========8=========9=========0=========1====*/
/**
\brief Test case: TC_GenWaitBasic
\details
- Call osDelay and wait until delay is executed
- Call osWait  and wait until delay is executed
- Function calls must return osEventTimeout.
*/
void TC_GenWaitBasic (void) {
	ASSERT_TRUE (osDelay (10) == osEventTimeout);
	// [ILG]
	// `osFeatureWait` does not exist in CMSIS RTOS, so this
	// case was not tested. Enable it using the correct macro.
#if (osFeature_Wait)
	// #if (osFeatureWait)
	// [ILG]
	// The status value is returned in a structure member.
	ASSERT_TRUE (osWait  (10).status == osEventTimeout);
	// ASSERT_TRUE (osWait  (10) == osEventTimeout);
#endif
}

/*=======0=========1=========2=========3=========4=========5=========6=========7=========8=========9=========0=========1====*/
/**
\brief Test case: TC_GenWaitInterrupts
\details
- Call generic wait functions from the ISR
*/
void TC_GenWaitInterrupts (void) {

	TST_IRQHandler = GenWait_IRQHandler;

	//NVIC_EnableIRQ((IRQn_Type)0);

	ISR_ExNum = 0; /* Test: osDelay */

	// [ILG]
	// Be sure the tested value is initialised with a different value.
	Stat_Isr = osOK;

	//NVIC_SetPendingIRQ((IRQn_Type)0);
	EnterInterrupt();

	// [ILG]
	// Do not expect infinite speed
	osDelay(2);
	ExitInterrupt();

	ASSERT_TRUE (Stat_Isr == osErrorISR);

	// [ILG]
	// `osFeatureWait` does not exist in CMSIS RTOS, so this
	// case was not tested. Enable it using the correct macro.
#if (osFeature_Wait)
	// #if (osFeatureWait)

	ISR_ExNum = 1; /* Test: osWait */

	// [ILG]
	// Be sure the tested value is initialised with a different value.
	Event_Isr.status = osOK;

	// [ILG]
	// IRQ_SetPend() does not exist in CMSIS
	NVIC_SetPendingIRQ((IRQn_Type)0);
	// IRQ_SetPend (0);

	// [ILG]
	osDelay(2);

	// [ILG]
	// The status value is returned in a structure member.
	ASSERT_TRUE (Event_Isr.status == osErrorISR);
	// ASSERT_TRUE (Stat_Isr == osErrorISR);
#endif

	//NVIC_DisableIRQ((IRQn_Type)0);
}

/**
@}
*/
// end of group genwait_funcs

/*-----------------------------------------------------------------------------
 * End of file
 *----------------------------------------------------------------------------*/

