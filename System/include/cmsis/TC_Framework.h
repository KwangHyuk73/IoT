/*
 ******************************************************************************
 * $File    TC_Framework.h
 * $Brief   Header file of CMSIS RTOS Test Framework.
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


#ifndef INCLUDE_TC_FRAMEWORK_H_
#define INCLUDE_TC_FRAMEWORK_H_

/*-----------------------------------------------------------------------------
 *      Name:         TC_Framework.h
 *      Purpose:      Framework header
 *----------------------------------------------------------------------------
 *      Copyright(c) KEIL - An ARM Company
 *----------------------------------------------------------------------------*/

#include "cmsis/TC_Typedefs.h"
#include "cmsis/TC_Report.h"

/*-----------------------------------------------------------------------------
 * Test framework global definitions
 *----------------------------------------------------------------------------*/

/* Test case definition macro                                                 */
#define TCD(x, y) {x, #x, y}

/* Test case description structure                                            */
typedef struct __TestCase {
	void (*TestFunc)(void);             /* Test function                      */
	const char *TFName;                 /* Test function name string          */
	BOOL en;                            /* Test function enabled              */
} TEST_CASE;

/* Test suite description structure                                           */
typedef struct __TestSuite {
	const char *FileName;               /* Test module file name                */
	const char *Date;                   /* Compilation date                     */
	const char *Time;                   /* Compilation time                     */
	const char *ReportTitle;            /* Title or name of module under test   */
	void (*Init)(void);                 /* Init function callback               */

	uint32_t TCBaseNum;                 /* Base number for test case numbering  */
	TEST_CASE *TC;                      /* Array of test cases                  */
	uint32_t NumOfTC;                   /* Number of test cases (sz of TC array)*/

} TEST_SUITE;

/* Defined in user test module                                                */
extern TEST_SUITE ts;

#endif /* INCLUDE_TC_FRAMEWORK_H_ */
