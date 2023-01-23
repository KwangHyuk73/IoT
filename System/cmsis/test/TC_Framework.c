/*
 ******************************************************************************
 * $File    TC_Framework.c
 * $Brief   C Source file of CMSIS RTOS Framework.
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
 *      Name:         framework.c
 *      Purpose:      Test framework entry point
 *----------------------------------------------------------------------------
 *      Copyright(c) KEIL - An ARM Company
 *----------------------------------------------------------------------------*/
#include "cmsis/cmsis_rv.h"
#include "cmsis/TC_Framework.h"
// [ILG]
#include "cmsis/TC_Report.h"

// [ILG]
// Required to return a non zero value from cmsis_rv()
extern TEST_REPORT  test_report;

/*=======0=========1=========2=========3=========4=========5=========6=========7=========8=========9=========0=========1====*/
/**
\defgroup framework_funcs Framework Functions
\brief Functions in the Framework software component
\details

@{
*/

/*=======0=========1=========2=========3=========4=========5=========6=========7=========8=========9=========0=========1====*/
/**
\brief Close the debug session.
\details
Debug session dead end - debug script should close session here.
*/
void closeDebug(void) {
// [ILG]
// CMSIS defines NOP with upper case.
#if defined(__ARM_EABI__)
  __NOP();
  __NOP();
  __NOP();
#endif
//  __nop();
//  __nop();
//  __nop();
  // Test completed
}


/*=======0=========1=========2=========3=========4=========5=========6=========7=========8=========9=========0=========1====*/
/**
\brief This is the entry point of the test framework.
\details
Program flow:
  -# Hardware is first initialized if Init callback function is provided
  -# Test report statistics is initialized
  -# Test report headers are written to the standard output
  -# All defined test cases are executed:
      - Test case statistics is initialized
      - Test case report header is written to the standard output
      - Test case is executed
      - Test case results are written to the standard output
      - Test case report footer is written to the standard output
      - Test case is closed
  -# Test report footer is written to the standard output
  -# Debug session ends in dead loop
*/

// [ILG]
// The test result is an int.
int cmsis_rv(void)
{
	const char *fn;
	uint32_t tc, no;

	//kprintf("%s %08X\r\n", __FUNCTION__, ts.Init);

	/* Init test suite */
	if (ts.Init) {
		//kprintf("1");
		ts.Init();                            /* Init hardware                    */
	}
	//kprintf("2");

	ritf.Init();                           	/* Init test report                 */
	//kprintf("3");
	ritf.Open(	ts.ReportTitle,              /* Write test report title          */
				ts.Date,                     /* Write compilation date           */
				ts.Time,                     /* Write compilation time           */
				ts.FileName);                /* Write module file name           */
	//kprintf("4");

	/* Execute all test cases */
	for (tc = 0; tc < ts.NumOfTC; tc++) {
		no = ts.TCBaseNum+tc;                 /* Test case number                 */
		fn = ts.TC[tc].TFName;                /* Test function name string        */
		ritf.Open_TC (no, fn);                /* Open test case #(Base + TC)      */
		if (ts.TC[tc].en)
			ts.TC[tc].TestFunc();               /* Execute test case if enabled     */
		ritf.Close_TC ();                     /* Close test case                  */
	}
	//kprintf("5");
	ritf.Close ();                          /* Close test report                */
	//kprintf("6");

	closeDebug();                           /* Close debug session              */
	//kprintf("7");

	// [ILG]
	// Return 0 for success, 1 for failure.
	return test_report.failed ? 1 : 0;
}

/**
@}
*/
// end of group framework_funcs

