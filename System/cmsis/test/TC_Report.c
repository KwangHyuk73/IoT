/*
 ******************************************************************************
 * $File    TC_Report.c
 * $Brief   C Source file of CMSIS RTOS Test Report.
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
 *      Name:         report.c
 *      Purpose:      Report statistics and layout implementation
 *-----------------------------------------------------------------------------
 *      Copyright(c) KEIL - An ARM Company
 *----------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>

#include "cmsis/TC_Report.h"

#ifndef ITM_PRINTER_DISABLE
#define ITM_Port8(n)    (*((volatile unsigned char *)(0xE0000000+4*n)))
#define ITM_Port16(n)   (*((volatile unsigned short*)(0xE0000000+4*n)))
#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))
#define DEMCR           (*((volatile unsigned long *)(0xE000EDFC)))
#define TRCENA          0x01000000
/*-----------------------------------------------------------------------------
 *       SER_PutChar:  Write a character to the ITM Debug Port
 *----------------------------------------------------------------------------*/
int32_t SER_PutChar (int32_t ch) {
  if (DEMCR & TRCENA) {
    while (ITM_Port32(0) == 0);
    ITM_Port8(0) = ch;
  }
  return(ch);
}
/*-----------------------------------------------------------------------------
 *       SER_GetChar:  Dummy implementation
 *----------------------------------------------------------------------------*/
int32_t SER_GetChar (void) {
  return (-1);
}
#endif /* ITM_PRINTER_DISABLE */


TEST_REPORT test_report;
AS_STAT     current_assertions;   /* Current test case assertions statistics  */
uint32_t    current_result;       /* Current test case result                 */
#define TAS (&test_report.assertions)         /* Total assertions             */
#define CAS (&current_assertions)             /* Current assertions           */

// [ILG]
#if defined(TRACE)
//#define PRINT(x) trace_printf x
#define PRINT(x) kprintf(x)
#define FLUSH()
#else
#define PRINT(x) MsgPrint x
#define FLUSH()  MsgFlush()
#endif

#define TRACE_CLR_BGREEN		"\033[1;32m"    // Bold Green
#define TRACE_CLR_BYELLOW		"\033[0;33m"    // Bold Yellow
#define TRACE_CLR_BRED			"\033[1;31m"    // Bold Red
#define TRACE_CLR_BBLUE			"\033[1;34m"    // Bold Blue
#define TRACE_CLR_OFF			"\033[0m"       // Reset

static uint8_t Passed[] = TRACE_CLR_BGREEN"\tPASSED\r\n"TRACE_CLR_OFF;
static uint8_t Warning[] = TRACE_CLR_BYELLOW"\tWARNING\r\n"TRACE_CLR_OFF;
static uint8_t Failed[] = TRACE_CLR_BRED"\tFAILED\r\n"TRACE_CLR_OFF;
static uint8_t NotExe[] = TRACE_CLR_BBLUE"\tNOT EXECUTED\r\n"TRACE_CLR_OFF;


/*-----------------------------------------------------------------------------
 * Test report function prototypes
 *----------------------------------------------------------------------------*/
static BOOL     tr_Init   (void);
static BOOL     tc_Init   (void);
static uint8_t *tr_Eval   (void);
static uint8_t *tc_Eval   (void);
static BOOL     StatCount (TC_RES res);

#if !defined(TRACE)
/*-----------------------------------------------------------------------------
 * Printer function prototypes
 *----------------------------------------------------------------------------*/
static void MsgPrint (const char *msg, ...);
static void MsgFlush (void);
#endif

/*-----------------------------------------------------------------------------
 * Assert interface function prototypes
 *----------------------------------------------------------------------------*/
static BOOL As_File_Result (TC_RES res);
static BOOL As_File_Dbgi   (TC_RES res, const char *fn, uint32_t ln, char *desc);

TC_ITF tcitf = {
  As_File_Result,
  As_File_Dbgi,
};


/*-----------------------------------------------------------------------------
 * Test report interface function prototypes
 *----------------------------------------------------------------------------*/
BOOL tr_File_Init  (void);
BOOL tr_File_Open  (const char *title, const char *date, const char *time, const char *fn);
BOOL tr_File_Close (void);
BOOL tc_File_Open  (uint32_t num, const char *fn);
BOOL tc_File_Close (void);

REPORT_ITF ritf = {
  tr_File_Init,
  tr_File_Open,
  tr_File_Close,
  tc_File_Open,
  tc_File_Close
};


/*-----------------------------------------------------------------------------
 * Init test report
 *----------------------------------------------------------------------------*/
BOOL tr_File_Init (void) {
  return (tr_Init());
}


/*-----------------------------------------------------------------------------
 * Open test report
 *----------------------------------------------------------------------------*/
BOOL tr_File_Open (const char *title, const char *date, const char *time, const char *fn) {
#if (PRINT_XML_REPORT==1)
  PRINT(("<?xml version=\"1.0\"?>\n"));
  PRINT(("<?xml-stylesheet href=\"TR_Style.xsl\" type=\"text/xsl\" ?>\n"));
  PRINT(("<report>\n"));
  PRINT(("<test>\n"));
  PRINT(("<title>%s</title>\n", title));
  PRINT(("<date>%s</date>\n",   date));
  PRINT(("<time>%s</time>\n",   time));
  PRINT(("<file>%s</file>\n",   fn));
  PRINT(("<test_cases>\n"));
#else
  //PRINT(("%s   %s   %s \n\n", title, date, time));
  PRINT(("\r\n"));
#endif
  return (__TRUE);
}


/*-----------------------------------------------------------------------------
 * Open test case
 *----------------------------------------------------------------------------*/
BOOL tc_File_Open (uint32_t num, const char *fn) {
  tc_Init ();
#if (PRINT_XML_REPORT==1)
  PRINT(("<tc>\n"));
  PRINT(("<no>%d</no>\n",     num));
  PRINT(("<func>%s</func>\n", fn));
  PRINT(("<req></req>"));
  PRINT(("<meth></meth>"));
  PRINT(("<dbgi>\n"));
#else
  PRINT(("TEST %02d: %-32s ", num, fn));
#endif
  return (__TRUE);
}


/*-----------------------------------------------------------------------------
 * Close test case
 *----------------------------------------------------------------------------*/
BOOL tc_File_Close (void) {
  uint8_t *res = tc_Eval();
#if (PRINT_XML_REPORT==1)
  PRINT(("</dbgi>\n"));
  PRINT(("<res>%s</res>\n", res));
  PRINT(("</tc>\n"));
#else
  if ((res==Passed)||(res==NotExe))
    PRINT(("%s\n", res));
  else
    PRINT(("\n"));
#endif
  FLUSH();
  return (__TRUE);
}


/*-----------------------------------------------------------------------------
 * Close test report
 *----------------------------------------------------------------------------*/
BOOL tr_File_Close (void) {
#if (PRINT_XML_REPORT==1)
  PRINT(("</test_cases>\n"));
  PRINT(("<summary>\n"));
  PRINT(("<tcnt>%d</tcnt>\n", test_report.tests));
  PRINT(("<exec>%d</exec>\n", test_report.executed));
  PRINT(("<pass>%d</pass>\n", test_report.passed));
  PRINT(("<fail>%d</fail>\n", test_report.failed));
  PRINT(("<warn>%d</warn>\n", test_report.warnings));
  PRINT(("<tres>%s</tres>\n", tr_Eval()));
  PRINT(("</summary>\n"));
  PRINT(("</test>\n"));
  PRINT(("</report>\n"));
#else
  PRINT(("\n"));
  PRINT(("Test Summary: %d Tests, %d Executed, %d Passed, %d Failed, %d Warnings.\n",
         test_report.tests,
         test_report.executed,
         test_report.passed,
         test_report.failed,
         test_report.warnings));
  PRINT(("Test Result: %s\n", tr_Eval()));
#endif
  FLUSH();
  return (__TRUE);
}


/*-----------------------------------------------------------------------------
 * Assertion result counter
 *----------------------------------------------------------------------------*/
BOOL As_File_Result (TC_RES res) {
  return (StatCount (res));
}


/*-----------------------------------------------------------------------------
 * Set debug information state
 *----------------------------------------------------------------------------*/
BOOL As_File_Dbgi (TC_RES res, const char *fn, uint32_t ln, char *desc) {
  /* Write debug information block */
#if (PRINT_XML_REPORT==1)
  PRINT(("<detail>\n"));
  if (desc!=NULL) PRINT(("<desc>%s</desc>\n", desc));
  PRINT(("<module>%s</module>\n", fn));
  PRINT(("<line>%d</line>\n", ln));
  PRINT(("</detail>\n"));
#else
  // [ILG]
  PRINT(("\n"));
  PRINT(("  %s (%d)", fn, ln));
  // PRINT(("\n  %s (%d)", fn, ln));
  if (res==WARNING) PRINT((" [WARNING]"));
  if (res==FAILED) PRINT((" [FAILED]"));
  if (desc!=NULL) PRINT((" %s", desc));
#endif
  return (__TRUE);
}


/*-----------------------------------------------------------------------------
 * Init test report
 *----------------------------------------------------------------------------*/
BOOL tr_Init (void) {
  TAS->passed = 0;
  TAS->failed = 0;
  TAS->warnings = 0;
  return (__TRUE);
}


/*-----------------------------------------------------------------------------
 * Init test case
 *----------------------------------------------------------------------------*/
BOOL tc_Init (void) {
  CAS->passed = 0;
  CAS->failed = 0;
  CAS->warnings = 0;
  return (__TRUE);
}


/*-----------------------------------------------------------------------------
 * Evaluate test report results
 *----------------------------------------------------------------------------*/
uint8_t *tr_Eval (void) {
  if (test_report.failed > 0) {
    /* Test fails if any test case failed */
    return (Failed);
  }
  else if (test_report.warnings > 0) {
    /* Test warns if any test case warnings */
    return (Warning);
  }
  else if (test_report.passed > 0) {
    /* Test passes if at least one test case passed */
    return (Passed);
  }
  else {
    /* No test cases were executed */
    return (NotExe);
  }
}


/*-----------------------------------------------------------------------------
 * Evaluate test case results
 *----------------------------------------------------------------------------*/
uint8_t *tc_Eval (void) {
  test_report.tests++;
  test_report.executed++;

  if (CAS->failed > 0) {
    /* Test case fails if any failed assertion recorded */
    current_result = REP_TC_FAIL;
    test_report.failed++;
    return Failed;
  }
  else if (CAS->warnings > 0) {
    /* Test case warns if any warnings assertion recorded */
    current_result = REP_TC_WARN;
    test_report.warnings++;
    return Warning;
  }
  else if (CAS->passed > 0) {
    /* Test case passes if at least one assertion passed */
    current_result = REP_TC_PASS;
    test_report.passed++;
    return Passed;
  }
  else {
    /* Assert was not invoked - nothing to evaluate */
    current_result = REP_TC_NOEX;
    test_report.executed--;
    return NotExe;
  }
}


/*-----------------------------------------------------------------------------
 * Statistics result counter
 *----------------------------------------------------------------------------*/
BOOL StatCount (TC_RES res) {
  switch (res) {
    case PASSED:
      CAS->passed++;
      TAS->passed++;
      break;

    case WARNING:
      CAS->warnings++;
      TAS->warnings++;
      break;

    case FAILED:
      CAS->failed++;
      TAS->failed++;
      break;

    default:
      return (__FALSE);
  }
  return (__TRUE);
}


/*-----------------------------------------------------------------------------
 * Set result
 *----------------------------------------------------------------------------*/
uint32_t __set_result (const char *fn, uint32_t ln, TC_RES res, char* desc) {

  // save assertion result
  switch (res) {
    case PASSED:
      if (TAS->passed < BUFFER_ASSERTIONS) {
        test_report.assertions.info.passed[TAS->passed].module = fn;
        test_report.assertions.info.passed[TAS->passed].line = ln;
      }
      break;
    case FAILED:
      if (TAS->failed < BUFFER_ASSERTIONS) {
        test_report.assertions.info.failed[TAS->failed].module = fn;
        test_report.assertions.info.failed[TAS->failed].line = ln;
      }
      break;
    case WARNING:
      if (TAS->warnings < BUFFER_ASSERTIONS) {
        test_report.assertions.info.warnings[TAS->warnings].module = fn;
        test_report.assertions.info.warnings[TAS->warnings].line = ln;
      }
      break;
    default:
      break;
  }

  // set debug info (if the test case didn't pass)
  if (res != PASSED) tcitf.Dbgi (res, fn, ln, desc);
  // set result
  tcitf.Result (res);
  return (res);
}

/*-----------------------------------------------------------------------------
 * Assert true
 *----------------------------------------------------------------------------*/
uint32_t __assert_true (const char *fn, uint32_t ln, uint32_t cond) {
	TC_RES res = FAILED;
	if (cond!=0) res = PASSED;
	__set_result(fn, ln, res, NULL);
	return (res);
}

#if !defined(TRACE)
/*-----------------------------------------------------------------------------
 *       MsgPrint:  Print a message to the standard output
 *----------------------------------------------------------------------------*/
void MsgPrint (const char *msg, ...) {
	va_list args;
	va_start(args, msg);
	vprintf(msg, args);
	va_end(args);
}


/*-----------------------------------------------------------------------------
 *       SER_MsgFlush:  Flush the standard output
 *----------------------------------------------------------------------------*/
void MsgFlush(void) {
	fflush(stdout);
}
#endif

/*-----------------------------------------------------------------------------
 * End of file
 *----------------------------------------------------------------------------*/

