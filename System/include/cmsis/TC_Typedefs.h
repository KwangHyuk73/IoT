/*
 ******************************************************************************
 * $File    TC_Typedefs.h
 * $Brief   Header file of CMSIS RTOS Test Type definitions.
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


#ifndef INCLUDE_TC_TYPEDEFS_H_
#define INCLUDE_TC_TYPEDEFS_H_

/*-----------------------------------------------------------------------------
 *      Name:         DV_Typedefs.h
 *      Purpose:      Test framework filetypes and structures description
 *----------------------------------------------------------------------------
 *      Copyright(c) KEIL - An ARM Company
 *----------------------------------------------------------------------------*/

#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <stdio.h>

typedef unsigned int    BOOL;

#ifndef __TRUE
 #define __TRUE         1
#endif
#ifndef __FALSE
 #define __FALSE        0
#endif

#ifndef ENABLED
 #define ENABLED        1
#endif
#ifndef DISABLED
 #define DISABLED       0
#endif

#ifndef NULL
 #ifdef __cplusplus              // EC++
  #define NULL          0
 #else
  #define NULL          ((void *) 0)
 #endif
#endif

#define ARRAY_SIZE(arr) (sizeof(arr)/sizeof((arr)[0]))

//#define __FILENAME__ (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)

/* Assertions and test results */
#define SET_RESULT(res, desc) __set_result(__FILENAME__, __LINE__, res, desc);
#define ASSERT_TRUE(cond) __assert_true (__FILENAME__, __LINE__, cond);

#endif /* INCLUDE_TC_TYPEDEFS_H_ */
