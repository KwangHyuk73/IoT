/*
 ******************************************************************************
 * $File    RTE_Components.h
 * $Brief   Header file of CMSIS RTOS Run-Time-Environment Component Configuration.
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


#ifndef INCLUDE_RTE_COMPONENTS_H_
#define INCLUDE_RTE_COMPONENTS_H_

/*
 * Auto generated Run-Time-Environment Component Configuration File
 *      *** Do not modify ! ***
 *
 * Project: 'CMSIS_RV'
 * Target:  'Debug'
 */

#define RTE_CMSIS_RTOS                  /* CMSIS-RTOS */
        #define RTE_CMSIS_RTOS_FreeRTOS              /* CMSIS-RTOS FreeRTOS */
#define RTE_Compiler_IO_STDOUT          /* Compiler I/O: STDOUT */
        #define RTE_Compiler_IO_STDOUT_ITM      /* Compiler I/O: STDOUT ITM */
#define RTE_RV_GENWAIT                     /* RTOS Validation - GenWait test enabled */
#define RTE_RV_MAILQUEUE                   /* RTOS Validation - MailQueue test enabled */
#define RTE_RV_MEMORYPOOL                  /* RTOS Validation - MemoryPool test enabled */
#define RTE_RV_MSGQUEUE                    /* RTOS Validation - MsgQueue test enabled */
#define RTE_RV_MUTEX                       /* RTOS Validation - Mutex test enabled */
#define RTE_RV_SEMAPHORE                   /* RTOS Validation - Semaphore test enabled */
#define RTE_RV_SIGNAL                      /* RTOS Validation - Signal test enabled */
#define RTE_RV_THREAD                      /* RTOS Validation - Thread test enabled */
#define RTE_RV_TIMER                       /* RTOS Validation - Timer test enabled */
//#define RTE_RV_WAITFUNC                    /* RTOS Validation - WaitFunc test enabled */

#endif /* INCLUDE_RTE_COMPONENTS_H_ */
