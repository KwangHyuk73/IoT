/*
 ******************************************************************************
 * $File    TC_Mutex.c
 * $Brief   C Source file of CMSIS RTOS Mutex Test Case.
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
 *      Name:         TC_Mutex3.c
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
typedef struct {
	uint32_t Ex_Num;                      /* Exception number                   */
	uint32_t Ex_Res;                      /* Exception result                   */
} ISR_RES;

ISR_RES Isr;

osMutexDef (MutexIsr);
osMutexId ISR_MutexId;
osStatus  ISR_OsStat;

/* Definitions of shared variables */
uint8_t     G_ExecArr[3];               /* Global execution array             */
osMutexId   G_MutexId;                  /* Global mutex id                    */
osThreadId  G_Mutex_ThreadId;           /* Global thread id                   */


/* Definitions for TC_MutexBasic */
osMutexDef (MutexBas);

/* Definitions for TC_MutexTimeout */
static void  Th_MutexLock (void const *arg);
osThreadDef (Th_MutexLock, Th_MutexLock, configMINIMAL_STACK_SIZE, osPriorityAboveNormal);

osMutexDef (MutexTout);

/* Definitions for TC_MutexNestedAcquire */
static void  Th_MutexWait (void const *arg);
osThreadDef (Th_MutexWait, Th_MutexWait, configMINIMAL_STACK_SIZE, osPriorityAboveNormal);

static void RecursiveMutexAcquire (uint32_t depth, uint32_t ctrl);
osMutexDef (Mutex_Nest);


/* Definitions for TC_MutexPriorityInversion */
void Th_LowPrioJob    (void const *arg);
void Th_MediumPrioJob (void const *arg);
void Th_HighPrioJob   (void const *arg);

osThreadDef (Th_LowPrioJob,    Th_LowPrioJob,    configMINIMAL_STACK_SIZE, osPriorityBelowNormal);
osThreadDef (Th_MediumPrioJob, Th_MediumPrioJob, configMINIMAL_STACK_SIZE, osPriorityNormal);
osThreadDef (Th_HighPrioJob,   Th_HighPrioJob,   configMINIMAL_STACK_SIZE, osPriorityAboveNormal);

osMutexDef (Mutex_PrioInv);

/* Definitions for TC_MutexOwnership */
osMutexDef (Mutex_Ownership);

void Th_MutexAcqLow  (void const *arg);
void Th_MutexRelHigh (void const *arg);
osThreadDef (Th_MutexAcqLow,  Th_MutexAcqLow,  configMINIMAL_STACK_SIZE, osPriorityLow);
osThreadDef (Th_MutexRelHigh, Th_MutexRelHigh, configMINIMAL_STACK_SIZE, osPriorityHigh);



/*-----------------------------------------------------------------------------
 *      Default IRQ Handler
 *----------------------------------------------------------------------------*/
void Mutex_IRQHandler (void) {
	uint32_t res = 0;

	switch (Isr.Ex_Num) {
	case 0: ISR_MutexId = osMutexCreate  (osMutex (MutexIsr));  break;
	case 1: ISR_OsStat  = osMutexWait    (ISR_MutexId, 0);      break;
	case 2: ISR_OsStat  = osMutexRelease (ISR_MutexId);         break;
	case 3: ISR_OsStat  = osMutexDelete  (ISR_MutexId);         break;
	}

	Isr.Ex_Res  = res;
}

/*-----------------------------------------------------------------------------
 * Mutex locking thread
 *----------------------------------------------------------------------------*/
static void Th_MutexLock (void const *arg) {
	osThreadId *ctrl_id = (osThreadId *)arg;
	osEvent    evt;

	/* Obtain a mutex */
	ASSERT_TRUE (osMutexWait (G_MutexId, 0) == osOK);

	/* Wait for signal from the control thread */
	evt = osSignalWait (0x01, 100);
	ASSERT_TRUE (evt.status == osEventSignal);

	/* Release a mutex object */
	ASSERT_TRUE (osMutexRelease (G_MutexId) == osOK);

	/* Inform control thread that mutex is released */
	osSignalSet (*ctrl_id, 0x01);
	osDelay(500);                         /* This call should never return      */
}

/*-----------------------------------------------------------------------------
 * Recursive mutex acquisition
 *----------------------------------------------------------------------------*/
static void RecursiveMutexAcquire (uint32_t depth, uint32_t ctrl) {
	static uint32_t acq;                  /* Mutex acquisition counter          */
	osStatus stat;

	/* Acquire a mutex */
	stat = osMutexWait (G_MutexId, 100);
	ASSERT_TRUE (stat == osOK);

	if (stat == osOK) {
		if (ctrl == depth) {
			/* - Verify that mutex was acquired at count zero */
			ASSERT_TRUE (acq == 0);
		}
		acq++;

		if (depth) {
			RecursiveMutexAcquire (depth - 1, ctrl);
		}
		acq--;

		/* Release a mutex */
		stat = osMutexRelease (G_MutexId);
		ASSERT_TRUE (stat == osOK);
	}
}

/*-----------------------------------------------------------------------------
 * Thread waiting for mutex with G_MutexId
 *----------------------------------------------------------------------------*/
static void Th_MutexWait (void const *arg) {
	//fprintf(stderr, "Th_MutexWait entry\n");
	RecursiveMutexAcquire (3, 3);
	//fprintf(stderr, "Th_MutexWait exit\n");

	osThreadTerminate(osThreadGetId());
}


/*-----------------------------------------------------------------------------
 * Low priority job used for priority inversion test
 *----------------------------------------------------------------------------*/
void Th_LowPrioJob (void const *arg) {
	osThreadId *ctrl_id = (osThreadId *)arg;
	osStatus    stat;
	uint32_t    i;

	/* Obtain a mutex object */
	stat = osMutexWait (G_MutexId, 0);
	ASSERT_TRUE (stat == osOK);

	if (stat == osOK) {
		/* Mutex acquired, inform control thread */
		osSignalSet (*ctrl_id, (1 << 0));

		/* Set mark into execution array */
		for (i = 0; i < 3; i++) {
			if (G_ExecArr[i] == 0) {
				G_ExecArr[i] = 'L';             /* L as Low priority job              */

				/* Inform control thread */
				osSignalSet (*ctrl_id, (1 << 1));
				break;
			}
		}

		ASSERT_TRUE (osMutexRelease (G_MutexId) == osOK);
	}

	osThreadTerminate(osThreadGetId());
}

/*-----------------------------------------------------------------------------
 * Medium priority job used for priority inversion test
 *----------------------------------------------------------------------------*/
void Th_MediumPrioJob (void const *arg) {
	osThreadId *ctrl_id = (osThreadId *)arg;
	uint32_t    i;

	/* Set mark into execution array */
	for (i = 0; i < 3; i++) {
		if (G_ExecArr[i] == 0) {
			G_ExecArr[i] = 'M';             /* M as Medium priority job           */

			/* Inform control thread */
			osSignalSet (*ctrl_id, (1 << 2));
			break;
		}
	}

	osThreadTerminate(osThreadGetId());
}

/*-----------------------------------------------------------------------------
 * High priority job used for priority inversion test
 *----------------------------------------------------------------------------*/
void Th_HighPrioJob (void const *arg) {
	osThreadId *ctrl_id = (osThreadId *)arg;
	uint32_t    i;
	osStatus    stat;

	/* Allow control thread to be executed while this thread waits */
	osSignalSet (*ctrl_id, 0x01);

	/* Wait for a mutex object */
	stat = osMutexWait (G_MutexId, 200);
	ASSERT_TRUE (stat == osOK);

	if (stat == osOK) {
		/* Set mark into execution array */
		for (i = 0; i < 3; i++) {
			if (G_ExecArr[i] == 0) {
				G_ExecArr[i] = 'H';             /* H as High priority job             */

				/* Inform control thread */
				osSignalSet (*ctrl_id, (1 << 3));
				break;
			}
		}

		// [ILG]
		// Must release mutex before thread exit
		osMutexRelease (G_MutexId);
	}

	osThreadTerminate(osThreadGetId());
}

/*-----------------------------------------------------------------------------
 * Low priority thread which acquires a mutex object
 *----------------------------------------------------------------------------*/
void Th_MutexAcqLow  (void const *arg) {
	ASSERT_TRUE (osMutexWait (G_MutexId, 0) == osOK);

	osSignalWait (0x01, osWaitForever);

	// [ILG]
	// Must release mutex before thread exit
	osMutexRelease (G_MutexId);

	osThreadTerminate(osThreadGetId());
}

/*-----------------------------------------------------------------------------
 * High priority thread which releases a mutex object
 *----------------------------------------------------------------------------*/
void Th_MutexRelHigh (void const *arg) {
	ASSERT_TRUE (osMutexRelease (G_MutexId) == osErrorResource);

	osSignalWait (0x01, osWaitForever);

	osThreadTerminate(osThreadGetId());
}

/*-----------------------------------------------------------------------------
 *      Test cases
 *----------------------------------------------------------------------------*/

/*=======0=========1=========2=========3=========4=========5=========6=========7=========8=========9=========0=========1====*/
/**
\defgroup mutex_func Mutex Functions
\brief Mutex Functions Test Cases
\details
The test cases check the osMutex* functions.

@{
*/

/*=======0=========1=========2=========3=========4=========5=========6=========7=========8=========9=========0=========1====*/
/**
\brief Test case: TC_MutexBasic
\details
- Create a mutex object
- Try to release mutex that was not obtained before
- Obtain a mutex object
- Release a mutex object
- Delete a mutex object
*/
void TC_MutexBasic (void) {
	osMutexId m_id;

	/* - Create a mutex object */
	m_id = osMutexCreate (osMutex (MutexBas));
	ASSERT_TRUE (m_id != NULL);

	if (m_id != NULL) {
		/* - Try to release mutex that was not obtained before */
		ASSERT_TRUE (osMutexRelease (m_id) == osErrorResource);
		/* - Obtain a mutex object */
		ASSERT_TRUE (osMutexWait (m_id, 0) == osOK);
		/* - Release a mutex object */
		ASSERT_TRUE (osMutexRelease (m_id) == osOK);
		/* - Delete a mutex object */
		ASSERT_TRUE (osMutexDelete (m_id)  == osOK);
	}
}

/*=======0=========1=========2=========3=========4=========5=========6=========7=========8=========9=========0=========1====*/
/**
\brief Test case: TC_MutexTimeout
\details
- Create and initialize a mutex object
- Create a thread that acquires a mutex but never release it
- Wait for mutex release until timeout
*/
void TC_MutexTimeout (void) {
	osThreadId ctrl_id, lock_id;
	osEvent    evt;

	/* Get control thread id */
	ctrl_id = osThreadGetId ();
	ASSERT_TRUE (ctrl_id != NULL);

	if (ctrl_id != NULL) {
		/* - Create and initialize a mutex object */
		G_MutexId = osMutexCreate (osMutex (MutexTout));
		ASSERT_TRUE (G_MutexId != NULL);

		if (G_MutexId != NULL) {
			/* - Create a thread that acquires a mutex but never release it */
			lock_id = osThreadCreate (osThread (Th_MutexLock), &ctrl_id);
			ASSERT_TRUE (lock_id != NULL);

			if (lock_id != NULL) {
				/* - Wait for mutex release until timeout */
				ASSERT_TRUE (osMutexWait (G_MutexId, 10) == osErrorTimeoutResource);
				/* - Release a mutex */
				osSignalSet (lock_id, 0x01);
				evt = osSignalWait (0x01, 100);
				ASSERT_TRUE (evt.status == osEventSignal);
				/* - Terminate locking thread */
				ASSERT_TRUE (osThreadTerminate (lock_id)  == osOK);
			}

			/* Delete mutex object */
			ASSERT_TRUE (osMutexDelete (G_MutexId)  == osOK);
		}
	}
}

/*=======0=========1=========2=========3=========4=========5=========6=========7=========8=========9=========0=========1====*/
/**
\brief Test case: TC_MutexNestedAcquire
\details
- Create a mutex object
- Obtain a mutex object
- Create a high priority thread that waits for the same mutex
- Recursively acquire and release a mutex object
- Release a mutex
- Verify that every subsequent call released the mutex
- Delete a mutex object
- Mutex object must be released after each acquisition
*/
void TC_MutexNestedAcquire (void) {
	osStatus stat;

	/* - Create a mutex object */
	G_MutexId = osMutexCreate (osMutex (Mutex_Nest));
	ASSERT_TRUE (G_MutexId != NULL);

	if (G_MutexId  != NULL) {
		/* - Obtain a mutex object */
		stat = osMutexWait (G_MutexId, 0);
		ASSERT_TRUE (stat == osOK);

		if (stat == osOK) {
			/* - Create a high priority thread that will wait for the same mutex */
			G_Mutex_ThreadId = osThreadCreate (osThread (Th_MutexWait), NULL);
			ASSERT_TRUE (G_Mutex_ThreadId != NULL);

			/* - Recursively acquire and release a mutex object */
			//fprintf(stderr, "TC_MutexNestedAcquire\n");
			RecursiveMutexAcquire (5, 5);

			/* - Release a mutex */
			stat = osMutexRelease (G_MutexId);
			ASSERT_TRUE (stat == osOK);

			// [ILG] give the thread time to do its number
			osDelay(100);

			/* - Verify that every subsequent call released the mutex */
			ASSERT_TRUE (osMutexRelease (G_MutexId) == osErrorResource);
		}

		/* - Delete a mutex object */
		ASSERT_TRUE (osMutexDelete (G_MutexId) == osOK);
	}
}

/*=======0=========1=========2=========3=========4=========5=========6=========7=========8=========9=========0=========1====*/
/**
\brief Test case: TC_MutexPriorityInversion
\details
- Set priority of the control thread to normal
- Create low priority thread, which will acquire mutex
- Wait until low priority thread acquires mutex
- Raise priority of the control thread
- Create high priority thread, which will wait for a mutex
- Allow high priority job to wait for mutex
- Create medium priority thread
- Set priority of the control thread to be the lowest of all
- Wait until all jobs finish
- Verify thread execution order
- Thread execution order must be: Low, High, Medium
*/
void TC_MutexPriorityInversion (void) {
	osThreadId ctrl_id, id[3];
	osStatus   stat;
	osEvent    evt;
	uint32_t   i;

	/* Init execution array */
	for (i = 0; i < 3; i++) {
		G_ExecArr[i] = 0;
	}

	/* Get id of the control thread */
	ctrl_id = osThreadGetId();

	/* - Set priority of the control thread to normal */
	stat = osThreadSetPriority (ctrl_id, osPriorityNormal);
	ASSERT_TRUE (stat == osOK);

	if (stat == osOK) {
		/* Create a mutex object */
		G_MutexId = osMutexCreate (osMutex (Mutex_PrioInv));
		ASSERT_TRUE (G_MutexId != NULL);

		if (G_MutexId != NULL) {
			/* - Create low priority thread, which will acquire mutex */
			id[0] = osThreadCreate (osThread (Th_LowPrioJob), &ctrl_id);
			ASSERT_TRUE (id[0] != NULL);

			if (id[0] != NULL) {
				/* - Wait until low priority thread acquires mutex */
				evt = osSignalWait (0x01, 100);
				ASSERT_TRUE (evt.status == osEventSignal);

				if (evt.status == osEventSignal) {
					/* - Raise priority of the control thread */
					stat = osThreadSetPriority (ctrl_id, osPriorityAboveNormal);
					ASSERT_TRUE (stat == osOK);

					if (stat == osOK) {
						/* - Create high priority thread, which will wait for a mutex */
						id[1] = osThreadCreate (osThread (Th_HighPrioJob), &ctrl_id);
						ASSERT_TRUE (id[1] != NULL);

						if (id[1] != NULL) {
							/* - Allow high priority job to wait for mutex */
							osSignalWait (0x01, 100);

							/* - Create medium priority thread */
							id[2] = osThreadCreate (osThread (Th_MediumPrioJob), &ctrl_id);
							ASSERT_TRUE (id[2] != NULL);

							if (id[2] != NULL) {
								/* - Set priority of the control thread to be the lowest of all */
								stat = osThreadSetPriority (ctrl_id, osPriorityLow);
								ASSERT_TRUE (stat == osOK);

								if (stat == osOK) {
									/* Wait until all jobs finish */
									evt = osSignalWait (0x0E, 100);
									ASSERT_TRUE (evt.status == osEventSignal);

									if (evt.status == osEventSignal) {
										/* - Verify thread execution order */
										ASSERT_TRUE (G_ExecArr[0] == 'L');
										ASSERT_TRUE (G_ExecArr[1] == 'H');
										ASSERT_TRUE (G_ExecArr[2] == 'M');
									}
								}
							}
						}
					}
				}
			}
		}

		/* - Delete mutex object */
		ASSERT_TRUE (osMutexDelete (G_MutexId) == osOK);

		/* - Restore priority of the control thread to normal */
		stat = osThreadSetPriority (ctrl_id, osPriorityNormal);
		ASSERT_TRUE (stat == osOK);
	}
}

/*=======0=========1=========2=========3=========4=========5=========6=========7=========8=========9=========0=========1====*/
/**
\brief Test case: TC_MutexOwnership
\details
- Create a mutex object
- Create a child thread and wait until acquires mutex
- Create a child thread that trys to release a mutex
- Only thread that obtains a mutex can release it.
*/
void TC_MutexOwnership (void) {
	osThreadId ctrl_id, id[2];

	/* Ensure that priority of the control thread is set to normal */
	ctrl_id = osThreadGetId();
	ASSERT_TRUE (osThreadSetPriority (ctrl_id, osPriorityNormal) == osOK);

	/* - Create a mutex object */
	G_MutexId = osMutexCreate (osMutex (Mutex_Ownership));
	ASSERT_TRUE (G_MutexId != NULL);

	if (G_MutexId != NULL) {
		/* - Create a child thread and wait until acquires mutex */
		id[0] = osThreadCreate (osThread (Th_MutexAcqLow), NULL);
		ASSERT_TRUE (id[0] != NULL);

		if (id[0] != NULL) {
			osDelay(10);

			/* - Create a child thread that trys to release a mutex */
			id[1] = osThreadCreate (osThread (Th_MutexRelHigh), NULL);
			ASSERT_TRUE (id[1] != NULL);

			/* Terminate both threads */
			if (id[1] != NULL) {
				//ASSERT_TRUE (osThreadTerminate (id[1]) == osOK);
				osSignalSet(id[1], 0x1);
			}

			// [ILG]
			// Notify the thread that acquired the mutex to release it.
			osSignalSet(id[0], 0x1);
			osDelay(10);

			//ASSERT_TRUE (osThreadTerminate (id[0]) == osOK);
		}
	}

	/* - Delete a mutex object */
	ASSERT_TRUE (osMutexDelete (G_MutexId) == osOK);
}

/*=======0=========1=========2=========3=========4=========5=========6=========7=========8=========9=========0=========1====*/
/**
\brief Test case: TC_MutexParam
\details
- Test mutex management functions with invalid parameters
*/
void TC_MutexParam (void) {
	ASSERT_TRUE (osMutexCreate  (NULL)    == NULL);
	ASSERT_TRUE (osMutexRelease (NULL)    == osErrorParameter);
	ASSERT_TRUE (osMutexWait    (NULL, 0) == osErrorParameter);
}

/*=======0=========1=========2=========3=========4=========5=========6=========7=========8=========9=========0=========1====*/
/**
\brief Test case: TC_MutexInterrupts
\details
- Call all mutex management functions from the ISR
*/
void TC_MutexInterrupts (void) {

	TST_IRQHandler = Mutex_IRQHandler;

	//NVIC_EnableIRQ((IRQn_Type)0);

	Isr.Ex_Num = 0; /* Test: osMutexCreate */

	//NVIC_SetPendingIRQ((IRQn_Type)0);
	EnterInterrupt();

	// [ILG]
	// Do not expect infinite speed
	osDelay(2);
	ExitInterrupt();

	ASSERT_TRUE (ISR_MutexId == NULL);

	Isr.Ex_Num = 1; /* Test: osMutexWait */

	/* Create valid mutex, to be used for ISR function calls */
	ISR_MutexId = osMutexCreate  (osMutex (MutexIsr));
	ASSERT_TRUE (ISR_MutexId != NULL);

	if (ISR_MutexId != NULL) {

		// [ILG]
		ISR_OsStat = osOK;

		//NVIC_SetPendingIRQ((IRQn_Type)0);
		EnterInterrupt();

		// [ILG]
		// Do not expect infinite speed
		osDelay(2);
		ExitInterrupt();

		ASSERT_TRUE (ISR_OsStat == osErrorISR);

		// [ILG]
		ISR_OsStat = osOK;

		Isr.Ex_Num = 2; /* Test: osMutexRelease */
		//NVIC_SetPendingIRQ((IRQn_Type)0);
		EnterInterrupt();

		// [ILG]
		// Do not expect infinite speed
		osDelay(2);
		ExitInterrupt();

		ASSERT_TRUE (ISR_OsStat == osErrorISR);

		// [ILG]
		ISR_OsStat = osOK;

		Isr.Ex_Num = 3; /* Test: osMutexDelete */
		//NVIC_SetPendingIRQ((IRQn_Type)0);
		EnterInterrupt();

		// [ILG]
		// Do not expect infinite speed
		osDelay(2);
		ExitInterrupt();

		ASSERT_TRUE (ISR_OsStat == osErrorISR);

		/* Delete mutex */
		ASSERT_TRUE (osMutexDelete (ISR_MutexId) == osOK);
	}

	//NVIC_DisableIRQ((IRQn_Type)0);
}

/**
@}
*/
// end of group mutex_func

/*-----------------------------------------------------------------------------
 * End of file
 *----------------------------------------------------------------------------*/

