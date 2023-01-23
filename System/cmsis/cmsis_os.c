/*
 ******************************************************************************
 * $File    cmsis_os.c
 * $Brief   C Source file of CMSIS RTOS API.
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


/*
 * Copyright (c) 2015, Lab A Part
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this
 * o list of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "cmsis_os.h"

#define	PRIORITY_BIAS	4

// Convert CMSIS-RTOS priority to FreeRTOS priority
static unsigned portBASE_TYPE ConvertToFreeRTOSPriority(osPriority prior)
{
	return ((unsigned portBASE_TYPE)(((int)prior) + PRIORITY_BIAS));
}


#if (INCLUDE_uxTaskPriorityGet == 1)
// Convert FreeRTOS priority to CMSIS-RTOS priority
static osPriority ConvertToCmsisPriority(unsigned portBASE_TYPE prior)
{
	return ((osPriority)(((int)prior) - PRIORITY_BIAS));
}
#endif


// Thread mode or Handler mode?
static int inHandlerMode(void)
{
	return (__get_IPSR() != 0);
}
//#define	inHandlerMode()		(__get_IPSR() != 0)

///////////////////////// Kernel Information and Control /////////////////////////

#if ( (osFeature_Signal == 24) || (osFeature_Signal == 8) )			// Signal uses EventGroup
typedef struct os_thread_cb {
	TaskHandle_t		Handler;
	EventGroupHandle_t	EventGroup;
} os_thread_cb_t;

os_thread_cb_t	ThreadTable[MaxNumOfThread];
#endif																// Signal uses EventGroup

/// Initialize the RTOS Kernel for creating objects.
/// \return status code that indicates the execution status of the function.
/// \note MUST REMAIN UNCHANGED: \b osKernelInitialize shall be consistent in every CMSIS-RTOS.
osStatus osKernelInitialize(void)
{
#if ( (osFeature_Signal == 24) || (osFeature_Signal == 8) )			// Signal uses EventGroup
	memset(ThreadTable, 0, sizeof(os_thread_cb_t) * MaxNumOfThread);
#endif																// Signal uses EventGroup
  	osNVICInterruptInit();

	return osOK;
}

/// Start the RTOS Kernel.
/// \return status code that indicates the execution status of the function.
/// \note MUST REMAIN UNCHANGED: \b osKernelStart shall be consistent in every CMSIS-RTOS.
osStatus osKernelStart(void)
{
	// void vTaskStartScheduler( void ) PRIVILEGED_FUNCTION;
	vTaskStartScheduler();

	return osOK;
}

/// Check if the RTOS kernel is already started.
/// \note MUST REMAIN UNCHANGED: \b osKernelRunning shall be consistent in every CMSIS-RTOS.
/// \return 0 RTOS is not started, 1 RTOS is started.
int32_t osKernelRunning(void)
{
	int32_t ret = 0;

#if ( INCLUDE_xTaskGetSchedulerState == 1 )		// INCLUDE_xTaskGetSchedulerState
	//BaseType_t xTaskGetSchedulerState( void );
	if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
		ret = 1;
#endif											// INCLUDE_xTaskGetSchedulerState

	return ret;
}


#if (defined (osFeature_SysTick)  &&  (osFeature_SysTick != 0))

/// Get the RTOS kernel system timer counter
/// \note MUST REMAIN UNCHANGED: \b osKernelSysTick shall be consistent in every CMSIS-RTOS.
/// \return RTOS kernel system timer as 32-bit value
uint32_t osKernelSysTick(void)
{
	if (inHandlerMode()) {
		//TickType_t xTaskGetTickCountFromISR( void );
		return xTaskGetTickCountFromISR();
	} else {
		//TickType_t xTaskGetTickCount( void );
		return xTaskGetTickCount();
	}
}

#endif    // System Timer available



///////////////////////// Thread Management /////////////////////////

/// Create a thread and add it to Active Threads and set it to state READY.
/// \param[in]     thread_def    thread definition referenced with \ref osThread.
/// \param[in]     argument      pointer that is passed to the thread function as start argument.
/// \return thread ID for reference by other functions or NULL in case of error.
/// \note MUST REMAIN UNCHANGED: \b osThreadCreate shall be consistent in every CMSIS-RTOS.
osThreadId osThreadCreate(const osThreadDef_t *thread_def, void *argument)
{
	osThreadId TaskID = NULL;
	unsigned portBASE_TYPE	RtosPrior;

	if (inHandlerMode()) {
		return (TaskID);
	} else {
#if ( (osFeature_Signal == 24) || (osFeature_Signal == 8) )			// Signal uses EventGroup
		register int i;
#endif																// Signal uses EventGroup

		configSTACK_DEPTH_TYPE usStackDepth = configMINIMAL_STACK_SIZE;

		if (thread_def == NULL) {
			return (TaskID);
		}

		if ((thread_def->tpriority < osPriorityIdle) || (thread_def->tpriority > osPriorityRealtime)) {
			return (TaskID);
		}

		RtosPrior = ConvertToFreeRTOSPriority(thread_def->tpriority);

		if (usStackDepth < thread_def->stacksize) {
			usStackDepth = thread_def->stacksize;
		}

#if ( configSUPPORT_STATIC_ALLOCATION == 1 )	// configSUPPORT_STATIC_ALLOCATION
		// 	TaskHandle_t xTaskCreateStatic(	TaskFunction_t pxTaskCode,
		//									const char * const pcName,
		//									const uint32_t ulStackDepth,
		//									void * const pvParameters,
		//									UBaseType_t uxPriority,
		//									StackType_t * const puxStackBuffer,
		//									StaticTask_t * const pxTaskBuffer ) PRIVILEGED_FUNCTION;
		if ((thread_def->pStk != NULL) && (thread_def->pTCB != NULL)) {
#if ( (osFeature_Signal == 24) || (osFeature_Signal == 8) )			// Signal uses EventGroup
			for (i = 0; i < MaxNumOfThread; i++) {
				if (ThreadTable[i].Handler == NULL) {
					TaskID = &ThreadTable[i];
					break;
				}
			}
			if (TaskID == NULL) {
				return TaskID;
			}

			TaskID->Handler = xTaskCreateStatic((TaskFunction_t)thread_def->pthread, thread_def->name, usStackDepth, argument, RtosPrior, thread_def->pStk, thread_def->pTCB);
			if (thread_def->pSGN != NULL) {
				// EventGroupHandle_t xEventGroupCreateStatic( StaticEventGroup_t *pxEventGroupBuffer );
				TaskID->EventGroup = xEventGroupCreateStatic(thread_def->pSGN);
			} else {
				// EventGroupHandle_t xEventGroupCreate( void );
				TaskID->EventGroup = xEventGroupCreate();
			}
#elif ( osFeature_Signal == 31 )									// Signal uses Notify
			TaskID = xTaskCreateStatic((TaskFunction_t)thread_def->pthread, thread_def->name, usStackDepth, argument, RtosPrior, thread_def->pStk, thread_def->pTCB);
#endif																// Signal uses EventGroup
		} else
#endif											// configSUPPORT_STATIC_ALLOCATION
#if ( configSUPPORT_DYNAMIC_ALLOCATION == 1 )	// configSUPPORT_DYNAMIC_ALLOCATION
		{
			//	BaseType_t xTaskCreate(	TaskFunction_t pxTaskCode,
			//							const char * const pcName,
			//							const configSTACK_DEPTH_TYPE usStackDepth,
			//							void * const pvParameters,
			//							UBaseType_t uxPriority,
			//							TaskHandle_t * const pxCreatedTask ) PRIVILEGED_FUNCTION;
#if ( (osFeature_Signal == 24) || (osFeature_Signal == 8) )			// Signal uses EventGroup
			for (i = 0; i < MaxNumOfThread; i++) {
				if (ThreadTable[i].Handler == NULL) {
					TaskID = (osThreadId)&ThreadTable[i];
					kprintf("\n%d %08X", i, TaskID);
					break;
				}
			}
			if (TaskID == NULL) {
				return TaskID;
			}

			//if (xTaskCreate((TaskFunction_t)thread_def->pthread, thread_def->name, usStackDepth, argument, RtosPrior, &TaskID->Handler) == pdPASS) {
			if (xTaskCreate((TaskFunction_t)thread_def->pthread, thread_def->name, usStackDepth, argument, RtosPrior, &TaskID->Handler) == pdPASS) {
				ThreadTable[i].Handler = TaskID->Handler;
				kprintf("\t%08X\n", TaskID->Handler);

				// EventGroupHandle_t xEventGroupCreate( void );
				TaskID->EventGroup = xEventGroupCreate();
			} else {
				return NULL;
			}
#elif ( osFeature_Signal == 31 )									// Signal uses Notify
			if (xTaskCreate((TaskFunction_t)thread_def->pthread, thread_def->name, usStackDepth, argument, RtosPrior, &TaskID) != pdPASS) {
				TaskID = NULL;
			}
#endif																// Signal uses EventGroup
		}
#endif											// configSUPPORT_DYNAMIC_ALLOCATION
		;
	}

	return (TaskID);
}

/// Return the thread ID of the current running thread.
/// \return thread ID for reference by other functions or NULL in case of error.
/// \note MUST REMAIN UNCHANGED: \b osThreadGetId shall be consistent in every CMSIS-RTOS.
osThreadId osThreadGetId(void)
{
	osThreadId TaskID = NULL;

	if (inHandlerMode()) {
		return (TaskID);
	} else {
#if ( (osFeature_Signal == 24) || (osFeature_Signal == 8) )			// Signal uses EventGroup
		int i;

		if (inHandlerMode()) {
			TaskID = NULL;
		} else {
			// TaskHandle_t xTaskGetCurrentTaskHandle( void );
			TaskHandle_t pCurHandler = xTaskGetCurrentTaskHandle();
			kprintf("\nP:%08X\t", pCurHandler);

			for (i = 0; i < MaxNumOfThread; i++) {
				TaskID = ((osThreadId)&ThreadTable[i]);
				kprintf("\t%08X\t%08X\t\t", TaskID, TaskID->Handler);
				if (TaskID->Handler == pCurHandler) {
					kprintf("B:%08X", TaskID);
					break;
				}
			}
		}
#elif ( osFeature_Signal == 31 )									// Signal uses Notify
		TaskID = (osThreadId)xTaskGetCurrentTaskHandle();
#endif																// Signal uses EventGroup
	}

	return TaskID;
}

/// Terminate execution of a thread and remove it from Active Threads.
/// \param[in]     thread_id   thread ID obtained by \ref osThreadCreate or \ref osThreadGetId.
/// \return status code that indicates the execution status of the function.
/// \note MUST REMAIN UNCHANGED: \b osThreadTerminate shall be consistent in every CMSIS-RTOS.
osStatus osThreadTerminate(osThreadId thread_id)
{
	osStatus ret = osErrorResource;

	if (inHandlerMode()) {
		ret = osErrorISR;
	} else {
#if ( (osFeature_Signal == 24) || (osFeature_Signal == 8) )			// Signal uses EventGroup
#if (INCLUDE_vTaskDelete == 1)		// INCLUDE_vTaskDelete
		if ((thread_id == NULL) || (thread_id->Handler == NULL)) {
			ret = osErrorParameter;
		} else {
			if (thread_id->EventGroup != NULL) {
				// void vEventGroupDelete( EventGroupHandle_t xEventGroup );
				vEventGroupDelete(thread_id->EventGroup);
				thread_id->EventGroup = NULL;
			}
			// void vTaskDelete( TaskHandle_t pxTask );
			vTaskDelete((TaskHandle_t)thread_id->Handler);
			thread_id->Handler = NULL;

			ret = osOK;
		}
#endif								// INCLUDE_vTaskDelete
#elif ( osFeature_Signal == 31 )									// Signal uses Notify
#if (INCLUDE_vTaskDelete == 1)		// INCLUDE_vTaskDelete
		if (thread_id == NULL) {
			ret = osErrorParameter;
		} else {
			// void vTaskDelete( TaskHandle_t pxTask );
			vTaskDelete((TaskHandle_t)thread_id);

			ret = osOK;
		}
#endif								// INCLUDE_vTaskDelete
#endif																// Signal uses EventGroup
	}

	return ret;
}

/// Pass control to next thread that is in state \b READY.
/// \return status code that indicates the execution status of the function.
/// \note MUST REMAIN UNCHANGED: \b osThreadYield shall be consistent in every CMSIS-RTOS.
osStatus osThreadYield(void)
{
	osStatus ret = osOK;

	if (inHandlerMode()) {
		ret = osErrorISR;
	} else {
		taskYIELD();
	}

	return ret;
}

/// Change priority of an active thread.
/// \param[in]     thread_id     thread ID obtained by \ref osThreadCreate or \ref osThreadGetId.
/// \param[in]     priority      new priority value for the thread function.
/// \return status code that indicates the execution status of the function.
/// \note MUST REMAIN UNCHANGED: \b osThreadSetPriority shall be consistent in every CMSIS-RTOS.
osStatus osThreadSetPriority(osThreadId thread_id, osPriority priority)
{
	osStatus ret = osErrorPriority;

	if (inHandlerMode()) {
		ret = osErrorISR;
	} else {
		if ((priority < osPriorityIdle) || (priority > osPriorityRealtime)) {
			return (ret);
		}

#if ( (osFeature_Signal == 24) || (osFeature_Signal == 8) )			// Signal uses EventGroup
#if ( INCLUDE_vTaskPrioritySet == 1 )		// INCLUDE_vTaskPrioritySet
		if ((thread_id == NULL) || (thread_id->Handler == NULL)) {
			ret = osErrorParameter;
		} else {
			if (inHandlerMode()) {
				ret = osErrorISR;
			} else {
				// void vTaskPrioritySet( TaskHandle_t pxTask, UBaseType_t uxNewPriority );
				vTaskPrioritySet((TaskHandle_t)thread_id->Handler, ConvertToFreeRTOSPriority(priority));

				ret = osOK;
			}
		}
#endif										// INCLUDE_vTaskPrioritySet
#elif ( osFeature_Signal == 31 )									// Signal uses Notify
#if ( INCLUDE_vTaskPrioritySet == 1 )		// INCLUDE_vTaskPrioritySet
		if (thread_id == NULL) {
			ret = osErrorParameter;
		} else {
			// void vTaskPrioritySet( TaskHandle_t pxTask, UBaseType_t uxNewPriority );
			vTaskPrioritySet((TaskHandle_t)thread_id, ConvertToFreeRTOSPriority(priority));

			ret = osOK;
		}
#endif										// INCLUDE_vTaskPrioritySet
#endif																// Signal uses EventGroup
	}

	return ret;
}

/// Get current priority of an active thread.
/// \param[in]     thread_id     thread ID obtained by \ref osThreadCreate or \ref osThreadGetId.
/// \return current priority value of the thread function.
/// \note MUST REMAIN UNCHANGED: \b osThreadGetPriority shall be consistent in every CMSIS-RTOS.
osPriority osThreadGetPriority(osThreadId thread_id)
{
	osPriority ret = osPriorityError;

	if (inHandlerMode()) {
		ret = osErrorISR;
	} else {
#if ( (osFeature_Signal == 24) || (osFeature_Signal == 8) )			// Signal uses EventGroup
#if ( INCLUDE_uxTaskPriorityGet == 1 )		// INCLUDE_vTaskPrioritySet
		if ((thread_id == NULL) || (thread_id->Handler == NULL)) {
			ret = osErrorParameter;
		} else {
			if (inHandlerMode()) {
				ret = ConvertToCmsisPriority(uxTaskPriorityGetFromISR((TaskHandle_t)thread_id->Handler));
			} else {
				ret = ConvertToCmsisPriority(uxTaskPriorityGet((TaskHandle_t)thread_id->Handler));
			}
		}
#endif										// INCLUDE_vTaskPrioritySet
#elif ( osFeature_Signal == 31 )									// Signal uses Notify
#if ( INCLUDE_uxTaskPriorityGet == 1 )		// INCLUDE_vTaskPrioritySet
		if (thread_id == NULL) {
			ret = osErrorParameter;
		} else {
			ret = ConvertToCmsisPriority(uxTaskPriorityGet((TaskHandle_t)thread_id));
		}
#endif										// INCLUDE_vTaskPrioritySet
#endif																// Signal uses EventGroup
	}

	return ret;
}



///////////////////////// Generic Wait Functions /////////////////////////
/// Wait for Timeout (Time Delay).
/// \param[in]     millisec      \ref CMSIS_RTOS_TimeOutValue "time delay" value
/// \return status code that indicates the execution status of the function.
osStatus osDelay(uint32_t millisec)
{
	osStatus ret = osErrorOS;

#if	(INCLUDE_vTaskDelay == 1)		// INCLUDE_vTaskDelay
	if (inHandlerMode()) {
		ret = osErrorISR;
	} else {
		// void vTaskDelay( TickType_t xTicksToDelay );
		vTaskDelay( pdMS_TO_TICKS( millisec ) );
		ret = osEventTimeout;
	}
#endif		//INCLUDE_vTaskDelay

	return ret;
}

#if (defined (osFeature_Wait)  &&  (osFeature_Wait != 0))     // Generic Wait available

/// Wait for Signal, Message, Mail, or Timeout.
/// \param[in] millisec          \ref CMSIS_RTOS_TimeOutValue or 0 in case of no time-out
/// \return event that contains signal, message, or mail information or error code.
/// \note MUST REMAIN UNCHANGED: \b osWait shall be consistent in every CMSIS-RTOS.
osEvent osWait(uint32_t millisec)
{
	osEvent ret;

	ret.status = osErrorOS;

	return ret;
}

#endif  // Generic Wait available



///////////////////////// Timer Management /////////////////////////
/// Create a timer.
/// \param[in]     timer_def     timer object referenced with \ref osTimer.
/// \param[in]     type          osTimerOnce for one-shot or osTimerPeriodic for periodic behavior.
/// \param[in]     argument      argument to the timer call back function.
/// \return timer ID for reference by other functions or NULL in case of error.
/// \note MUST REMAIN UNCHANGED: \b osTimerCreate shall be consistent in every CMSIS-RTOS.
osTimerId osTimerCreate(const osTimerDef_t *timer_def, os_timer_type type, void *argument)
{
	TimerHandle_t TimerHandler = NULL;

#if ( configUSE_TIMERS == 1 )		// configUSE_TIMERS
	UBaseType_t	Periodic = pdFALSE;

	if (inHandlerMode()) {
		return TimerHandler;
	} else {
		if (timer_def == NULL) {
			return TimerHandler;
		}

		if (type == osTimerPeriodic) Periodic = pdTRUE;

#if ( configSUPPORT_STATIC_ALLOCATION == 1 )		// configSUPPORT_STATIC_ALLOCATION
		// 	TimerHandle_t xTimerCreateStatic( const char *pcTimerName,
		//								const TickType_t xTimerPeriod,
		//								const UBaseType_t uxAutoReload,
		//								void * const pvTimerID,
		//								TimerCallbackFunction_t pxCallbackFunction,
		//								StaticTimer_t *pxTimerBuffer );
		if (timer_def->pTB != NULL) {
			TimerHandler = xTimerCreateStatic(NULL, 0xFFFFFFFF, Periodic, argument, (TimerCallbackFunction_t)timer_def->ptimer, (StaticTimer_t *)timer_def->pTB);
		} else
#endif		// configSUPPORT_STATIC_ALLOCATION

#if ( configSUPPORT_DYNAMIC_ALLOCATION == 1 )
		// 	TimerHandle_t xTimerCreate( const char *pcTimerName,
		//								const TickType_t xTimerPeriod,
		//								const UBaseType_t uxAutoReload,
		//								void * const pvTimerID,
		//								TimerCallbackFunction_t pxCallbackFunction );
		{
			TimerHandler = xTimerCreate(NULL, 0xFFFFFFFF, Periodic, argument, (TimerCallbackFunction_t)timer_def->ptimer);
			//vTimerSetTimerID( TimerHandler, ( void * ) argument );
		}
#endif		// configSUPPORT_DYNAMIC_ALLOCATION

#endif		// configUSE_TIMERS
	}

	return TimerHandler;
}

/// Start or restart a timer.
/// \param[in]     timer_id      timer ID obtained by \ref osTimerCreate.
/// \param[in]     millisec      \ref CMSIS_RTOS_TimeOutValue "time delay" value of the timer.
/// \return status code that indicates the execution status of the function.
/// \note MUST REMAIN UNCHANGED: \b osTimerStart shall be consistent in every CMSIS-RTOS.
osStatus osTimerStart(osTimerId timer_id, uint32_t millisec)
{
	osStatus ret = osErrorOS;

#if ( configUSE_TIMERS == 1 )		// configUSE_TIMERS

	if (inHandlerMode()) {
		ret = osErrorISR;
	} else {
		if (timer_id == NULL) {
			ret = osErrorParameter;
		} else {
			//BaseType_t xTimerChangePeriod( TimerHandle_t xTimer,
			//								 TickType_t xNewPeriod,
			//								 TickType_t xTicksToWait );
			if (xTimerChangePeriod((TimerHandle_t)timer_id, pdMS_TO_TICKS(millisec), pdMS_TO_TICKS(osProcessDelayDeadTick)) == pdPASS) {
				ret = osOK;
			}
		}
	}

#endif		// configUSE_TIMERS

	return ret;
}

/// Stop the timer.
/// \param[in]     timer_id      timer ID obtained by \ref osTimerCreate.
/// \return status code that indicates the execution status of the function.
/// \note MUST REMAIN UNCHANGED: \b osTimerStop shall be consistent in every CMSIS-RTOS.
osStatus osTimerStop(osTimerId timer_id)
{
	osStatus ret = osErrorOS;

#if ( configUSE_TIMERS == 1 )		// configUSE_TIMERS

	if (inHandlerMode()) {
		ret = osErrorISR;
	} else {
		if (timer_id == NULL) {
			ret = osErrorParameter;
		} else {
			if (xTimerIsTimerActive((TimerHandle_t)timer_id) == pdFALSE) {
				ret = osErrorResource;
			} else {
				//BaseType_t xTimerStop( TimerHandle_t xTimer, TickType_t xTicksToWait );
				if (xTimerStop((TimerHandle_t)timer_id, pdMS_TO_TICKS(osProcessDelayDeadTick)) == pdPASS) {
					ret = osOK;
				}
			}
		}
	}

#endif		// configUSE_TIMERS

	return ret;
}

/// Delete a timer that was created by \ref osTimerCreate.
/// \param[in]     timer_id      timer ID obtained by \ref osTimerCreate.
/// \return status code that indicates the execution status of the function.
/// \note MUST REMAIN UNCHANGED: \b osTimerDelete shall be consistent in every CMSIS-RTOS.
osStatus osTimerDelete(osTimerId timer_id)
{
	osStatus ret = osErrorOS;

#if ( configUSE_TIMERS == 1 )		// configUSE_TIMERS

	if (inHandlerMode()) {
		ret = osErrorISR;
	} else {
		if (timer_id == NULL) {
			ret = osErrorParameter;
		} else {
			//BaseType_t xTimerDelete( TimerHandle_t xTimer, TickType_t xTicksToWait );
			if (xTimerDelete((TimerHandle_t)timer_id, pdMS_TO_TICKS(osProcessDelayDeadTick)) == pdPASS) {
				timer_id = NULL;

				ret = osOK;
			}
		}
	}

#endif		// configUSE_TIMERS

	return ret;
}



///////////////////////// Signal Management /////////////////////////

#if (defined (osFeature_Signal)  &&  (osFeature_Signal != 0))     // Signal

/// Set the specified Signal Flags of an active thread.
/// \param[in]     thread_id     thread ID obtained by \ref osThreadCreate or \ref osThreadGetId.
/// \param[in]     signals       specifies the signal flags of the thread that should be set.
/// \return previous signal flags of the specified thread or 0x80000000 in case of incorrect parameters.
/// \note MUST REMAIN UNCHANGED: \b osSignalSet shall be consistent in every CMSIS-RTOS.
int32_t osSignalSet(osThreadId thread_id, int32_t signals)
{
	int32_t ret = 0x80000000;

#if ( (osFeature_Signal == 24) || (osFeature_Signal == 8) )					// Signal uses EventGroup
	if ((thread_id != NULL) && (thread_id->Handler != NULL) && (thread_id->EventGroup != NULL) && (signals != 0) && (signals <= SigMax) && (signals >= 1)) {
		if (inHandlerMode()) {
#if ( ( INCLUDE_xTimerPendFunctionCall == 1 ) && ( configUSE_TIMERS == 1 ) )			// ( (configUSE_TIMERS == 1) && (INCLUDE_xTimerPendFunctionCall == 1) )
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;

			// EventBits_t xEventGroupGetBitsFromISR( EventGroupHandle_t xEventGroup );
			ret = xEventGroupGetBitsFromISR(thread_id->EventGroup);

			// BaseType_t xEventGroupSetBitsFromISR( EventGroupHandle_t xEventGroup,
			//										 const EventBits_t uxBitsToSet,
			//										 BaseType_t *pxHigherPriorityTaskWoken );
			xEventGroupSetBitsFromISR(thread_id->EventGroup, (const EventBits_t)signals, &xHigherPriorityTaskWoken);
			if(xHigherPriorityTaskWoken == pdTRUE) {
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
			}
#else																					// ( (configUSE_TIMERS == 1) && (INCLUDE_xTimerPendFunctionCall == 1) )
			ret = ret;
#endif																					// ( (configUSE_TIMERS == 1) && (INCLUDE_xTimerPendFunctionCall == 1) )
		} else {
			// EventBits_t xEventGroupGetBits( EventGroupHandle_t xEventGroup );
			ret = xEventGroupGetBits(thread_id->EventGroup);
			// EventBits_t xEventGroupSetBits( EventGroupHandle_t xEventGroup,
			//									const EventBits_t uxBitsToSet );
			xEventGroupSetBits(thread_id->EventGroup, (const EventBits_t)signals);
		}
	}
#elif ( (osFeature_Signal == 31) && (configUSE_TASK_NOTIFICATIONS == 1) )	// Signal uses Notify and configUSE_TASK_NOTIFICATIONS
	if ((thread_id != NULL) && (signals != 0) && (signals <= SigMax) && (signals >= 1)) {
		uint32_t ulPreviousValue;
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;

		if (inHandlerMode()) {
			// BaseType_t xTaskNotifyAndQueryFromISR( TaskHandle_t xTaskToNotify,
			//											uint32_t ulValue,
			//											eNotifyAction eAction,
			//											uint32_t *pulPreviousNotifyValue,
			//											BaseType_t *pxHigherPriorityTaskWoken );
			if (xTaskNotifyAndQueryFromISR((TaskHandle_t)thread_id, signals, eSetBits, &ulPreviousValue, &xHigherPriorityTaskWoken) == pdPASS) {
				ret = ulPreviousValue;
			}
			if(xHigherPriorityTaskWoken == pdTRUE) {
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
			}
		} else {
			// BaseType_t xTaskNotifyAndQuery( TaskHandle_t xTaskToNotify,
			//									uint32_t ulValue,
			//									eNotifyAction eAction,
			//									uint32_t *pulPreviousNotifyValue );
			if (xTaskNotifyAndQuery((TaskHandle_t)thread_id, signals, eSetBits, &ulPreviousValue) == pdPASS) {
				ret = ulPreviousValue;
			}
		}
	}
#endif																		// Signal uses EventGroup

	return ret;
}

/// Clear the specified Signal Flags of an active thread.
/// \param[in]     thread_id     thread ID obtained by \ref osThreadCreate or \ref osThreadGetId.
/// \param[in]     signals       specifies the signal flags of the thread that shall be cleared.
/// \return previous signal flags of the specified thread or 0x80000000 in case of incorrect parameters or call from ISR.
/// \note MUST REMAIN UNCHANGED: \b osSignalClear shall be consistent in every CMSIS-RTOS.
int32_t osSignalClear(osThreadId thread_id, int32_t signals)
{
	int32_t ret = 0x80000000;

#if ( (osFeature_Signal == 24) || (osFeature_Signal == 8) )					// Signal uses EventGroup
	if ((thread_id != NULL) && (thread_id->Handler != NULL) && (thread_id->EventGroup != NULL) && (signals != 0) && (signals <= SigMax) && (signals >= 1)) {
		if ((thread_id != NULL) && (signals != 0)) {
			if (inHandlerMode()) {
				BaseType_t xHigherPriorityTaskWoken = pdFALSE;

				// EventBits_t xEventGroupGetBitsFromISR( EventGroupHandle_t xEventGroup );
				ret = xEventGroupGetBitsFromISR(thread_id->EventGroup);
				// BaseType_t xEventGroupClearBitsFromISR( EventGroupHandle_t xEventGroup,
				//										 const EventBits_t uxBitsToClear );
				xEventGroupClearBitsFromISR(thread_id->EventGroup, (const EventBits_t)signals);
				if(xHigherPriorityTaskWoken == pdTRUE) {
					portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
				}
			} else {
				// EventBits_t xEventGroupGetBits( EventGroupHandle_t xEventGroup );
				ret = xEventGroupGetBits(thread_id->EventGroup);
				// EventBits_t xEventGroupClearBits( EventGroupHandle_t xEventGroup,
				//									const EventBits_t uxBitsToClear );
				xEventGroupClearBits(thread_id->EventGroup, (const EventBits_t)signals);
			}
		}
	}
#elif ( (osFeature_Signal == 31) && (configUSE_TASK_NOTIFICATIONS == 1) )	// Signal uses Notify and configUSE_TASK_NOTIFICATIONS

	if (inHandlerMode()) {
		return ret;
	} else {
		if ((thread_id != NULL) && (signals <= SigMax) && (signals >= 0)) {
			uint32_t ulPreviousValue;

			xTaskGetNotifyValue( (TaskHandle_t)thread_id, &ulPreviousValue );

			signals = (((~(signals)) & ulPreviousValue) & SigMax);

			// BaseType_t xTaskNotifyAndQuery( TaskHandle_t xTaskToNotify,
			//									uint32_t ulValue,
			//									eNotifyAction eAction,
			//									uint32_t *pulPreviousNotifyValue );
			if (xTaskNotifyAndQuery((TaskHandle_t)thread_id, signals, eSetValueWithOverwrite, NULL) == pdPASS) {
				ret = ulPreviousValue;
			}

			// ToDo : Should need to check the pending clear condition!!!
			// 			William
			// BaseType_t xTaskNotifyStateClear( TaskHandle_t xTask );
			xTaskNotifyStateClear(NULL);
		}
	}

#endif																		// Signal uses EventGroup

	return ret;
}

/// Wait for one or more Signal Flags to become signaled for the current \b RUNNING thread.
/// \param[in]     signals       wait until all specified signal flags set or 0 for any single signal flag.
/// \param[in]     millisec      \ref CMSIS_RTOS_TimeOutValue or 0 in case of no time-out.
/// \return event flag information or error code.
/// \note MUST REMAIN UNCHANGED: \b osSignalWait shall be consistent in every CMSIS-RTOS.
osEvent osSignalWait(int32_t signals, uint32_t millisec)
{
	osEvent event;

	event.status =  osErrorOS;

#if ( (osFeature_Signal == 24) || (osFeature_Signal == 8) )					// Signal uses EventGroup
	if (signals == 0) {
#if (osFeature_Signal == 24)
		signals = 0x00FFFFFF;
#endif
#if (osFeature_Signal == 8)
		signals = 0x000000FF;
#endif
	}

	if ((signals != 0) && (signals <= SigMax) && (signals >= 1)) {
		if (inHandlerMode()) {
			event.status =  osErrorISR;
		} else {
			EventBits_t uxBits;
			osThreadId TaskID = osThreadGetId();

			if (TaskID->EventGroup == NULL) {
				TaskID->EventGroup = xEventGroupCreate();
			}

			// EventBits_t xEventGroupWaitBits( const EventGroupHandle_t xEventGroup,
			//									const EventBits_t uxBitsToWaitFor,
			//									const BaseType_t xClearOnExit,				// Clear on exit
			//									const BaseType_t xWaitForAllBits,			// Wait for ANY bits set
			//									TickType_t xTicksToWait );
			uxBits = xEventGroupWaitBits(TaskID->EventGroup, signals, pdTRUE, pdFALSE, pdMS_TO_TICKS(millisec));

			if (uxBits & signals) {
				event.status = osEventSignal;
				event.value.v = uxBits;
			} else {
				if (millisec == 0) {
					event.status = osOK;
				} else {
					event.status = osEventTimeout;
				}
			}
		}
	} else {
		event.status =  osErrorValue;
	}
#elif ( (osFeature_Signal == 31) && (configUSE_TASK_NOTIFICATIONS == 1) )	// Signal uses Notify and configUSE_TASK_NOTIFICATIONS

	if (inHandlerMode()) {
		event.status =  osErrorISR;
	} else {
		if (signals == 0) {
			signals = SigMax;
		}

		if ((signals <= SigMax) && (signals >= 1)) {
			//BaseType_t xTaskNotifyWait( uint32_t ulBitsToClearOnEntry,
			//							  uint32_t ulBitsToClearOnExit,
			//							  uint32_t *pulNotificationValue,
			//							  TickType_t xTicksToWait );
			if (xTaskNotifyWait(0, signals, (uint32_t *)&event.value.signals, pdMS_TO_TICKS(millisec)) == pdTRUE) {
				event.status =  osEventSignal;
			} else {
				if (millisec == 0) {
					event.status = osOK;
				} else {
					event.status = osEventTimeout;
				}
			}
		} else {
			event.status =  osErrorValue;
		}
	}

#endif																		// Signal uses EventGroup

	return event;
}

#endif	     // Signal



///////////////////////// Mutex Management /////////////////////////
/// Create and Initialize a Mutex object.
/// \param[in]     mutex_def     mutex definition referenced with \ref osMutex.
/// \return mutex ID for reference by other functions or NULL in case of error.
/// \note MUST REMAIN UNCHANGED: \b osMutexCreate shall be consistent in every CMSIS-RTOS.
osMutexId osMutexCreate(const osMutexDef_t *mutex_def)
{
	SemaphoreHandle_t xSemaphoreHandle = NULL;

	if (inHandlerMode()) {
		return (osMutexId)xSemaphoreHandle;
	} else {

#if ( osFeature_Mutex == 'M' )			// osFeature_Mutex == 'M'
		if (mutex_def != NULL) {
#if ( configSUPPORT_STATIC_ALLOCATION == 1 )		// configSUPPORT_STATIC_ALLOCATION
			if (mutex_def->pMB != NULL) {
				// SemaphoreHandle_t xSemaphoreCreateRecursiveMutexStatic( StaticSemaphore_t *pxMutexBuffer );
				xSemaphoreHandle = xSemaphoreCreateMutexStatic(mutex_def->pMB);
			} else
#endif												// configSUPPORT_STATIC_ALLOCATION
#if ( configSUPPORT_DYNAMIC_ALLOCATION == 1 )		// configSUPPORT_DYNAMIC_ALLOCATION
			{
				// SemaphoreHandle_t xSemaphoreCreateRecursiveMutex( void );
				xSemaphoreHandle = xSemaphoreCreateRecursiveMutex();
			}
#endif												// configSUPPORT_DYNAMIC_ALLOCATION
		}
#else									// osFeature_Mutex == 'M'
		if (mutex_def != NULL) {
#if ( configSUPPORT_STATIC_ALLOCATION == 1 )		// configSUPPORT_STATIC_ALLOCATION
			if (mutex_def->pMB != NULL) {
				// SemaphoreHandle_t xSemaphoreCreateBinaryStatic( StaticSemaphore_t *pxSemaphoreBuffer );
				xSemaphoreHandle = xSemaphoreCreateBinaryStatic(mutex_def->pMB);
			} else
#endif												// configSUPPORT_STATIC_ALLOCATION
#if ( configSUPPORT_DYNAMIC_ALLOCATION == 1 )		// configSUPPORT_DYNAMIC_ALLOCATION
			{
				// SemaphoreHandle_t xSemaphoreCreateBinary( void );
				xSemaphoreHandle = xSemaphoreCreateBinary();
				if (xSemaphoreHandle != NULL) {
					/*
					 * Binary semaphores created using the xSemaphoreCreateBinary() function are created ‘empty’, so the
					 * semaphore must first be given before the semaphore can be taken (obtained) using a call to xSemaphoreTake().
					 */
					// BaseType_t xSemaphoreGive( SemaphoreHandle_t xSemaphore );
					xSemaphoreGive(xSemaphoreHandle);
				}
			}
#endif												// configSUPPORT_DYNAMIC_ALLOCATION
			;
		}
#endif									// osFeature_Mutex == 'M'

	}

	return (osMutexId)xSemaphoreHandle;
}

/// Wait until a Mutex becomes available.
/// \param[in]     mutex_id      mutex ID obtained by \ref osMutexCreate.
/// \param[in]     millisec      \ref CMSIS_RTOS_TimeOutValue or 0 in case of no time-out.
/// \return status code that indicates the execution status of the function.
/// \note MUST REMAIN UNCHANGED: \b osMutexWait shall be consistent in every CMSIS-RTOS.
osStatus osMutexWait(osMutexId mutex_id, uint32_t millisec)
{
	osStatus ret = osErrorOS;

	if (inHandlerMode()) {
		ret = osErrorISR;
	} else {
		if (mutex_id == NULL) {
			ret = osErrorParameter;
		}
#if ( osFeature_Mutex == 'M' )			// osFeature_Mutex == 'M'

#if ( configUSE_RECURSIVE_MUTEXES == 1 )		// configUSE_RECURSIVE_MUTEXES
		else {
			// BaseType_t xSemaphoreTake( SemaphoreHandle_t xSemaphore, TickType_t xTicksToWait );
			if (xSemaphoreTakeRecursive((SemaphoreHandle_t)mutex_id, pdMS_TO_TICKS(millisec)) == pdPASS) {
				ret = osOK;
			} else {
				if (millisec) {
					ret = osErrorTimeoutResource;
				} else {
					ret = osErrorResource;
				}
			}
		}
#endif											// configUSE_RECURSIVE_MUTEXES

#else									// osFeature_Mutex == 'M'

#if ( configUSE_COUNTING_SEMAPHORES == 1 )		// configUSE_COUNTING_SEMAPHOR
		else {
			// BaseType_t xSemaphoreTake( SemaphoreHandle_t xSemaphore, TickType_t xTicksToWait );
			if (xSemaphoreTake((SemaphoreHandle_t)mutex_id, pdMS_TO_TICKS(millisec)) == pdPASS) {
				ret = osOK;
			} else {
				if (millisec) {
					ret = osErrorTimeoutResource;
				} else {
					ret = osErrorResource;
				}
			}
		}
#endif											// configUSE_COUNTING_SEMAPHORES

#endif									// osFeature_Mutex == 'M'
	}

	return ret;
}

/// Release a Mutex that was obtained by \ref osMutexWait.
/// \param[in]     mutex_id      mutex ID obtained by \ref osMutexCreate.
/// \return status code that indicates the execution status of the function.
/// \note MUST REMAIN UNCHANGED: \b osMutexRelease shall be consistent in every CMSIS-RTOS.
osStatus osMutexRelease(osMutexId mutex_id)
{
	osStatus ret = osErrorResource;

	if (inHandlerMode()) {
		ret =  osErrorISR;
	} else {
		if (mutex_id == NULL) {
			ret = osErrorParameter;
		}

#if ( osFeature_Mutex == 'M' )			// osFeature_Mutex == 'M'

#if ( configUSE_RECURSIVE_MUTEXES == 1 )		// configUSE_RECURSIVE_MUTEXES
		else {
			// UBaseType_t uxSemaphoreGetCount( SemaphoreHandle_t xSemaphore );
			if (uxSemaphoreGetCount(mutex_id) == 0) {
				// BaseType_t xSemaphoreGiveRecursive( SemaphoreHandle_t xMutex );
				if (xSemaphoreGiveRecursive((SemaphoreHandle_t)mutex_id) == pdPASS) {
					ret = osOK;
				}
			}
		}
#endif											// configUSE_RECURSIVE_MUTEXES

#else									// osFeature_Mutex == 'M'

#if ( configUSE_COUNTING_SEMAPHORES == 1 )		// configUSE_COUNTING_SEMAPHOR
		else {
			// UBaseType_t uxSemaphoreGetCount( SemaphoreHandle_t xSemaphore );
			if (uxSemaphoreGetCount(mutex_id) == 0) {
				// BaseType_t xSemaphoreGive( SemaphoreHandle_t xSemaphore );
				if (xSemaphoreGive((SemaphoreHandle_t)mutex_id) == pdPASS) {
					ret = osOK;
				}
			}
		}
#endif											// configUSE_COUNTING_SEMAPHORES

#endif									// osFeature_Mutex == 'M'

	}

	return ret;
}

/// Delete a Mutex that was created by \ref osMutexCreate.
/// \param[in]     mutex_id      mutex ID obtained by \ref osMutexCreate.
/// \return status code that indicates the execution status of the function.
/// \note MUST REMAIN UNCHANGED: \b osMutexDelete shall be consistent in every CMSIS-RTOS.
osStatus osMutexDelete(osMutexId mutex_id)
{
	osStatus ret = osErrorOS;

	if (inHandlerMode()) {
		ret =  osErrorISR;
	}
#if ( configUSE_COUNTING_SEMAPHORES == 1 )		// configUSE_COUNTING_SEMAPHOR
	else {
		if (mutex_id == NULL) {
			ret = osErrorParameter;
		} else {
			if (inHandlerMode()) {
				ret =  osErrorISR;
			} else {
				// void vSemaphoreDelete( SemaphoreHandle_t xSemaphore );
				vSemaphoreDelete((SemaphoreHandle_t)mutex_id);

				ret = osOK;
			}
		}
	}
#endif											// configUSE_COUNTING_SEMAPHORES

	return ret;
}



///////////////////////// Semaphore Management Functions /////////////////////////
#if (defined (osFeature_Semaphore)  &&  (osFeature_Semaphore != 0))     // Semaphore available

/// Create and Initialize a Semaphore object used for managing resources.
/// \param[in]     semaphore_def semaphore definition referenced with \ref osSemaphore.
/// \param[in]     count         number of available resources.
/// \return semaphore ID for reference by other functions or NULL in case of error.
/// \note MUST REMAIN UNCHANGED: \b osSemaphoreCreate shall be consistent in every CMSIS-RTOS.
osSemaphoreId osSemaphoreCreate(const osSemaphoreDef_t *semaphore_def, int32_t count)
{
	SemaphoreHandle_t xSemaphoreHandle = NULL;
	UBaseType_t uxMaxCnt = osFeature_Semaphore;

	if (inHandlerMode()) {
		return (osSemaphoreId)xSemaphoreHandle;
	} else {
		if (count) {
			uxMaxCnt = count;
		}

#if ( configUSE_COUNTING_SEMAPHORES == 1 )		// configUSE_COUNTING_SEMAPHORES
		if (semaphore_def != NULL) {
#if ( configSUPPORT_STATIC_ALLOCATION == 1 )			// configSUPPORT_STATIC_ALLOCATION
			// 	SemaphoreHandle_t xSemaphoreCreateCountingStatic( UBaseType_t uxMaxCount,
			//													  UBaseType_t uxInitialCount,
			//													  StaticSemaphore_t pxSempahoreBuffer );
			if (semaphore_def->pSB != NULL) {
				xSemaphoreHandle = xSemaphoreCreateCountingStatic(uxMaxCnt, count, semaphore_def->pSB);
			} else
#endif													// configSUPPORT_STATIC_ALLOCATION
#if ( configSUPPORT_DYNAMIC_ALLOCATION == 1 )			// configSUPPORT_DYNAMIC_ALLOCATION
			{
				// 	SemaphoreHandle_t xSemaphoreCreateCounting(	UBaseType_t uxMaxCount,
				//												UBaseType_t uxInitialCount );
				xSemaphoreHandle = xSemaphoreCreateCounting(uxMaxCnt, count);
			}
#endif													// configSUPPORT_DYNAMIC_ALLOCATION
		}
#endif											// configUSE_COUNTING_SEMAPHORES
	}

	return (osSemaphoreId)xSemaphoreHandle;
}

/// Wait until a Semaphore token becomes available.
/// \param[in]     semaphore_id  semaphore object referenced with \ref osSemaphoreCreate.
/// \param[in]     millisec      \ref CMSIS_RTOS_TimeOutValue or 0 in case of no time-out.
/// \return number of available tokens, or -1 in case of incorrect parameters.
/// \note MUST REMAIN UNCHANGED: \b osSemaphoreWait shall be consistent in every CMSIS-RTOS.
int32_t osSemaphoreWait(osSemaphoreId semaphore_id, uint32_t millisec)
{
	int32_t ret = -1;

	if (inHandlerMode()) {
		return ret;
	} else {
#if ( configUSE_COUNTING_SEMAPHORES == 1 )		// configUSE_COUNTING_SEMAPHORES
		if (semaphore_id != NULL) {
			// BaseType_t xSemaphoreTake( SemaphoreHandle_t xSemaphore, TickType_t xTicksToWait );
			if (xSemaphoreTake((SemaphoreHandle_t) semaphore_id, pdMS_TO_TICKS(millisec)) == pdPASS) {
				// UBaseType_t uxSemaphoreGetCount( SemaphoreHandle_t xSemaphore );
				ret = uxSemaphoreGetCount( (SemaphoreHandle_t) semaphore_id );
			} else {
				ret = 0;	// Has no available tokens.
			}
		}
#endif											// configUSE_COUNTING_SEMAPHORES
	}

	return ret;
}

/// Release a Semaphore token.
/// \param[in]     semaphore_id  semaphore object referenced with \ref osSemaphoreCreate.
/// \return status code that indicates the execution status of the function.
/// \note MUST REMAIN UNCHANGED: \b osSemaphoreRelease shall be consistent in every CMSIS-RTOS.
osStatus osSemaphoreRelease(osSemaphoreId semaphore_id)
{
	osStatus ret = osErrorOS;

#if ( configUSE_COUNTING_SEMAPHORES == 1 )		// configUSE_COUNTING_SEMAPHORES
	if (semaphore_id != NULL) {
		if (inHandlerMode()) {
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;

			// BaseType_t xSemaphoreGiveFromISR( SemaphoreHandle_t xSemaphore,
			//									 signed BaseType_t *pxHigherPriorityTaskWoken );
			if (xSemaphoreGiveFromISR((SemaphoreHandle_t) semaphore_id, &xHigherPriorityTaskWoken) == pdPASS) {
				ret = osOK;
			} else {
				ret = osErrorResource;
			}
			if(xHigherPriorityTaskWoken == pdTRUE) {
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
			}
		} else {
			// BaseType_t xSemaphoreGive( SemaphoreHandle_t xSemaphore );
			if (xSemaphoreGive((SemaphoreHandle_t) semaphore_id) == pdPASS) {
				ret = osOK;
			} else {
				ret = osErrorResource;
			}
		}
	} else {
		ret = osErrorParameter;
	}
#endif											// configUSE_COUNTING_SEMAPHORES

	return ret;
}

/// Delete a Semaphore that was created by \ref osSemaphoreCreate.
/// \param[in]     semaphore_id  semaphore object referenced with \ref osSemaphoreCreate.
/// \return status code that indicates the execution status of the function.
/// \note MUST REMAIN UNCHANGED: \b osSemaphoreDelete shall be consistent in every CMSIS-RTOS.
osStatus osSemaphoreDelete(osSemaphoreId semaphore_id)
{
	osStatus ret = osErrorResource;

	if (inHandlerMode()) {
		ret =  osErrorISR;
	}
#if ( configUSE_COUNTING_SEMAPHORES == 1 )		// configUSE_COUNTING_SEMAPHORES
	else {
		if (semaphore_id != NULL) {
			// void vSemaphoreDelete( SemaphoreHandle_t xSemaphore );
			vSemaphoreDelete((SemaphoreHandle_t) semaphore_id);

			ret = osOK;
		} else {
			ret = osErrorParameter;
		}
	}
#endif											// configUSE_COUNTING_SEMAPHORES

	return ret;
}

#endif     // Semaphore available



///////////////////////// Memory Pool Management Functions /////////////////////////
#if (defined (osFeature_Pool)  &&  (osFeature_Pool != 0))		// osFeature_Pool

typedef struct os_pool_cb {
	void 	*pool;
	uint32_t flags;
	uint32_t pool_sz;
	uint32_t item_sz;
} os_pool_cb_t;

/// Create and Initialize a memory pool.
/// \param[in]     pool_def      memory pool definition referenced with \ref osPool.
/// \return memory pool ID for reference by other functions or NULL in case of error.
/// \note MUST REMAIN UNCHANGED: \b osPoolCreate shall be consistent in every CMSIS-RTOS.
osPoolId osPoolCreate(const osPoolDef_t *pool_def)
{
	osPoolId PoolID = NULL;

	if (inHandlerMode()) {
		return PoolID;
	}
#if ( configSUPPORT_DYNAMIC_ALLOCATION == 1 )
	else {
		if (pool_def != NULL) {
			if (pool_def->pool_sz > osFeatureMaxPoolNumber) {
				return PoolID;
			}

			if ( xPortGetFreeHeapSize() < (sizeof(os_pool_cb_t) + (pool_def->pool_sz * pool_def->item_sz) + 8)) {
				return PoolID;
			}

			PoolID = pvPortMalloc(sizeof(os_pool_cb_t));

			PoolID->item_sz = pool_def->item_sz;
			PoolID->pool_sz = pool_def->pool_sz;
			PoolID->flags	= 0;
#endif		// configSUPPORT_DYNAMIC_ALLOCATION
#if ( configSUPPORT_STATIC_ALLOCATION == 1 )
			if (pool_def->pool != NULL) {
				PoolID->pool	= pool_def->pool;
			} else
#endif	// configSUPPORT_STATIC_ALLOCATION == 1
#if ( configSUPPORT_DYNAMIC_ALLOCATION == 1 )
			{
				PoolID->pool	= pvPortMalloc(PoolID->item_sz * PoolID->pool_sz);
			}
		}
	}
#endif	// configSUPPORT_DYNAMIC_ALLOCATION

	return PoolID;
}

/// Allocate a memory block from a memory pool.
/// \param[in]     pool_id       memory pool ID obtain referenced with \ref osPoolCreate.
/// \return address of the allocated memory block or NULL in case of no memory available.
/// \note MUST REMAIN UNCHANGED: \b osPoolAlloc shall be consistent in every CMSIS-RTOS.
void *osPoolAlloc(osPoolId pool_id)
{
	void *pptr = NULL;

#if ( configSUPPORT_DYNAMIC_ALLOCATION == 1 )
	register uint32_t i;
	register uint32_t BitMarker = 0x00000001UL;
	register uint32_t PreviousMask = 0;

	if (pool_id != NULL) {
		if (inHandlerMode()) {
			PreviousMask = portSET_INTERRUPT_MASK_FROM_ISR();
		} else {
			vPortEnterCritical();
		}

		for ( i = 0; i < pool_id->pool_sz; i++ ) {
			if ( (pool_id->flags & BitMarker) == 0 ) {
				pool_id->flags |= BitMarker;
				pptr = (void *) ((uint32_t)(pool_id->pool) + (i * pool_id->item_sz));
				break;
			}
			BitMarker <<= 1;
		}

		if (inHandlerMode()) {
			portCLEAR_INTERRUPT_MASK_FROM_ISR(PreviousMask);
		} else {
			vPortExitCritical();
		}
	}
#endif

	return pptr;
}

/// Allocate a memory block from a memory pool and set memory block to zero.
/// \param[in]     pool_id       memory pool ID obtain referenced with \ref osPoolCreate.
/// \return address of the allocated memory block or NULL in case of no memory available.
/// \note MUST REMAIN UNCHANGED: \b osPoolCAlloc shall be consistent in every CMSIS-RTOS.
void *osPoolCAlloc(osPoolId pool_id)
{
	void *pptr = NULL;

#if ( configSUPPORT_DYNAMIC_ALLOCATION == 1 )
	if (pool_id != NULL) {
		pptr = osPoolAlloc(pool_id);

		if (pptr != NULL) {
			memset(pptr, 0, pool_id->pool_sz * pool_id->item_sz);
		}
	}
#endif

	return pptr;
}

/// Return an allocated memory block back to a specific memory pool.
/// \param[in]     pool_id       memory pool ID obtain referenced with \ref osPoolCreate.
/// \param[in]     block         address of the allocated memory block that is returned to the memory pool.
/// \return status code that indicates the execution status of the function.
/// \note MUST REMAIN UNCHANGED: \b osPoolFree shall be consistent in every CMSIS-RTOS.
osStatus osPoolFree(osPoolId pool_id, void *block)
{
	osStatus ret = osErrorOS;

#if ( configSUPPORT_DYNAMIC_ALLOCATION == 1 )
	uint32_t index;
	register uint32_t BitMarker = 0x00000001UL;

	if ((pool_id == NULL) || (block == NULL)) {
		ret = osErrorParameter;
	} else if (((uint32_t)block > ((uint32_t)pool_id->pool + pool_id->pool_sz * pool_id->item_sz)) || ((uint32_t)block < (uint32_t)pool_id->pool)) {
		ret = osErrorValue;
	} else {
		index = ((uint32_t)block - (uint32_t)(pool_id->pool)) / pool_id->item_sz;
		pool_id->flags &= ~(BitMarker << index);

		ret = osOK;
	}
#endif

	return ret;
}

#endif					// osFeature_Pool



///////////////////////// Message Queue Management Functions /////////////////////////
#if (defined (osFeature_MessageQ)  &&  (osFeature_MessageQ != 0))		// osFeature_MessageQ

/// Create and Initialize a Message Queue.
/// \param[in]     queue_def     queue definition referenced with \ref osMessageQ.
/// \param[in]     thread_id     thread ID (obtained by \ref osThreadCreate or \ref osThreadGetId) or NULL.
/// \return message queue ID for reference by other functions or NULL in case of error.
/// \note MUST REMAIN UNCHANGED: \b osMessageCreate shall be consistent in every CMSIS-RTOS.
osMessageQId osMessageCreate(const osMessageQDef_t *queue_def, osThreadId thread_id)
{
	QueueHandle_t xQueueHandle = NULL;

	if (inHandlerMode()) {
		return (osMessageQId)xQueueHandle;
	}
	else {
		if (queue_def != NULL) {
#if ( configSUPPORT_STATIC_ALLOCATION == 1 )
			// QueueHandle_t xQueueCreateStatic( UBaseType_t uxQueueLength,
			//									 UBaseType_t uxItemSize,
			//									 uint8_t *pucQueueStorageBuffer,
			//									 StaticQueue_t *pxQueueBuffer );
			if ((queue_def->pool != NULL) && (queue_def->pMQB != NULL)) {
				xQueueHandle = xQueueCreateStatic(queue_def->queue_sz, queue_def->item_sz, (uint8_t *)queue_def->pool, (StaticQueue_t *)queue_def->pMQB);
			} else
#endif	// configSUPPORT_STATIC_ALLOCATION
#if ( configSUPPORT_DYNAMIC_ALLOCATION == 1 )
			{
				// 	QueueHandle_t xQueueCreate( UBaseType_t uxQueueLength,
				//								UBaseType_t uxItemSize );
				xQueueHandle = xQueueCreate(queue_def->queue_sz, queue_def->item_sz);
			}
#endif	// configSUPPORT_DYNAMIC_ALLOCATION
			;
		}
	}

	return (osMessageQId)xQueueHandle;
}

/// Put a Message to a Queue.
/// \param[in]     queue_id      message queue ID obtained with \ref osMessageCreate.
/// \param[in]     info          message information.
/// \param[in]     millisec      \ref CMSIS_RTOS_TimeOutValue or 0 in case of no time-out.
/// \return status code that indicates the execution status of the function.
/// \note MUST REMAIN UNCHANGED: \b osMessagePut shall be consistent in every CMSIS-RTOS.
osStatus osMessagePut(osMessageQId queue_id, uint32_t info, uint32_t millisec)
{
	osStatus ret = osErrorNoMemory;

	if (queue_id == NULL) {
		ret = osErrorParameter;
	} else {
		if (inHandlerMode()) {
			if (millisec) {
				ret = osErrorParameter;
			} else {
				BaseType_t xHigherPriorityTaskWoken = pdFALSE;

				// BaseType_t xQueueSendFromISR( QueueHandle_t xQueue,
				//								 const void *pvItemToQueue,
				//								 BaseType_t *pxHigherPriorityTaskWoken );
				if (xQueueSendFromISR((QueueHandle_t) queue_id, &info, &xHigherPriorityTaskWoken) == pdPASS) {
					ret = osOK;
				} else {
					ret = osErrorResource;
				}
				if(xHigherPriorityTaskWoken == pdTRUE) {
					portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
				}
			}
		} else {
			// BaseType_t xQueueSend( QueueHandle_t xQueue,
			//						  const void * pvItemToQueue,
			//						  TickType_t xTicksToWait );
			if (xQueueSend((QueueHandle_t) queue_id, &info, pdMS_TO_TICKS(millisec)) == pdPASS) {
				ret = osOK;
			} else {
				if (millisec) {
					ret = osErrorTimeoutResource;
				} else {
					ret = osErrorResource;
				}
			}
		}
	}

	return ret;
}

/// Get a Message or Wait for a Message from a Queue.
/// \param[in]     queue_id      message queue ID obtained with \ref osMessageCreate.
/// \param[in]     millisec      \ref CMSIS_RTOS_TimeOutValue or 0 in case of no time-out.
/// \return event information that includes status code.
/// \note MUST REMAIN UNCHANGED: \b osMessageGet shall be consistent in every CMSIS-RTOS.
osEvent osMessageGet(osMessageQId queue_id, uint32_t millisec)
{
	osEvent event;

	event.status =  osErrorOS;

	if (queue_id == NULL) {
		event.status =  osErrorParameter;
	} else {
		if (inHandlerMode()) {
			if (millisec) {
				event.status = osErrorParameter;
			} else {
				BaseType_t xHigherPriorityTaskWoken = pdFALSE;

				// BaseType_t xQueueReceiveFromISR( QueueHandle_t xQueue,
				//									void *pvBuffer,
				//									BaseType_t *pxHigherPriorityTaskWoken );
				if (xQueueReceiveFromISR((QueueHandle_t) queue_id, &event.value.v, &xHigherPriorityTaskWoken) == pdPASS) {
					event.status = osEventMessage;
				} else {
					event.status = osOK;
				}
				if(xHigherPriorityTaskWoken == pdTRUE) {
					portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
				}
			}
		} else {
			// BaseType_t xQueueReceive( QueueHandle_t xQueue,
			//							 void *pvBuffer,
			//							 TickType_t xTicksToWait );
			if (xQueueReceive((QueueHandle_t) queue_id, &event.value.v, pdMS_TO_TICKS(millisec)) == pdPASS) {
				event.status = osEventMessage;
			} else {
				if (millisec) {
					event.status = osEventTimeout;
				} else {
					event.status = osOK;
				}
			}
		}
	}

	return event;
}

#endif					// osFeature_MessageQ



///////////////////////// Mail Queue Management Functions /////////////////////////

#if (defined (osFeature_MailQ)  &&  (osFeature_MailQ != 0))     // Mail Queues available

typedef struct os_mailQ_cb {
	uint32_t 		queue_sz;		///< number of elements in the queue
	uint32_t		item_sz;	 	///< size of an item
	osPoolId		PoolID;
	QueueHandle_t	QueueID;
} os_mailQ_cb_t;

/// Create and Initialize mail queue.
/// \param[in]     queue_def     reference to the mail queue definition obtain with \ref osMailQ
/// \param[in]     thread_id     thread ID (obtained by \ref osThreadCreate or \ref osThreadGetId) or NULL.
/// \return mail queue ID for reference by other functions or NULL in case of error.
/// \note MUST REMAIN UNCHANGED: \b osMailCreate shall be consistent in every CMSIS-RTOS.
osMailQId osMailCreate(const osMailQDef_t *queue_def, osThreadId thread_id)
{
	osMailQId MailID = NULL;

	if (inHandlerMode()) {
		return MailID;
	}
#if ( configSUPPORT_DYNAMIC_ALLOCATION == 1 )
	else {
		osPoolDef_t pool_def = { queue_def->queue_sz, queue_def->item_sz };

		if (queue_def == NULL) {
			return MailID;
		} else if (queue_def->queue_sz > osFeatureMaxPoolNumber) {
			return MailID;
		} else if ( xPortGetFreeHeapSize() < (sizeof (os_mailQ_cb_t) + sizeof(os_pool_cb_t) + (queue_def->queue_sz * sizeof(void *)) + (queue_def->queue_sz * queue_def->item_sz) + 8)) {
			return MailID;
		} else {
			MailID = pvPortMalloc(sizeof(os_mailQ_cb_t));

			MailID->queue_sz = queue_def->queue_sz;
			MailID->item_sz  = queue_def->item_sz;

			// QueueHandle_t xQueueCreate( UBaseType_t uxQueueLength,
			//								UBaseType_t uxItemSize );
			MailID->QueueID = xQueueCreate(queue_def->queue_sz, sizeof(void *));

	#if ( configSUPPORT_STATIC_ALLOCATION == 1 )
			if (queue_def->poolid != NULL) {
				MailID->PoolID	 = queue_def->poolid;
			} else
	#endif	// configSUPPORT_STATIC_ALLOCATION == 1
			{
				MailID->PoolID = osPoolCreate(&pool_def);
			}
		}
	}
#endif	// configSUPPORT_DYNAMIC_ALLOCATION == 1

	return MailID;
}

/// Allocate a memory block from a mail.
/// \param[in]     queue_id      mail queue ID obtained with \ref osMailCreate.
/// \param[in]     millisec      \ref CMSIS_RTOS_TimeOutValue or 0 in case of no time-out
/// \return pointer to memory block that can be filled with mail or NULL in case of error.
/// \note MUST REMAIN UNCHANGED: \b osMailAlloc shall be consistent in every CMSIS-RTOS.
void *osMailAlloc(osMailQId queue_id, uint32_t millisec)
{
	void *mptr = NULL;

#if ( configSUPPORT_DYNAMIC_ALLOCATION == 1 )
	if (queue_id == NULL) {
		return mptr;
	} else {
		if (inHandlerMode()) {
			if (millisec != 0) {
				return mptr;
			}
		}

		mptr = osPoolAlloc(queue_id->PoolID);
	}
#endif	// configSUPPORT_DYNAMIC_ALLOCATION == 1

	return mptr;
}


/// Allocate a memory block from a mail and set memory block to zero.
/// \param[in]     queue_id      mail queue ID obtained with \ref osMailCreate.
/// \param[in]     millisec      \ref CMSIS_RTOS_TimeOutValue or 0 in case of no time-out
/// \return pointer to memory block that can be filled with mail or NULL in case of error.
/// \note MUST REMAIN UNCHANGED: \b osMailCAlloc shall be consistent in every CMSIS-RTOS.
void *osMailCAlloc (osMailQId queue_id, uint32_t millisec)
{
	void *mptr = NULL;

#if ( configSUPPORT_DYNAMIC_ALLOCATION == 1 )
	if (queue_id == NULL) {
		return mptr;
	} else {
		if (inHandlerMode()) {
			if (millisec != 0) {
				return mptr;
			}
		}

		mptr = osPoolCAlloc(queue_id->PoolID);
	}
#endif	// configSUPPORT_DYNAMIC_ALLOCATION == 1

	return mptr;
}

/// Put a mail to a queue.
/// \param[in]     queue_id      mail queue ID obtained with \ref osMailCreate.
/// \param[in]     mail          memory block previously allocated with \ref osMailAlloc or \ref osMailCAlloc.
/// \return status code that indicates the execution status of the function.
/// \note MUST REMAIN UNCHANGED: \b osMailPut shall be consistent in every CMSIS-RTOS.
osStatus osMailPut(osMailQId queue_id, void *mail)
{
	osStatus ret = osErrorOS;

#if ( configSUPPORT_DYNAMIC_ALLOCATION == 1 )
	if (queue_id == NULL) {
		ret = osErrorParameter;
	} else if (mail == NULL) {
		ret = osErrorValue;
	} else if (((uint32_t)mail > ((uint32_t)queue_id->PoolID->pool + queue_id->PoolID->pool_sz * queue_id->PoolID->item_sz)) || ((uint32_t)mail < (uint32_t)queue_id->PoolID->pool)) {
		ret = osErrorValue;
	} else {
		if (inHandlerMode()) {
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;

			// BaseType_t xQueueSendFromISR( QueueHandle_t xQueue,
			//								 const void *pvItemToQueue,
			//								 BaseType_t *pxHigherPriorityTaskWoken );
			if (xQueueSendFromISR((QueueHandle_t) queue_id->QueueID, &mail, &xHigherPriorityTaskWoken) == pdPASS) {
				ret = osOK;
			}
			if(xHigherPriorityTaskWoken == pdTRUE) {
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
			}
		} else {
			// BaseType_t xQueueSend( QueueHandle_t xQueue,
			//						  const void * pvItemToQueue,
			//						  TickType_t xTicksToWait );
			if (xQueueSend((QueueHandle_t) queue_id->QueueID, &mail, pdMS_TO_TICKS(osProcessDelayDeadTick)) == pdPASS) {
				ret = osOK;
			}
		}
	}
#endif	// configSUPPORT_DYNAMIC_ALLOCATION == 1

	return ret;
}

/// Get a mail from a queue.
/// \param[in]     queue_id      mail queue ID obtained with \ref osMailCreate.
/// \param[in]     millisec      \ref CMSIS_RTOS_TimeOutValue or 0 in case of no time-out
/// \return event that contains mail information or error code.
/// \note MUST REMAIN UNCHANGED: \b osMailGet shall be consistent in every CMSIS-RTOS.
osEvent osMailGet(osMailQId queue_id, uint32_t millisec)
{
	osEvent event;

	event.status =  osErrorOS;

#if ( configSUPPORT_DYNAMIC_ALLOCATION == 1 )
	if (queue_id == NULL) {
		event.status = osErrorParameter;
	} else {
		if (inHandlerMode()) {
			if (millisec) {
				event.status = osErrorParameter;
			} else {
				BaseType_t xHigherPriorityTaskWoken = pdFALSE;

				// BaseType_t xQueueReceiveFromISR( QueueHandle_t xQueue,
				//									void *pvBuffer,
				//									BaseType_t *pxHigherPriorityTaskWoken );
				if (xQueueReceiveFromISR((QueueHandle_t) queue_id->QueueID, &event.value.p, &xHigherPriorityTaskWoken) == pdPASS) {
					event.status = osEventMail;
				} else {
					event.status = osOK;
				}
				if(xHigherPriorityTaskWoken == pdTRUE) {
					portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
				}
			}
		} else {
			// BaseType_t xQueueReceive( QueueHandle_t xQueue,
			//							 void *pvBuffer,
			//							 TickType_t xTicksToWait );
			if (xQueueReceive((QueueHandle_t) queue_id->QueueID, &event.value.p, pdMS_TO_TICKS(millisec)) == pdPASS) {
				event.status = osEventMail;
			} else {
				if (millisec) {
					event.status = osEventTimeout;
				} else {
					event.status = osOK;
				}
			}
		}
	}
#endif	// configSUPPORT_DYNAMIC_ALLOCATION == 1

	return event;
}
/// Free a memory block from a mail.
/// \param[in]     queue_id      mail queue ID obtained with \ref osMailCreate.
/// \param[in]     mail          pointer to the memory block that was obtained with \ref osMailGet.
/// \return status code that indicates the execution status of the function.
/// \note MUST REMAIN UNCHANGED: \b osMailFree shall be consistent in every CMSIS-RTOS.
osStatus osMailFree(osMailQId queue_id, void *mail)
{
	osStatus ret = osErrorOS;

#if ( configSUPPORT_DYNAMIC_ALLOCATION == 1 )
	if (queue_id == NULL) {
		ret = osErrorParameter;
	} else if (mail == NULL) {
		ret = osErrorValue;
	} else if (((uint32_t)mail > ((uint32_t)queue_id->PoolID->pool + queue_id->PoolID->pool_sz * queue_id->PoolID->item_sz)) || ((uint32_t)mail < (uint32_t)queue_id->PoolID->pool)) {
		ret = osErrorValue;
	} else {
		ret = osPoolFree(queue_id->PoolID, mail);
	}
#endif	// configSUPPORT_DYNAMIC_ALLOCATION == 1

	return ret;
}

#endif  // Mail Queues available


///////////////////////// Other Functions /////////////////////////
uint8_t	osNVICInterruptSetPriority(uint8_t InterruptNumber, osInterruptPriority InterruptPriority)
{
	// Valid check
	if (InterruptNumber > configNumberOfUserInterrutp) {
		return 0;
	}

	NVIC_SetPriority(InterruptNumber, InterruptPriority);

	return InterruptPriority;
}



