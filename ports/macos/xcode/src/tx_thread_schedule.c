/**************************************************************************/
/*                                                                        */
/*       Copyright (c) thoughtworks Corporation. All rights reserved.     */
/*                                                                        */
/*       This software is licensed under the Microsoft Software License   */
/*       Terms for Microsoft Azure RTOS. Full text of the license can be  */
/*       found in the LICENSE file at https://aka.ms/AzureRTOS_EULA       */
/*       and in the root directory of this software.                      */
/*                                                                        */
/**************************************************************************/

#define TX_SOURCE_CODE

/* Include necessary system files. */
#include <stdio.h>
#include <errno.h>
#include <pthread.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdbool.h>
#include <assert.h>

#include "tx_api.h"
#include "tx_thread.h"
#include "tx_timer.h"

/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _tx_thread_schedule                                 macos/xcode     */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    cheng.gao, thoughtworks Corporation                                 */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function waits for a thread control block pointer to appear in */
/*    the _tx_thread_execute_ptr variable.  Once a thread pointer appears */
/*    in the variable, the corresponding thread is resumed.               */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  08-07-2022        cheng.gao                Initial Version 6.1        */
/*                                                                        */
/**************************************************************************/
VOID _tx_thread_schedule(VOID)
{
    TX_INTERRUPT_SAVE_AREA
    TX_THREAD *thread_ptr;

    while (true) {
        info("_tx_thread_schedule\n");

        tx_macos_mutex_lock(_tx_macos_mutex);

        pthread_cond_wait(&_tx_macos_schedule_cond, &_tx_macos_mutex);

        TX_DISABLE
        thread_ptr = _tx_thread_execute_ptr;
        TX_RESTORE

        if (NULL != thread_ptr) {
            /* Setup the current thread pointer. */
            _tx_thread_current_ptr = thread_ptr;

            info("schedule result %s\n", _tx_thread_current_ptr->tx_thread_name);

            /* Increment the run count for this thread. */
            _tx_thread_current_ptr->tx_thread_run_count++;

            /* Setup time-slice, if present. */
            _tx_timer_time_slice = _tx_thread_current_ptr->tx_thread_time_slice;

            _tx_macos_thread_resume(_tx_thread_current_ptr);
        }

        /* Unlock macos mutex. */
        tx_macos_mutex_unlock(_tx_macos_mutex);
    }
}

void _tx_thread_delete_port_completion(TX_THREAD *thread_ptr, UINT tx_saved_posture)
{
    INT macos_status;
    sem_t *threadrunsemaphore;
    pthread_t thread_id;
    struct timespec ts;

    info("_tx_thread_delete_port_completion");

    thread_id = thread_ptr->tx_macos_thread_id;
    threadrunsemaphore = thread_ptr->tx_thread_macos_thread_run_semaphore;
    ts.tv_sec = 0;
    ts.tv_nsec = 1000000;
    TX_RESTORE
    do {
        macos_status = pthread_cancel(thread_id);
        if (macos_status != EAGAIN) {
            break;
        }
        _tx_macos_thread_resume(thread_ptr);
        tx_macos_sem_post(threadrunsemaphore);
        nanosleep(&ts, &ts);
    } while (1);
    pthread_join(thread_id, NULL);
    sem_close(threadrunsemaphore);
    sem_unlink(thread_ptr->tx_thread_name);
    TX_DISABLE
}

void _tx_thread_reset_port_completion(TX_THREAD *thread_ptr, UINT tx_saved_posture)
{
    INT macos_status;
    sem_t *threadrunsemaphore;
    pthread_t thread_id;
    struct timespec ts;

    info("_tx_thread_reset_port_completion");

    thread_id = thread_ptr->tx_macos_thread_id;
    threadrunsemaphore = thread_ptr->tx_thread_macos_thread_run_semaphore;
    ts.tv_sec = 0;
    ts.tv_nsec = 1000000;
    TX_RESTORE
    do {
        macos_status = pthread_cancel(thread_id);
        if (macos_status != EAGAIN) {
            break;
        }
        _tx_macos_thread_resume(thread_ptr);
        tx_macos_sem_post(threadrunsemaphore);
        nanosleep(&ts, &ts);
    } while (1);
    pthread_join(thread_id, NULL);
    sem_close(threadrunsemaphore);
    sem_unlink(thread_ptr->tx_thread_name);
    TX_DISABLE
}
