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

/**************************************************************************/
/**************************************************************************/
/**                                                                       */
/** ThreadX Component                                                     */
/**                                                                       */
/**   Thread                                                              */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define    TX_SOURCE_CODE

/* Include necessary system files. */
#include <stdio.h>
#include <assert.h>
#include <stdbool.h>

#include "tx_api.h"
#include "tx_thread.h"
#include "tx_timer.h"

/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _tx_thread_system_return                            Linux/GNU       */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    cheng.gao, thoughtworks Corporation                                 */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function is target processor specific.  It is used to transfer */
/*    control from a thread back to the system.  Only a minimal context   */
/*    is saved since the compiler assumes temp registers are going to get */
/*    slicked by a function call anyway.                                  */
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
VOID _tx_thread_system_return(VOID)
{
    TX_THREAD *temp_thread_ptr;
    sem_t *temp_run_semaphore;
    UINT temp_thread_state;
    pthread_t thread_id;
    int exit_code = 0;

    info(":::::::::::::::::::::::::::::return %s r1\n", _tx_thread_current_ptr->tx_thread_name);

    /* Lock Linux mutex. */
    tx_macos_mutex_lock(_tx_macos_mutex);

    /* First, determine if the thread was terminated. */

    /* Pickup the id of the current thread. */
    thread_id = pthread_self();

    /* Pickup the current thread pointer. */
    temp_thread_ptr = _tx_thread_current_ptr;

    /* Determine if this is a thread (0) and it does not match the current thread pointer. */
    if ((_tx_linux_threadx_thread) &&
        ((!temp_thread_ptr) || (!pthread_equal(temp_thread_ptr->tx_macos_thread_id, thread_id)))) {
        /* This indicates the Linux thread was actually terminated by ThreadX is only being allowed to run in order to cleanup its resources. */
        tx_linux_mutex_recursive_unlock(_tx_macos_mutex);
        printf("pthread exit %p\n", temp_thread_ptr);
        dump_callstack();
        assert(false);
        pthread_exit((void *) &exit_code);
        exit(0);
    }

    /* Determine if the time-slice is active. */
    if (_tx_timer_time_slice) {
        /* Preserve current remaining time-slice for the thread and clear the current time-slice. */
        temp_thread_ptr->tx_thread_time_slice = _tx_timer_time_slice;
        _tx_timer_time_slice = 0;
    }

    /* Save the run semaphore into a temporary variable as well. */
    temp_run_semaphore = temp_thread_ptr->tx_thread_linux_thread_run_semaphore;

    /* Pickup the current thread state. */
    temp_thread_state = temp_thread_ptr->tx_thread_state;

    /* Setup the suspension type for this thread. */
    temp_thread_ptr->tx_thread_linux_suspension_type = 0;

    /* Set the current thread pointer to NULL. */
    _tx_thread_current_ptr = TX_NULL;

    /* Unlock Linux mutex. */
    tx_linux_mutex_recursive_unlock(_tx_macos_mutex);

    _tx_thread_schedule();
}

