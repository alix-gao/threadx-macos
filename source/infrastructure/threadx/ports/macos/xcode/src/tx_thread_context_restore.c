/**************************************************************************/
/*                                                                        */
/*       Copyright (c) Microsoft Corporation. All rights reserved.        */
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


#define TX_SOURCE_CODE


/* Include necessary system files.  */

#include "tx_api.h"
#include "tx_thread.h"
#include "tx_timer.h"

extern sem_t *_tx_linux_isr_semaphore;
UINT _tx_linux_timer_waiting = 0;
/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _tx_thread_context_restore                          Linux/GNU       */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    William E. Lamie, Microsoft Corporation                             */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function restores the interrupt context if it is processing a  */
/*    nested interrupt.  If not, it returns to the interrupt thread if no */
/*    preemption is necessary.  Otherwise, if preemption is necessary or  */
/*    if no thread was running, the function returns to the scheduler.    */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    tx_macos_mutex_lock                                                 */
/*    sem_trywait                                                         */
/*    tx_linux_sem_post                                                   */
/*    tx_macos_sem_wait                                                   */
/*    _tx_linux_thread_resume                                             */
/*    tx_linux_mutex_recursive_unlock                                     */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    ISRs                                  Interrupt Service Routines    */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  09-30-2020     William E. Lamie         Initial Version 6.1           */
/*                                                                        */
/**************************************************************************/
VOID   _tx_thread_context_restore(VOID)
{
    /* Lock mutex to ensure other threads are not playing with
       the core ThreadX data structures.  */
    tx_macos_mutex_lock(_tx_macos_mutex);

    /* Determine if interrupts are nested. */
    if ((!--_tx_thread_system_state) && (_tx_thread_current_ptr)) {
        /* Interrupts are nested.  */
        /* Just recover the saved registers and return to the point of interrupt.  */
        info("interrupt are nested");
        if ((_tx_thread_current_ptr == _tx_thread_execute_ptr)
         || (_tx_thread_preempt_disable)) {
            /* Determine if a thread was interrupted and no preemption is required.  */
            /* Restore interrupted thread or ISR.  */
            info("interrupt restore");

            /* Pickup the saved stack pointer.  */
            /* Recover the saved context and return to the point of interrupt.  */
            _tx_linux_thread_resume(_tx_thread_current_ptr);
        } else {
            _tx_linux_thread_suspend(_tx_thread_current_ptr);

            /* Save the remaining time-slice and disable it.  */
            if (_tx_timer_time_slice) {
                _tx_thread_current_ptr->tx_thread_time_slice =  _tx_timer_time_slice;
                _tx_timer_time_slice =  0;
            }

            /* Clear the current task pointer.  */
            _tx_thread_current_ptr =  TX_NULL;

        }
    }

     /* Just return back to the scheduler!  */


    /* Unlock linux mutex. */
    tx_linux_mutex_recursive_unlock(_tx_macos_mutex);
}

