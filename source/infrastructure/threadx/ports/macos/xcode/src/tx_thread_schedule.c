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


#define TX_SOURCE_CODE


/* Include necessary system files.  */
#include <stdio.h>
#include <errno.h>
#include <pthread.h>
#include <sys/types.h>
#include <unistd.h>

#include "tx_api.h"
#include "tx_thread.h"
#include "tx_timer.h"

/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _tx_thread_schedule                                 Linux/GNU       */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    William E. Lamie, Microsoft Corporation                             */
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
/*  CALLS                                                                 */
/*                                                                        */
/*    tx_linux_mutex_lock                                                 */
/*    tx_linux_mutex_unlock                                               */
/*    _tx_linux_thread_resume                                             */
/*    tx_linux_sem_post                                                   */
/*    sem_trywait                                                         */
/*    tx_linux_sem_wait                                                   */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    _tx_initialize_kernel_enter          ThreadX entry function         */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  09-30-2020     William E. Lamie         Initial Version 6.1           */
/*                                                                        */
/**************************************************************************/
VOID   _tx_thread_schedule(VOID)
{
struct timespec ts;
int back;

    /* Set timer. */
    ts.tv_sec = 0;
    ts.tv_nsec = 200000;

    /* Loop forever.  */
    while (1) {
info("_tx_thread_schedule\n");
        tx_linux_sem_wait(_tx_schedule_semaphore);

        //while (!_tx_thread_execute_ptr || pthread_main_np() || !pthread_equal(_tx_thread_execute_ptr->tx_thread_linux_thread_id, pthread_self()));
        info("sche");

        if ((NULL == _tx_thread_execute_ptr)
         || (!pthread_main_np() && !pthread_equal(_tx_thread_execute_ptr->tx_thread_linux_thread_id, pthread_self()))) {
            continue;
        }


        tx_linux_mutex_lock(_tx_macos_mutex);

        back = 0;
        if (_tx_thread_execute_ptr) {

            /* Yes! We have a thread to execute. Note that the critical section is already
            active from the scheduling loop above.  */

            /* Setup the current thread pointer.  */
            _tx_thread_current_ptr =  _tx_thread_execute_ptr;
    info("_tx_thread_schedule :::::::::::::::::::::::: %s\n", _tx_thread_current_ptr->tx_thread_name);
            /* Increment the run count for this thread.  */
            _tx_thread_current_ptr -> tx_thread_run_count++;

            /* Setup time-slice, if present.  */
            _tx_timer_time_slice =  _tx_thread_current_ptr -> tx_thread_time_slice;

            back = pthread_equal(_tx_thread_current_ptr->tx_thread_linux_thread_id, pthread_self());
        }
        /* Unlock linux mutex. */
        tx_linux_mutex_unlock(_tx_macos_mutex);

    tx_linux_sem_post_nolock(_tx_sch_end_semaphore);

#if 1
        if (!back) {
            continue;
        }
#endif
        if (pthread_main_np()) {
            info("the world of dio");
            sleep(-1);
        }

        info("_tx_thread_schedule s4\n");
        break;

    }
}

void _tx_thread_delete_port_completion(TX_THREAD *thread_ptr, UINT tx_saved_posture)
{
INT             linux_status;
sem_t           *threadrunsemaphore;
pthread_t       thread_id;
struct          timespec ts;

    thread_id = thread_ptr -> tx_thread_linux_thread_id;
    threadrunsemaphore = thread_ptr -> tx_thread_linux_thread_run_semaphore;
    ts.tv_sec = 0;
    ts.tv_nsec = 1000000;
    TX_RESTORE
    do
    {
        linux_status = pthread_cancel(thread_id);
        if(linux_status != EAGAIN)
        {
            break;
        }
        _tx_linux_thread_resume(thread_ptr);
        tx_linux_sem_post(threadrunsemaphore);
        nanosleep(&ts, &ts);
    } while (1);
    pthread_join(thread_id, NULL);
    sem_close(threadrunsemaphore);
    sem_unlink(thread_ptr->tx_thread_name);
    TX_DISABLE
}

void _tx_thread_reset_port_completion(TX_THREAD *thread_ptr, UINT tx_saved_posture)
{
INT             linux_status;
sem_t           *threadrunsemaphore;
pthread_t       thread_id;
struct          timespec ts;

    thread_id = thread_ptr -> tx_thread_linux_thread_id;
    threadrunsemaphore = thread_ptr -> tx_thread_linux_thread_run_semaphore;
    ts.tv_sec = 0;
    ts.tv_nsec = 1000000;
    TX_RESTORE
    do
    {
        linux_status = pthread_cancel(thread_id);
        if(linux_status != EAGAIN)
        {
            break;
        }
        _tx_linux_thread_resume(thread_ptr);
        tx_linux_sem_post(threadrunsemaphore);
        nanosleep(&ts, &ts);
    } while (1);
    pthread_join(thread_id, NULL);
    sem_close(threadrunsemaphore);
    sem_unlink(thread_ptr->tx_thread_name);
    TX_DISABLE
}
