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
#include <stdio.h>
#include <unistd.h>


/* Prototype for new thread entry function.  */

void *_tx_linux_thread_entry(void *ptr);


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _tx_thread_stack_build                              Linux/GNU       */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    William E. Lamie, Microsoft Corporation                             */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function builds a stack frame on the supplied thread's stack.  */
/*    The stack frame results in a fake interrupt return to the supplied  */
/*    function pointer.                                                   */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    thread_ptr                            Pointer to thread control blk */
/*    function_ptr                          Pointer to return function    */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    pthread_create                                                      */
/*    pthread_setschedparam                                               */
/*    _tx_linux_thread_suspend                                            */
/*    sem_open                                                            */
/*    info                                                              */
/*    _tx_linux_thread_resume                                             */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    _tx_thread_create                     Create thread service         */
/*    _tx_thread_reset                      Reset thread service          */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  09-30-2020     William E. Lamie         Initial Version 6.1           */
/*                                                                        */
/**************************************************************************/
VOID   _tx_thread_stack_build(TX_THREAD *thread_ptr, VOID (*function_ptr)(VOID))
{
struct sched_param sp;

    (VOID)function_ptr;

    /* Create the run semaphore for the thread.  This will allow the scheduler
       control over when the thread actually runs.  */
    sem_unlink(thread_ptr->tx_thread_name);
    thread_ptr->tx_thread_linux_thread_run_semaphore = sem_open(thread_ptr->tx_thread_name, O_CREAT, 0666, 0);
    if(SEM_FAILED == thread_ptr -> tx_thread_linux_thread_run_semaphore)
    {

        /* Display an error message.  */
        info("ThreadX Linux error creating thread running semaphore! %s, \n", thread_ptr->tx_thread_name);
        while(1)
        {
        }
    }

    /* Create a Linux thread for the application thread.  */
    if(pthread_create(&thread_ptr -> tx_thread_linux_thread_id, NULL, _tx_linux_thread_entry, thread_ptr))
    {

        /* Display an error message.  */
        info("ThreadX Linux error creating thread!\n");
        while(1)
        {
        }
    }
    thread_ptr->tx_macos_thread_suspend = 0;
info("create id %lx", thread_ptr -> tx_thread_linux_thread_id);
    /* Otherwise, we have a good thread create.  */
    sp.sched_priority = TX_LINUX_PRIORITY_USER_THREAD;
    pthread_setschedparam(thread_ptr -> tx_thread_linux_thread_id, SCHED_FIFO, &sp);

    /* Setup the thread suspension type to solicited thread suspension.
       Pseudo interrupt handlers will suspend with this field set to 1.  */
    thread_ptr -> tx_thread_linux_suspension_type =  0;

    /* Clear the disabled count that will keep track of the
       tx_interrupt_control nesting.  */
    thread_ptr -> tx_thread_linux_int_disabled_flag =  0;

    /* Setup a fake thread stack pointer.   */
    thread_ptr -> tx_thread_stack_ptr =  (VOID *) (((CHAR *) thread_ptr -> tx_thread_stack_end) - 8);

    /* Clear the first word of the stack.  */
    *(((ULONG *) thread_ptr -> tx_thread_stack_ptr) - 1) =  0;
}


void *_tx_linux_thread_entry(void *ptr)
{

TX_THREAD  *thread_ptr;

    /* Pickup the current thread pointer.  */
    thread_ptr =  (TX_THREAD *) ptr;
    _tx_linux_threadx_thread = 1;
    nice(20);
info("%s %lx entry\n", thread_ptr->tx_thread_name, pthread_self());
    /* Now suspend the thread initially.  If the thread has already
       been scheduled, this will return immediately.  */
    //tx_linux_sem_wait(thread_ptr -> tx_thread_linux_thread_run_semaphore);
    thread_ptr->tx_macos_thread_suspend = 0;
    _tx_linux_thread_suspend(thread_ptr);
    //tx_linux_sem_post_nolock(_tx_sch_start_semaphore);
info("%s entry go\n", thread_ptr->tx_thread_name);
    /* Call ThreadX thread entry point.  */
    _tx_thread_shell_entry();

    return EXIT_SUCCESS;
}

