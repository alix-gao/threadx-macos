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

/* Include necessary system files. */
#include <stdio.h>
#include <unistd.h>
#include <assert.h>
#include <fcntl.h>

#include "tx_api.h"
#include "tx_thread.h"

/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _tx_thread_stack_build                              macos/GNU       */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    cheng.gao, thoughtworks Corporation                                 */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function builds a stack frame on the supplied thread's stack.  */
/*    The stack frame results in a fake interrupt return to the supplied  */
/*    function pointer.                                                   */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  08-07-2022        cheng.gao                Initial Version 6.1        */
/*                                                                        */
/**************************************************************************/
static void *_tx_macos_thread_entry(void *ptr)
{
    TX_THREAD *thread_ptr;

    assert(NULL != ptr);

    /* Pickup the current thread pointer. */
    thread_ptr = (TX_THREAD *) ptr;
    nice(20);

    info("%s %x entry\n", thread_ptr->tx_thread_name, pthread_self());

    /* thread entry may start before pthread_create is returned.
       so thread id maybe 0, here wait for thread id */
    while (0 == thread_ptr->tx_macos_thread_id) {
        info("%s entry waits for thread id", thread_ptr->tx_thread_name);
        sleep(1);
    }

    /* Now suspend the thread initially.
       If the thread has already been scheduled, this will return immediately. */
    _tx_macos_thread_suspend(thread_ptr);
    /* Setup the thread suspension type to solicited thread suspension.
       Pseudo interrupt handlers will suspend with this field set to 1. */
    thread_ptr->tx_thread_init_done = 1;
    info("%s entry go\n", thread_ptr->tx_thread_name);
    /* Call ThreadX thread entry point. */
    _tx_thread_shell_entry();

    return EXIT_SUCCESS;
}

VOID _tx_thread_stack_build(TX_THREAD *thread_ptr, VOID (*function_ptr)(VOID))
{
    struct sched_param sp;

    assert(NULL != function_ptr);

    /* Create the run semaphore for the thread.
       This will allow the scheduler control over when the thread actually runs. */
    sem_unlink(thread_ptr->tx_thread_name);
    thread_ptr->tx_thread_macos_thread_run_semaphore = sem_open(thread_ptr->tx_thread_name, O_CREAT, 0666, 0);
    if (SEM_FAILED == thread_ptr->tx_thread_macos_thread_run_semaphore) {
        /* Display an error message. */
        info("ThreadX macos error creating thread running semaphore! %s, \n", thread_ptr->tx_thread_name);
        dead();
    }

    /* Setup the thread suspension type to solicited thread suspension.
       Pseudo interrupt handlers will suspend with this field set to 1. */
    thread_ptr->tx_thread_init_done = 0;

    /* Create a macos thread for the application thread. */
    if (pthread_create(&thread_ptr->tx_macos_thread_id, NULL, _tx_macos_thread_entry, thread_ptr)) {
        /* Display an error message. */
        info("ThreadX macos error creating thread!\n");
        dead();
    }
    info("create id %lx", thread_ptr->tx_macos_thread_id);

    /* Otherwise, we have a good thread create. */
    sp.sched_priority = TX_MACOS_PRIORITY_USER_THREAD;
    pthread_setschedparam(thread_ptr->tx_macos_thread_id, SCHED_FIFO, &sp);

    /* Clear the disabled count that will keep track of the tx_interrupt_control nesting. */
    thread_ptr->tx_macos_thread_int_flag = TX_INT_ENABLE;

    /* Setup a fake thread stack pointer. */
    thread_ptr->tx_thread_stack_ptr = (VOID *)(((CHAR *) thread_ptr->tx_thread_stack_end) - 8);

    /* Clear the first word of the stack. */
    *(((ULONG *) thread_ptr->tx_thread_stack_ptr) - 1) =  0;
}
