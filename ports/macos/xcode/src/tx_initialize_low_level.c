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
/**   Initialize                                                          */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define TX_SOURCE_CODE

/* Include necessary system files. */
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <errno.h>
#include <stdbool.h>
#include <assert.h>
#include <fcntl.h>
#include <sys/time.h>

#include "tx_api.h"

/* Define various macos objects used by the ThreadX port. */
pthread_mutex_t _tx_macos_mutex;
sem_t *_tx_schedule_semaphore;
pthread_cond_t _tx_macos_schedule_cond;

/* trace */
struct timespec _tx_macos_time_stamp;

/* Define simulated timer interrupt.  This is done inside a thread, which is
   how other interrupts may be defined as well.  See code below for an
   example.  */

static pthread_t _tx_macos_timer_id;
static pthread_cond_t _tx_macos_timer_cond;
static pthread_mutex_t _tx_macos_timer_mutex;

/* Define other external variable references. */
extern VOID *_tx_initialize_unused_memory;

/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _tx_initialize_low_level                            macos/GNU       */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    cheng.gao, thoughtworks Corporation                                 */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function is responsible for any low-level processor            */
/*    initialization, including setting up interrupt vectors, setting     */
/*    up a periodic timer interrupt source, saving the system stack       */
/*    pointer for use in ISR processing later, and finding the first      */
/*    available RAM memory address for tx_application_define.             */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  08-07-2022        cheng.gao                Initial Version 6.1        */
/*                                                                        */
/**************************************************************************/

/* Define signals for macos thread. */
#define SUSPEND_SIG SIGUSR1
#define RESUME_SIG SIGUSR2

static sigset_t _tx_macos_thread_wait_mask;

/* signal cannot pass parameters,
   use __thread to instead of thread info, bool variable does not need to be protected */
static __thread bool _tx_macos_thread_suspended = false;

/* Define functions for macos thread. */
static void _tx_macos_thread_resume_handler(int sig)
{
    (void) sig;
}

static void _tx_macos_thread_suspend_handler(int sig)
{
    (void) sig;

    if (!_tx_macos_thread_suspended) {
        _tx_macos_thread_suspended = true;
        info("suspend handler %lx", pthread_self());
        sigsuspend(&_tx_macos_thread_wait_mask);
        _tx_macos_thread_suspended = false;
    }
}

void _tx_macos_thread_suspend(TX_THREAD *thread)
{
    sigset_t set;
    pthread_t thread_id = thread->tx_macos_thread_id;

    assert(NULL != thread);

    if (_tx_macos_thread_suspended) {
        info("curr thread has been suspended");
        return;
    }

    if (0 == thread_id) {
        dump_callstack();
    }

    if (pthread_kill(thread_id, 0)) {
        info("thread %lx %s not exist", thread_id, thread->tx_thread_name);
        return;
    }

    sigpending(&set);
    if (sigismember(&set, SUSPEND_SIG)) {
        info("thread %s has been suspend already", thread->tx_thread_name);
        return;
    }

    info("suspend %lx %s", thread->tx_macos_thread_id, thread->tx_thread_name);

    /* Send signal. */
    //tx_macos_mutex_lock(_tx_macos_mutex);
    pthread_kill(thread_id, SUSPEND_SIG);
    //tx_macos_mutex_unlock(_tx_macos_mutex);
    info("suspend signal issued");
}

void _tx_macos_thread_resume(TX_THREAD *thread)
{
    assert(NULL != thread);

    info("resume %lx %s", thread->tx_macos_thread_id, thread->tx_thread_name);

    /* Send signal. */
    //tx_macos_mutex_lock(_tx_macos_mutex);
    pthread_kill(thread->tx_macos_thread_id, RESUME_SIG);
    //tx_macos_mutex_unlock(_tx_macos_mutex);
}

static void _tx_macos_thread_init(void)
{
    struct sigaction sa;

    sigfillset(&_tx_macos_thread_wait_mask);
    sigdelset(&_tx_macos_thread_wait_mask, RESUME_SIG);

    sigfillset(&sa.sa_mask);
    sa.sa_flags = 0;
    sa.sa_handler = _tx_macos_thread_resume_handler;
    sigaction(RESUME_SIG, &sa, NULL);

    sa.sa_handler = _tx_macos_thread_suspend_handler;
    sigaction(SUSPEND_SIG, &sa, NULL);
}

/* This routine is called after initialization is complete in order to start all interrupt threads.  Interrupt threads in addition to the timer may be added to this routine as well. */
void _tx_initialize_start_interrupts(void)
{
    info("_tx_initialize_start_interrupts");

    /* Kick the timer thread off to generate the ThreadX periodic interrupt source. */
    pthread_mutex_lock(&_tx_macos_timer_mutex);
    pthread_cond_signal(&_tx_macos_timer_cond);
    pthread_mutex_unlock(&_tx_macos_timer_mutex);
}

/* Define the ThreadX system timer interrupt.
   Other interrupts may be simulated in a similar way. */

static void *_tx_macos_timer_interrupt(void *p)
{
    struct timespec ts;
    long timer_periodic_nsec;
    int err;

    (void) p;
    nice(10);

    info("timer interrupt thread\n");

    /* Wait startup semaphore. */
    pthread_mutex_lock(&_tx_macos_timer_mutex);
    pthread_cond_wait(&_tx_macos_timer_cond, &_tx_macos_timer_mutex);
    pthread_mutex_unlock(&_tx_macos_timer_mutex);

    while (1) {
        static int tick = 0;
        int result;

        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_nsec += (1000000000 / TX_TIMER_TICKS_PER_SECOND);
        if (ts.tv_nsec > 1000000000) {
            ts.tv_nsec -= 1000000000;
            ts.tv_sec++;
        }
        do {
            pthread_mutex_lock(&_tx_macos_timer_mutex);
            errno = 0;
            result = pthread_cond_timedwait(&_tx_macos_timer_cond, &_tx_macos_timer_mutex, &ts);
            err = errno;
            pthread_mutex_unlock(&_tx_macos_timer_mutex);
            if (0 == result) {
                break;
            }
        } while (result != ETIMEDOUT);
        info(".......timer interrupt.......%d", tick++);

        /* discard one tick */
        if (TX_INT_ENABLE == _tx_current_interrupt_status()) {
            info(".......timer isr.......");
            tx_macos_mutex_lock(_tx_macos_mutex);

            /* Call ThreadX context save for interrupt preparation. */
            _tx_thread_context_save();

            /* Call trace ISR enter event insert. */
            _tx_trace_isr_enter_insert(0);

            /* Call the ThreadX system timer interrupt processing. */
            _tx_timer_interrupt();
            pthread_cond_signal(&_tx_macos_schedule_cond);
            info(".......request schedule.......");

            /* Call trace ISR exit event insert. */
            _tx_trace_isr_exit_insert(0);

            /* Call ThreadX context restore for interrupt completion. */
            _tx_thread_context_restore();

            tx_macos_mutex_unlock(_tx_macos_mutex);
        }
    }
}

VOID _tx_initialize_low_level(VOID)
{
    struct sched_param sp;
    pthread_mutexattr_t attr;

    /* Pickup the first available memory address. */

    /* Save the first available memory address. */
    _tx_initialize_unused_memory = malloc(TX_MACOS_MEMORY_SIZE);

    /* Init macos thread. */
    _tx_macos_thread_init();

    /* Set priority and schedual of main thread. */
    sp.sched_priority = TX_MACOS_PRIORITY_SCHEDULE;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);

    /* Create the system critical section.
       This is used by the scheduler thread (which is the main thread) to block all other stuff out. */
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutex_init(&_tx_macos_mutex, &attr);
    pthread_cond_init(&_tx_macos_schedule_cond, NULL);

    /* Create semaphore for timer thread. */
    pthread_mutex_init(&_tx_macos_timer_mutex, NULL);
    pthread_cond_init(&_tx_macos_timer_cond, NULL);

    /* Setup periodic timer interrupt. */
    if (pthread_create(&_tx_macos_timer_id, NULL, _tx_macos_timer_interrupt, NULL)) {
        /* Error creating the timer interrupt. */
        info("ThreadX macos error creating timer interrupt thread!\n");
        dead();
    }

    /* Otherwise, we have a good thread create.
       Now set the priority to a level lower than the system thread but higher than the application threads.
       Processes scheduled under one of the real-time policies (SCHED_FIFO, SCHED_RR) have a sched_priority value in the range 1 (low) to 99 (high). */
    sp.sched_priority = TX_MACOS_PRIORITY_ISR;
    pthread_setschedparam(_tx_macos_timer_id, SCHED_FIFO, &sp);

    /* Done, return to caller. */
}
