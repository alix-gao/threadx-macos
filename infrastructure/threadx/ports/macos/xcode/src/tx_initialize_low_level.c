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

#include "tx_api.h"

/* Define various macos objects used by the ThreadX port. */
pthread_mutex_t _tx_macos_mutex;
sem_t *_tx_schedule_semaphore;

/* trace */
struct timespec _tx_macos_time_stamp;

/* Define simulated timer interrupt.  This is done inside a thread, which is
   how other interrupts may be defined as well.  See code below for an
   example.  */

pthread_t           _tx_macos_timer_id;
pthread_cond_t _tx_macos_timer_cond;
pthread_mutex_t _tx_macos_timer_mutex;
sem_t               *_tx_macos_isr_semaphore;
void               *_tx_macos_timer_interrupt(void *p);

/* Define the ThreadX timer interrupt handler. */
void _tx_timer_interrupt(void);

/* Define other external function references. */
VOID    _tx_thread_context_save(VOID);
VOID    _tx_thread_context_restore(VOID);

VOID _tx_thread_schedule(VOID);

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
    sem_unlink("_tx_schedule_semaphore");
    _tx_schedule_semaphore = sem_open("_tx_schedule_semaphore", O_CREAT, 0666, 0);

    /* Create semaphore for timer thread. */
    pthread_mutex_init(&_tx_macos_timer_mutex, NULL);
    pthread_cond_init(&_tx_macos_timer_cond, NULL);

    /* Create semaphore for ISR thread. */
    sem_unlink("_tx_macos_isr_semaphore");
    _tx_macos_isr_semaphore = sem_open("_tx_macos_isr_semaphore", O_CREAT, 0666, 0);

    /* Setup periodic timer interrupt. */
    if (pthread_create(&_tx_macos_timer_id, NULL, _tx_macos_timer_interrupt, NULL)) {
        /* Error creating the timer interrupt. */
        info("ThreadX macos error creating timer interrupt thread!\n");
        while (1) {
        }
    }

    /* Otherwise, we have a good thread create.
       Now set the priority to a level lower than the system thread but higher than the application threads. */
    sp.sched_priority = TX_MACOS_PRIORITY_ISR;
    pthread_setschedparam(_tx_macos_timer_id, SCHED_FIFO, &sp);

    /* Done, return to caller. */
}

/* This routine is called after initialization is complete in order to start all interrupt threads.  Interrupt threads in addition to the timer may be added to this routine as well. */
void _tx_initialize_start_interrupts(void)
{
    /* Kick the timer thread off to generate the ThreadX periodic interrupt source. */
    pthread_mutex_lock(&_tx_macos_timer_mutex);
    pthread_cond_signal(&_tx_macos_timer_cond);
    pthread_mutex_unlock(&_tx_macos_timer_mutex);
}

/* Define the ThreadX system timer interrupt.
   Other interrupts may be simulated in a similar way. */
#include <sys/time.h>

void *_tx_macos_timer_interrupt(void *p)
{
    struct timespec ts;
    long timer_periodic_nsec;
    int err;

    (VOID) p;

    /* Calculate periodic timer. */
    timer_periodic_nsec = 1000000000 / TX_TIMER_TICKS_PER_SECOND;
    nice(10);

    info("timer interrupt thread\n");

    /* Wait startup semaphore. */
    pthread_mutex_lock(&_tx_macos_timer_mutex);
    pthread_cond_wait(&_tx_macos_timer_cond, &_tx_macos_timer_mutex);
    pthread_mutex_unlock(&_tx_macos_timer_mutex);

    while (1) {
        static int tick = 0;
        int result;
info("::::::::timer loop:::::::::: %d", tick++);
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_nsec += timer_periodic_nsec;
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
info(":::::time end......");
        tx_macos_mutex_lock(_tx_macos_mutex);
        /* Call ThreadX context save for interrupt preparation. */
        _tx_thread_context_save();
info(":::::time end....333..");
        /* Call trace ISR enter event insert. */
        _tx_trace_isr_enter_insert(0);
info(":::::::timer proc start");
        /* Call the ThreadX system timer interrupt processing. */
        _tx_timer_interrupt();
        tx_macos_sem_post_nolock(_tx_schedule_semaphore);

info(":::::::timer proc end");
        /* Call trace ISR exit event insert. */
        _tx_trace_isr_exit_insert(0);

        /* Call ThreadX context restore for interrupt completion. */
        _tx_thread_context_restore();
        tx_macos_mutex_unlock(_tx_macos_mutex);
    }
}

/* Define signals for macos thread. */
#define SUSPEND_SIG SIGUSR1
#define RESUME_SIG SIGUSR2

static sigset_t _tx_macos_thread_wait_mask;

/* signal cannot pass parameters,
   use __thread to instead of thread info, bool variable does not need to be protected */
static __thread bool _tx_macos_thread_suspended = false;

/* Define functions for macos thread. */
void _tx_macos_thread_resume_handler(int sig)
{
    (VOID) sig;
}

void _tx_macos_thread_suspend_handler(int sig)
{
    (VOID) sig;
info("suspend handler, %d %lx %lx", pthread_equal(pthread_self(), _tx_macos_timer_id), pthread_self(), _tx_macos_timer_id);

    _tx_macos_thread_suspended = true;
    sigsuspend(&_tx_macos_thread_wait_mask);
    _tx_macos_thread_suspended = false;
}

void _tx_macos_thread_suspend(TX_THREAD *thread)
{
    sigset_t set;
    pthread_t thread_id = thread->tx_macos_thread_id;

    if (_tx_macos_thread_suspended) {
        printf("&");
        return;
    }
    if (0 == thread_id) { dump_callstack(); }

    if (pthread_kill(thread_id, 0)) {
        printf("%p-%s\n",thread_id, thread->tx_thread_name);info("thread not exist");
        return;
    }

    sigpending(&set);
    if (sigismember(&set, SUSPEND_SIG)) {
        info("thread is pending");
        return;
    }
info("suspend %d %lx", pthread_equal(thread_id, _tx_macos_timer_id), thread_id);
    /* Send signal. */
    pthread_kill(thread_id, SUSPEND_SIG);
    info("suspend killed");
}

void _tx_macos_thread_resume(TX_THREAD *thread)
{
info("resume %lx", thread->tx_macos_thread_id);

    /* Send signal. */
    tx_macos_mutex_lock(_tx_macos_mutex);
    pthread_kill(thread->tx_macos_thread_id, RESUME_SIG);
    tx_macos_mutex_unlock(_tx_macos_mutex);
}

void _tx_macos_thread_init(void)
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
