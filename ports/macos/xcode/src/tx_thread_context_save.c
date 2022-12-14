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
#include "tx_api.h"
#include "tx_thread.h"
#include "tx_timer.h"
#include <stdio.h>

/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _tx_thread_context_save                             macos/GNU       */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    cheng.gao, thoughtworks Corporation                                 */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function saves the context of an executing thread in the       */
/*    beginning of interrupt processing.  The function also ensures that  */
/*    the system stack is used upon return to the calling ISR.            */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  08-07-2022        cheng.gao                Initial Version 6.1        */
/*                                                                        */
/**************************************************************************/
VOID _tx_thread_context_save(VOID)
{
    /* Lock mutex to ensure other threads are not playing with the core ThreadX data structures. */
    tx_macos_mutex_lock(_tx_macos_mutex);

    /* If an application thread is running, suspend it to simulate preemption. */
    if ((_tx_thread_system_state == 0)
     && (_tx_thread_current_ptr) && (_tx_thread_current_ptr->tx_thread_init_done)) {
        info("_tx_thread_context_save %s", _tx_thread_current_ptr->tx_thread_name);

        /* Yes, this is the first interrupt and an application thread is running..., suspend it! */
        _tx_macos_thread_suspend(_tx_thread_current_ptr);
    }

    /* Increment the nested interrupt condition. */
    _tx_thread_system_state++;

    /* Unlock macos mutex. */
    tx_macos_mutex_unlock(_tx_macos_mutex);
}
