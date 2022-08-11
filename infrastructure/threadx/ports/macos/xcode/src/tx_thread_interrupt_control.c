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

/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _tx_thread_interrupt_control                        macos/GNU       */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    cheng.gao, thoughtworks Corporation                                 */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function is responsible for changing the interrupt lockout     */
/*    posture of the system.                                              */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  08-07-2022        cheng.gao                Initial Version 6.1        */
/*                                                                        */
/**************************************************************************/
UINT _tx_thread_interrupt_disable(void)
{
    UINT previous_value;

    previous_value = _tx_thread_interrupt_control(TX_INT_DISABLE);
    return (previous_value);
}

VOID _tx_thread_interrupt_restore(UINT previous_posture)
{
    previous_posture = _tx_thread_interrupt_control(previous_posture);
}

UINT _tx_thread_interrupt_control(UINT new_posture)
{
    static int pic_status = 0; // default status of pic is disable
    UINT old_posture;
    TX_THREAD *thread_ptr;
    pthread_t thread_id;
    int exit_code = 0;

    /* Lock macos mutex. */
    tx_macos_mutex_lock(_tx_macos_mutex);

    /* Pickup the id of the current thread. */
    thread_id = pthread_self();

    /* Pickup the current thread pointer. */
    thread_ptr = _tx_thread_current_ptr;

    /* Determine the current interrupt lockout condition.  */
    if (pic_status) {
        /* Interrupts are enabled. */
        old_posture = TX_INT_ENABLE;
    } else {
        /* Interrupts are disabled. */
        old_posture = TX_INT_DISABLE;
    }

    /* First, determine if this call is from a non-thread. */
    if (_tx_thread_system_state) {
        /* Determine how to apply the new posture. */
        if (new_posture == TX_INT_ENABLE) {
            pic_status++;
        } else if (new_posture == TX_INT_DISABLE) {
            pic_status--;
        }
    } else if (thread_ptr) {
        /* Determine how to apply the new posture. */
        if (new_posture == TX_INT_ENABLE) {
            pic_status++;

            /* Clear the disabled flag. */
            _tx_thread_current_ptr->tx_thread_macos_int_disabled_flag = TX_FALSE;
        } else if (new_posture == TX_INT_DISABLE) {
            pic_status--;

            /* Set the disabled flag. */
            _tx_thread_current_ptr->tx_thread_macos_int_disabled_flag = TX_TRUE;
        }
    }

    tx_macos_mutex_unlock(_tx_macos_mutex);

    /* Return the previous interrupt disable posture. */
    return (old_posture);
}
