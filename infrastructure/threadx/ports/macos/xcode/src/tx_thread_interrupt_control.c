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

/* idle task interrupt flags */
static int _main_pic_status = TX_INT_ENABLE;

UINT _tx_thread_interrupt_control(UINT new_posture)
{
    UINT old_posture;

    /* Lock macos mutex. */
    tx_macos_mutex_lock(_tx_macos_mutex);

    if (NULL == _tx_thread_current_ptr) {
        old_posture = _main_pic_status;
        _main_pic_status = new_posture;
    } else {
        old_posture = _tx_thread_current_ptr->tx_macos_thread_int_flag;
        _tx_thread_current_ptr->tx_macos_thread_int_flag = new_posture;
    }

    tx_macos_mutex_unlock(_tx_macos_mutex);

    /* Return the previous interrupt disable posture. */
    return (old_posture);
}

/* current thread interrupt status */
UINT current_interrupt_status(void)
{
    UINT posture;

    /* Lock macos mutex. */
    tx_macos_mutex_lock(_tx_macos_mutex);
    if (NULL == _tx_thread_current_ptr) {
        posture = _main_pic_status;
    } else {
        posture = _tx_thread_current_ptr->tx_macos_thread_int_flag;
    }
    tx_macos_mutex_unlock(_tx_macos_mutex);
    return posture;
}
