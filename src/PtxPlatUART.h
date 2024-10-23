/** \file
    ---------------------------------------------------------------
    SPDX-License-Identifier: BSD-3-Clause

    Copyright (c) 2024, Renesas Electronics Corporation and/or its affiliates


    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.

    3. Neither the name of Renesas nor the names of its
       contributors may be used to endorse or promote products derived from this
       software without specific prior written permission.



   THIS SOFTWARE IS PROVIDED BY Renesas "AS IS" AND ANY EXPRESS
   OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
   OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL RENESAS OR CONTRIBUTORS BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
    ---------------------------------------------------------------

    Project     : PTX105R Arduino
    Module      : UART
    File        : PtxPlatUART.h

    Description : UART interface
*/

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "ptxPLAT.h"

#ifdef __cplusplus
extern "C" {
#endif

// Forward declarations
typedef uint16_t ptxStatus_t;
typedef struct ptxPlat ptxPlat_t;
typedef struct ptxPLAT_ConfigPars ptxPLAT_ConfigPars_t;

/**
 * \brief Platform-specific TX state types.
 */
typedef enum ptxPLAT_UARTState {
  PTX_PLAT_UART_StateIDLE,
  PTX_PLAT_UART_StateONGOING,
  PTX_PLAT_UART_StateDONE,
} ptxPLAT_UARTState_t;

/**
 * \brief Rx control parameters structure.
 */
typedef struct ptxPLAT_UARTRxCtrl {
  uint16_t Idx;      /**< Index of the first unused location in Rx buffer. */
  uint16_t MsgIndex; /**< Index of the next message to be processed in Standard
                        mode. */
} ptxPLAT_UARTRxCtrl_t;

/**
 * \brief Platform-specific. UART main structure.
 */
typedef struct ptxPLAT_UART {
  uint32_t IntfSpeed; /**< UART baudrate in bps. Initial should be 115200. */
  uint8_t RxBuf[PTX_PLAT_RXBUF_SIZE]; /**< Pointer to an already allocated Rx
                                         buffer. */
  ptxPLAT_UARTRxCtrl_t RxCtrl;        /**< Pointer to Rx control structure. */
} ptxPLAT_UART_t;

/**
 * \brief Initialize the UART interface.
 * \note This function shall be successfully executed before any other call to
 * the functions in this module. It initializes UART hardware wise.
 * \param[out] uart Pointer to pointer where the allocated and UART context is
 * going to be provided.
 * \param[in] config Configuration for UART.
 * \return Status, indicating whether the operation was successful. See \ref
 * ptxStatus_t.
 */
ptxStatus_t ptxPlat_uartInit(ptxPLAT_UART_t **uart,
                             ptxPLAT_ConfigPars_t *config);

/**
 * \brief Deinitialize the UART interface.
 * \return Status, indicating whether the operation was successful. See \ref
 * ptxStatus_t.
 */
ptxStatus_t ptxPlat_uartDeinit();

/**
 * \brief UART transmit and receive function.
 * \note Wrapper function for \ref ptxPLAT_TRx. See it for detailed description
 * \param[in]     uart         Pointer to an initialized UART context.
 * \param[in]     txBuf        Array of buffers to transmit.
 * \param[in]     txLen        Array of lengths of buffers to transmit.
 * \param[in]     numTxBuffers Number of buffers to transmit.
 * \param[out]    rxBuf        Array of buffers to receive.
 * \param[in,out] rxLen        Array of lengths of buffers to receive.
 * \param[in]     numTxBuffers Number of buffers to receive.
 * \return Status, indicating whether the operation was successful. See \ref
 * ptxStatus_t.
 */
ptxStatus_t ptxPlat_uartTrx(ptxPLAT_UART_t *uart, const uint8_t *txBuf[],
                            size_t txLen[], size_t numTxBuffers,
                            uint8_t *rxBuf[], size_t *rxLen[],
                            size_t numRxBuffers);

/**
 * \brief Triggers Rx callback, if Rx transfer is done.
 * \param[in] plat Pointer to an initialized PLAT context.
 * \return Status, indicating whether the operation was successful. See \ref
 * ptxStatus_t.
 */
ptxStatus_t ptxPlat_uartTriggerRx(ptxPlat_t *plat);

/**
 * \brief Check if UART reception is pending.
 * \return Rx status: 0 - UART is idle, nothing to receive, 1 - UART is not
 * idle.
 */
uint8_t ptxPlat_uartIsRxPending();

/**
 * \brief Check if UART reception is taking place at the moment.
 * \return Rx status: 0 - no rx activity, 1 - rx activity ongoing.
 */
uint8_t ptxPlat_uartCheckRxActive();

/**
 * \brief Set UART bitrate.
 * This function is used to change already set bitrate. It is called after
 * NSC_INIT_CMD/RESP to update UART settings.
 * \param[in] uart    Pointer to an initialized UART context.
 * \param[in] bitrate New bitrate for the UART interface.
 * \return Status, indicating whether the operation was successful. See \ref
 * ptxStatus_t.
 */
ptxStatus_t ptxPLAT_uartSetIntfSpeed(ptxPLAT_UART_t *uart, uint32_t bitrate);

/**
 * \brief Set Rx control parameters to initial values.
 * This is typically called before starting Tx operation, so that asynchronous
 * Rx operation after Tx could start with expected state.
 * \param[in] uart Pointer to an initialized UART context.
 * \return Status, indicating whether the operation was successful. See \ref
 * ptxStatus_t.
 */
ptxStatus_t ptxPLAT_uartSetCleanStateRx(ptxPLAT_UART_t *uart);

/**
 * \brief Stores received UART message in given destination buffer and set
 * received message length.
 * \note Used only for UART interface.
 * \param[in] uart            Pointer to an initialized UART context.
 * \param[in] rxMessageBuffer Pointer to buffer where received message shall be
 * stored. Buffer size must be >= PTX_PLAT_RXBUF_SIZE.
 * \param[out] rxMessageBufferLen Pointer to variable where received message
 * length shall be stored.
 * \return Status, indicating whether the operation was successful. See \ref
 * ptxStatus_t.
 */
ptxStatus_t ptxPLAT_uartGetReceivedMessage(ptxPLAT_UART_t *uart,
                                           uint8_t *rxMessageBuffer,
                                           size_t *rxMessageBufferLen);

#ifdef __cplusplus
}
#endif