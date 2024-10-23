/** \file
    ---------------------------------------------------------------
    SPDX-License-Identifier: BSD-3-Clause

    Copyright (c) 2024, Renesas Electronics Corporation and/or its affiliates


    Redistribution and use in source and binary forms, with or without modification,
    are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice, this list of
       conditions and the following disclaimer in the documentation and/or other
       materials provided with the distribution.

    3. Neither the name of Renesas nor the names of its
       contributors may be used to endorse or promote products derived from this
       software without specific prior written permission.



    THIS SOFTWARE IS PROVIDED BY Renesas "AS IS" AND ANY EXPRESS
    OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
    OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL RENESAS OR CONTRIBUTORS BE
    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
    CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
    GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
    HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
    OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
    ---------------------------------------------------------------

    Project     : PTX1K
    Module      : NSC
    File        : ptxNSC_Intf_UART.c

    Description :
*/

/*
 * ####################################################################################################################
 * INCLUDES
 * ####################################################################################################################
 */

#include "ptxNSC_Intf.h"
#include "ptxNSC.h"
#include "ptxNSC_Hal.h"
#include "ptxNSC_Registers.h"
#include "ptxPLAT.h"
#include "ptxStatus.h"
#include <string.h>

/*
 * ####################################################################################################################
 * DEFINES / TYPES
 * ####################################################################################################################
 */

// Defines for HIF_UART_CONFIG0_REG
#define HIF_UART_CONFIG0_REG                                      (0x1013U)
#define HIF_UART_CONFIG0_REG_UART_BR_DIV0_VAL_MASK                (0x07U)
#define HIF_UART_CONFIG0_REG_UART_FLWCTRL_POS                     (3U)

// Defines for HIF_UART_CONFIG1_REG
#define HIF_UART_CONFIG1_REG                                      (0x1014U)
#define HIF_UART_CONFIG1_REG_UART_BR_DIV1_VAL_MASK                (0x7FU)
#define HIF_UART_CONFIG1_REG_UART_BR_DITHER_EN_MASK               (0x80U)

// Defines for HIF_UART_CONFIG2_REG
#define HIF_UART_CONFIG2_REG                                      (0x1015U)
#define HIF_UART_CONFIG2_REG_UART_RX_TRANS_EN_MASK                (0x02U)
#define HIF_UART_CONFIG2_REG_UART_RX_TRANS_EN_MASK_INV            (0xFDU)

#define HIF_UART_CONFIG2_REG_UART_FLWCTRL_INV_POL_MASK            (0x01U)
#define HIF_UART_CONFIG2_REG_UART_FLWCTRL_INV_POL_MASK_INV        (0xFEU)

// Some systems require inverted FlowControl-lines i.e. low-active mode -> comment this line if standard mode should be used i.e. high-active mode
#define HIF_UART_USE_INVERTED_FLOW_CONTROL

/*
 * ####################################################################################################################
 * INTERNAL FUNCTIONS
 * ####################################################################################################################
 */

/*
 * ####################################################################################################################
 * API FUNCTIONS
 * ####################################################################################################################
 */

void ptxNSC_GetRx (ptxNSC_t *nscCtx)
{
    ptxStatus_t status;

    uint8_t buf[PTX_PLAT_RXBUF_SIZE];
    size_t buf_size = 0;

    ptxNSC_t *nsc_ctx = (NULL != nscCtx) ? (ptxNSC_t *)nscCtx : NULL;

    if ((NULL != nsc_ctx) && (ptxStatus_Comp_NSC == nsc_ctx->CompId) && (NULL != nsc_ctx->Plat))
    {
        /**
         * This is a callback for the Rx ISR. It is called in System mode when Rx buffer has been filled up to the
         * number of bytes equal to the RxBuffer[0].
         *
         * NOTE: it is expected that PLAT component and interface buffers to be allocated prior to this call.
         */

        status = ptxPLAT_GetReceivedMessage(nsc_ctx->Plat, &buf[0], &buf_size);

        if (ptxStatus_Success == status)
        {
            (void) ptxNSC_Process (nsc_ctx, &buf[0], buf_size);
        }
    }
}

ptxStatus_t ptxNSC_UARTSetCleanStateRx (ptxNSC_t *nscCtx)
{
    ptxStatus_t status = ptxStatus_Success;

    ptxNSC_t *nsc_ctx = (NULL != nscCtx) ? (ptxNSC_t *)nscCtx : NULL;

    if ((NULL != nsc_ctx) && (ptxStatus_Comp_NSC == nsc_ctx->CompId) && (NULL != nsc_ctx->Plat))
    {
        status = ptxPLAT_SetCleanStateRx (nscCtx->Plat);
    } else
    {
        status = PTX_STATUS(ptxStatus_Comp_NSC, ptxStatus_InternalError);
    }

    return status;
}


ptxStatus_t ptxNSC_HAL_WriteBuffer(ptxNSC_t *nscCtx, ptxNscHal_BufferId_t bufferId, uint8_t *txBuf[], size_t txLen[], size_t numBuffers)
{
    ptxStatus_t status = ptxStatus_Success;
    const size_t max_num_buffers = 3u;

    if (PTX_COMP_CHECK(nscCtx, ptxStatus_Comp_NSC) && (bufferId < NscWriteBuffer_Max)
            && (NULL != txBuf) && (NULL != txLen) && (numBuffers > 0) && (numBuffers <= max_num_buffers))
    {
        if (NscMode_SYS == nscCtx->NscMode)
        {
            const uint8_t ptxNsc_Buffer_Add_Mask = 0x1F;
            const uint8_t ptxNsc_Buffer_Id_Mask = 0x07;
            const size_t num_buf = numBuffers + 2u;
            uint8_t address_write;
            uint8_t *tx_buf       [num_buf];
            size_t tx_len         [num_buf];

            uint8_t sof = 0x55;
            const size_t prepend_array_len = 2;
            uint8_t prepend_array [prepend_array_len];

            prepend_array [0] = sof;
            prepend_array [1] = 0; /* This length is going to collect the length of all the bytes to be sent out. */

            address_write = (uint8_t)((((uint8_t)bufferId) & ptxNsc_Buffer_Add_Mask) | ((ptxNsc_Buffer_Id_Mask) << 5u));

            tx_buf[0] = &prepend_array [0];
            tx_len[0] = prepend_array_len;

            /* Send to HAL with the same concept of the array of buffers to prevent double allocation on the Stack of the uCode */
            tx_buf[1] = &address_write;
            tx_len[1] = 1u;
            prepend_array[1] = (uint8_t)(prepend_array[1] + tx_len[1]);

            for (size_t i=2; i < num_buf; i++)
            {
                if (txBuf[i-2u] != NULL)
                {
                    tx_buf[i] = txBuf[i-2u];
                    tx_len[i] = txLen[i-2u];

                    prepend_array[1] = (uint8_t)(prepend_array[1] + tx_len[i]);
                } else
                {
                    /* Wrong buffer provided */
                    status = PTX_STATUS(ptxStatus_Comp_NSC, ptxStatus_InvalidParameter);
                    break;
                }
            }

            if (ptxStatus_Success == status)
            {
                status = ptxPLAT_TRx(nscCtx->Plat, tx_buf, tx_len, num_buf, NULL, NULL, 0, 0);
            }
        } else
        {
            status = PTX_STATUS(ptxStatus_Comp_NSC, ptxStatus_InternalError);
        }
    } else
    {
        status = PTX_STATUS(ptxStatus_Comp_NSC, ptxStatus_InvalidParameter);
    }
    return status;
}

ptxStatus_t ptxNSC_HAL_Wra(ptxNSC_t *nscCtx, uint16_t address, uint8_t value)
{
    ptxStatus_t status = ptxStatus_Success;

    if (PTX_COMP_CHECK(nscCtx, ptxStatus_Comp_NSC))
    {
        if (NscMode_HW == nscCtx->NscMode)
        {
            const size_t num_buf_tx = 3u;
            uint8_t *tx_buf [num_buf_tx];
            size_t tx_len [num_buf_tx];

            const size_t addr_write_len = 2u;
            uint8_t address_write [addr_write_len];

            uint8_t sof = 0x55;
            const size_t prepend_array_len = 2;
            uint8_t prepend_array [prepend_array_len];

            const size_t num_buf_rx = 1u;
            uint8_t *rx_buf[num_buf_rx];
            size_t *rx_buf_len[num_buf_rx];
            uint8_t rx;
            size_t rx_len = 1;

            prepend_array [0] = sof;
            prepend_array [1] = 3u;

            address_write[0] = (uint8_t)((address & 0xFF00u) >> 8) | (uint8_t)((uint8_t)PTX_NSC_HAL_WRITE_RANDOM_ADDRESS_MASK << 5);
            address_write[1] = (uint8_t) (address & 0x00FFu);

            /*
             * Three buffers are used for TX
             */
            tx_buf[0] = &prepend_array[0];
            tx_len[0] = prepend_array_len;

            tx_buf[1] = &address_write[0];
            tx_len[1] = 2;

            tx_buf[2] = &value;
            tx_len[2] = 1;

            /*
             * In HW Mode == One buffer is used for RX Operation. Wait for HW ACK
             */
            rx_buf[0] = &rx;
            rx_buf_len[0] = &rx_len;

            status = ptxPLAT_TRx(nscCtx->Plat, tx_buf, tx_len, num_buf_tx, rx_buf, rx_buf_len, num_buf_rx, 0);

            if(ptxStatus_Success == status)
            {
                if((rx_len == 1u) && (rx == 0x00))
                {
                    /* HW ACK received => Successful operation. */
                } else
                {
                    status = PTX_STATUS(ptxStatus_Comp_NSC, ptxStatus_InternalError);
                }
            }
        } else
        {
            status = PTX_STATUS(ptxStatus_Comp_NSC, ptxStatus_InternalError);
        }

    } else
    {
        status = PTX_STATUS(ptxStatus_Comp_NSC, ptxStatus_InvalidParameter);
    }

    return status;
}

ptxStatus_t ptxNSC_HAL_Wra_NoCheck(ptxNSC_t *nscCtx, uint16_t address, uint8_t value)
{
    /* Call ptxNSC_HAL_Wra_NoWait without checking the mode. This is special case used just for DFY activation. */
    ptxStatus_t status = ptxStatus_Success;

    if (PTX_COMP_CHECK(nscCtx, ptxStatus_Comp_NSC))
    {
        const size_t num_buf_tx = 3u;
        uint8_t *tx_buf [num_buf_tx];
        size_t tx_len [num_buf_tx];

        const size_t addr_write_len = 2u;
        uint8_t address_write [addr_write_len];

        uint8_t sof = 0x55;
        const size_t prepend_array_len = 2;
        uint8_t prepend_array [prepend_array_len];

        prepend_array [0] = sof;
        prepend_array [1] = 3u;

        address_write[0] = (uint8_t)((address & 0xFF00u) >> 8) | (uint8_t)((uint8_t)PTX_NSC_HAL_WRITE_RANDOM_ADDRESS_MASK << 5);
        address_write[1] = (uint8_t) (address & 0x00FFu);

        /*
         * Three buffers are used for TX
         */
        tx_buf[0] = &prepend_array[0];
        tx_len[0] = prepend_array_len;

        tx_buf[1] = &address_write[0];
        tx_len[1] = 2;

        tx_buf[2] = &value;
        tx_len[2] = 1;

        status = ptxPLAT_TRx(nscCtx->Plat, tx_buf, tx_len, num_buf_tx, NULL, NULL, 0, 0);
    } else
    {
        status = PTX_STATUS(ptxStatus_Comp_NSC, ptxStatus_InvalidParameter);
    }

    return status;
}

ptxStatus_t ptxNSC_HAL_Wra_NoWait(ptxNSC_t *nscCtx, uint16_t address, uint8_t value)
{
    ptxStatus_t status = ptxStatus_Success;

    if (PTX_COMP_CHECK(nscCtx, ptxStatus_Comp_NSC))
    {
        if (NscMode_HW == nscCtx->NscMode)
        {
            const size_t num_buf_tx = 3u;
            uint8_t *tx_buf [num_buf_tx];
            size_t tx_len [num_buf_tx];

            const size_t addr_write_len = 2u;
            uint8_t address_write [addr_write_len];

            uint8_t sof = 0x55;
            const size_t prepend_array_len = 2;
            uint8_t prepend_array [prepend_array_len];

            prepend_array [0] = sof;
            prepend_array [1] = 3u;

            address_write[0] = (uint8_t)((address & 0xFF00u) >> 8) | (uint8_t)((uint8_t)PTX_NSC_HAL_WRITE_RANDOM_ADDRESS_MASK << 5);
            address_write[1] = (uint8_t) (address & 0x00FFu);

            /*
             * Three buffers are used for TX
             */
            tx_buf[0] = &prepend_array[0];
            tx_len[0] = prepend_array_len;

            tx_buf[1] = &address_write[0];
            tx_len[1] = 2;

            tx_buf[2] = &value;
            tx_len[2] = 1;

            status = ptxPLAT_TRx(nscCtx->Plat, tx_buf, tx_len, num_buf_tx, NULL, NULL, 0, 0);
        } else
        {
            status = PTX_STATUS(ptxStatus_Comp_NSC, ptxStatus_InternalError);
        }
    } else
    {
        status = PTX_STATUS(ptxStatus_Comp_NSC, ptxStatus_InvalidParameter);
    }

    return status;
}

ptxStatus_t ptxNSC_HAL_WriteInstruction(ptxNSC_t *nscCtx, uint16_t address, uint8_t *pPayload, size_t txLen )
{
    ptxStatus_t status = ptxStatus_Success;

    if (PTX_COMP_CHECK(nscCtx, ptxStatus_Comp_NSC) && (NULL != pPayload) && (txLen > 0))
    {
        if (NscMode_HW == nscCtx->NscMode)
        {
            const size_t num_buf_tx = 3u;
            uint8_t *tx_buf       [num_buf_tx];
            size_t tx_len         [num_buf_tx];

            const size_t num_buf_rx = 1u;
            uint8_t *rx_buf[num_buf_rx];
            size_t *rx_buf_len[num_buf_rx];

            const size_t addr_write_len = 2u;
            uint8_t address_write [addr_write_len];
            uint8_t sof = 0x55;
            const size_t prepend_array_len = 2;
            uint8_t prepend_array [prepend_array_len];

            prepend_array [0] = sof;
            prepend_array [1] = (uint8_t)(2u + txLen);

            address_write[0] = (uint8_t)((address & 0xFF00u) >> 8) | (uint8_t)((uint8_t)PTX_NSC_HAL_WRITE_INSTRUCTION_MASK << 5);
            address_write[1] = (uint8_t) (address & 0x00FFu);

            /*
             * Three buffers are used for TX Operation
             */
            tx_buf[0] = &prepend_array[0];
            tx_len[0] = prepend_array_len;

            tx_buf[1] = &address_write[0];
            tx_len[1] = 2;

            tx_buf[2] = pPayload;
            tx_len[2] = txLen;

            /*
             * One buffer is used for RX Operation. Wait for HW ACK
             */
            uint8_t rx;
            size_t rx_len = 1;

            rx_buf[0] = &rx;
            rx_buf_len[0] = &rx_len;

            status = ptxPLAT_TRx(nscCtx->Plat, tx_buf, tx_len, num_buf_tx, rx_buf, rx_buf_len, num_buf_rx, 0);

            if(ptxStatus_Success == status)
            {
                if((rx_len == 1u) && (rx == 0x00))
                {
                    /* HW ACK received => Successful operation. */
                } else
                {
                    status = PTX_STATUS(ptxStatus_Comp_NSC, ptxStatus_InternalError);
                }
            }
        } else
        {
            status = PTX_STATUS(ptxStatus_Comp_NSC, ptxStatus_InternalError);
        }
    } else
    {
        status = PTX_STATUS(ptxStatus_Comp_NSC, ptxStatus_InvalidParameter);
    }

    return status;
}

ptxStatus_t ptxNSC_HAL_Rra(ptxNSC_t *nscCtx, uint16_t address, uint8_t *value)
{
    ptxStatus_t status = ptxStatus_Success;

    /*
     * Each interface has its own peculiarities. What is common and required is the address to read from.
     * Also, valid pointer for the returned data has to be provided.
     * It is expected to read a single byte from the given address.
     *
     * Here we have to provide the local rx buffer big enough to handle all interface�s response data.
     */
    if (PTX_COMP_CHECK(nscCtx, ptxStatus_Comp_NSC) && (NULL != value))
    {
        if (NscMode_HW == nscCtx->NscMode)
        {
            const size_t num_buf_tx = 2u;
            uint8_t *tx_buf[num_buf_tx];
            size_t tx_len[num_buf_tx];

            const size_t num_buf_rx = 1u;
            uint8_t *rx_buf[num_buf_rx];
            size_t *rx_buff_len[num_buf_rx];

            const size_t addr_read_len = PTX_NSC_HAL_ADDRESS_LENGTH;
            uint8_t address_to_read[addr_read_len];
            uint8_t sof = 0x55;

            const size_t prepend_array_len = 2;
            uint8_t prepend_array [prepend_array_len];

            size_t rx_len = 2u;
            uint8_t rx_buf_local[rx_len];

            prepend_array [0] = sof;
            prepend_array [1] = 2u;

            address_to_read[0] = (uint8_t)((address & 0xFF00u) >> 8) | (uint8_t)((uint8_t)PTX_NSC_HAL_READ_RANDOM_ADDRESS_MASK << 5);
            address_to_read[1] = (uint8_t) (address & 0x00FFu);

            /*
             * Two buffers are used for TX
             */
            tx_buf[0]  = &prepend_array [0];
            tx_len[0]  = prepend_array_len;

            tx_buf[1u] = &address_to_read[0];
            tx_len[1u] = PTX_NSC_HAL_ADDRESS_LENGTH;

            /*
             * One buffer is used for RX
             */
            rx_buf[0] = &rx_buf_local[0];
            rx_buff_len[0] = &rx_len;

            status = ptxPLAT_TRx(nscCtx->Plat, tx_buf, tx_len, num_buf_tx, rx_buf, rx_buff_len, num_buf_rx, 0);

            if(ptxStatus_Success == status)
            {
                if(rx_len > 0)
                {
                    *value = rx_buf_local[1u];
                } else
                {
                    status = PTX_STATUS(ptxStatus_Comp_NSC, ptxStatus_InternalError);
                }
            }
        } else
        {
            status = PTX_STATUS(ptxStatus_Comp_NSC, ptxStatus_InternalError);
        }
    } else
    {
        status = PTX_STATUS(ptxStatus_Comp_NSC, ptxStatus_InvalidParameter);
    }

    return status;
}

ptxStatus_t ptxNSC_SetMode (ptxNSC_t *nscCtx, ptxNSC_Mode_t newMode)
{
    ptxStatus_t status = ptxStatus_Success;

    if (PTX_COMP_CHECK(nscCtx, ptxStatus_Comp_NSC) && ((NscMode_HW == newMode) || (NscMode_SYS == newMode)))
    {
        if (newMode == nscCtx->NscMode)
        {
            /* New State same as current one. Nothing to do. */
        } else
        {
            /* Update State*/
            if (newMode == NscMode_SYS)
            {
                /* Disable HW ACK and Start RX Thread. */

                uint8_t value;
                status = ptxNSC_HAL_Rra(nscCtx, (uint16_t)HIF_UART_CONFIG2_REG, &value);

                if (ptxStatus_Success == status)
                {
                    if ((value & HIF_UART_CONFIG2_REG_UART_RX_TRANS_EN_MASK) == 0x00)
                    {
                        value |= HIF_UART_CONFIG2_REG_UART_RX_TRANS_EN_MASK;
#ifdef HIF_UART_USE_INVERTED_FLOW_CONTROL
                        value |= HIF_UART_CONFIG2_REG_UART_FLWCTRL_INV_POL_MASK;
#endif
                        status = ptxNSC_HAL_Wra (nscCtx, HIF_UART_CONFIG2_REG, value);
                    }
                }

                if (ptxStatus_Success == status)
                {
                    /* Start RX thread. */
                    status = ptxNSC_Start_WaitForRx(nscCtx);

                    if (ptxStatus_Success == status)
                    {
                        /* Update State. */
                        nscCtx->NscMode = newMode;
                    }
                }

            } else
            {
                /* Enable HW ACK and Stop RX Thread. */
                /* Stop RX thread. */
                status = ptxNSC_Stop_WaitForRx(nscCtx);

                if (ptxStatus_Success == status)
                {
                    uint8_t value;

                    /* Update State. */
                    nscCtx->NscMode = newMode;

                    status = ptxNSC_HAL_Rra(nscCtx, HIF_UART_CONFIG2_REG, &value);

                    if (ptxStatus_Success == status)
                    {
                        if ((value & HIF_UART_CONFIG2_REG_UART_RX_TRANS_EN_MASK) == HIF_UART_CONFIG2_REG_UART_RX_TRANS_EN_MASK)
                        {
                            value &= HIF_UART_CONFIG2_REG_UART_RX_TRANS_EN_MASK_INV;
                            /* This WRA is not acknowledged. After that, all other are acknowledged. */
                            status = ptxNSC_HAL_Wra_NoWait (nscCtx, HIF_UART_CONFIG2_REG, value);
                        }
                    }
                }
            }
        }
    } else
    {
        status = PTX_STATUS(ptxStatus_Comp_NSC, ptxStatus_InvalidParameter);
    }

    return status;

}

ptxStatus_t ptxNSC_GetMode (ptxNSC_t *nscCtx, ptxNSC_Mode_t *currentMode)
{
    ptxStatus_t status = ptxStatus_Success;

    if ((PTX_COMP_CHECK(nscCtx, ptxStatus_Comp_NSC)) && (NULL != currentMode))
    {
        *currentMode = nscCtx->NscMode;
    } else
    {
        status = PTX_STATUS(ptxStatus_Comp_NSC, ptxStatus_InvalidParameter);
    }

    return status;
}

ptxStatus_t ptxNSC_SoftReset(ptxNSC_t *nscCtx)
{
    ptxStatus_t st = ptxStatus_Success;
    const uint32_t ms_to_reset = 10UL;

    /*
     * Soft Reset Done in HW Mode
     */
    ptxNSC_SetMode(nscCtx, NscMode_HW);

    /*
     * The result of the WRA of the soft Reset is uncertain. It is possible that the NSC does
     * not acknowledge it properly as it is resetting itself.
     * However, here we expect only that Wra operation finishes successfully, nothing else.
     */
    (void) ptxNSC_HAL_Wra_NoWait(nscCtx, SYS_CONTROL_REG, SYS_CONTROL_REG_SYS_SOFT_RESET_MASK);

    /*
     * Sleep time needed to ensure that the actual Reset has been performed.
     */
    st = ptxPLAT_Sleep(nscCtx->Plat, ms_to_reset);

    return st;
}

ptxStatus_t ptxNSC_GetInitConfigParams(ptxNSC_t *nscCtx, uint32_t baudRate, uint8_t *uartConfig)
{
    ptxStatus_t st = ptxStatus_Success;

    if ((0 != baudRate) && (NULL != uartConfig))
    {
        /*
         * UART Registers HIF_UART_CONFIG0_REG & HIF_UART_CONFIG1_REG.
         */
        uint8_t NSC_Init_UART_BR_DITHER_EN = 0;                         /* Needed for some BaudRates, default 0. Bit 7 of HIF_UART_CONFIG0_REG*/
        uint8_t NSC_Init_UART_BR_DIV1_VAL = 0;                          /* divider1 of the baudrate generator */
        uint8_t NSC_Init_UART_BR_DET_EN = 0;                            /* Disabling automatic baud rate detection. Bit 7. */
        uint8_t NSC_Init_UART_BR_DIV0_VAL = 0;                          /* divider0 of the baudrate generator */
        uint8_t NSC_Init_UART_FLWCTRL = 1u << HIF_UART_CONFIG0_REG_UART_FLWCTRL_POS;    /* Enabling HW flow control lines. */

        switch(baudRate)
        {
            case 9600:
                NSC_Init_UART_BR_DIV0_VAL=7;
                NSC_Init_UART_BR_DIV1_VAL=43;
                break;

            case 14400:
                NSC_Init_UART_BR_DIV0_VAL=6;
                NSC_Init_UART_BR_DIV1_VAL=58;
                break;

            case 19200:
                NSC_Init_UART_BR_DIV0_VAL=6;
                NSC_Init_UART_BR_DIV1_VAL=43;
                break;

            case 28800:
                NSC_Init_UART_BR_DIV0_VAL=5;
                NSC_Init_UART_BR_DIV1_VAL=58;
                break;

            case 38400:
                NSC_Init_UART_BR_DIV0_VAL=5;
                NSC_Init_UART_BR_DIV1_VAL=43;
                break;

            case 57600:
                NSC_Init_UART_BR_DIV0_VAL=4;
                NSC_Init_UART_BR_DIV1_VAL=58;
                break;

            case 115200:
                NSC_Init_UART_BR_DIV0_VAL=3;
                NSC_Init_UART_BR_DIV1_VAL=58;
                break;

            case 153600:
                NSC_Init_UART_BR_DIV0_VAL=3;
                NSC_Init_UART_BR_DIV1_VAL=43;
                break;

            case 230400:
                NSC_Init_UART_BR_DIV0_VAL=2;
                NSC_Init_UART_BR_DIV1_VAL=58;
                break;

            case 1000000:
                NSC_Init_UART_BR_DIV0_VAL=0;
                NSC_Init_UART_BR_DIV1_VAL=54;
                NSC_Init_UART_BR_DITHER_EN = HIF_UART_CONFIG1_REG_UART_BR_DITHER_EN_MASK;
                break;

            case 3000000:
                NSC_Init_UART_BR_DIV0_VAL=0;
                NSC_Init_UART_BR_DIV1_VAL=17;
                break;

            default:
                break;
        }

        /*
         * HIF_UART_CONFIG1_REG
         */
        uartConfig[0] = NSC_Init_UART_BR_DITHER_EN | NSC_Init_UART_BR_DIV1_VAL;

        /*
         * HIF_UART_CONFIG0_REG
         */
        uartConfig[1] = NSC_Init_UART_BR_DET_EN | NSC_Init_UART_FLWCTRL | NSC_Init_UART_BR_DIV0_VAL;
    }

    (void)nscCtx;

    return st;
}

ptxStatus_t ptxNSC_UARTComSync(ptxNSC_t *nscCtx)
{
    ptxStatus_t st = ptxStatus_Success;

    const size_t NR_SYNC_BITRATES = 3;
    const uint32_t SYNC_BITRATES[] = {PTX_PLAT_HOST_SPEED_UART_115200, PTX_PLAT_HOST_SPEED_UART_57600, PTX_PLAT_HOST_SPEED_UART_19200};

    /*
     * Try to establish proper communication link with NSC chip.
     *
     * NOTE: The PTX1K chip supports automatic bitrate-detection after reset up to 115200 baud.
     *       The synchronization will start at 115200 baud and will try a couple lower values should 115200 baud not be successful
     *       (strongly depending on Host-system).
     *       The initial speed will then remain during the initialization phase incl. FW-download.
     *       Once the FW-download is completed, the system will switch from HW-mode to SYS-mode and the originally targeted UART-bitrate
     *       will be applied/set during the NSC_INIT_CMD.
     */
    if (PTX_COMP_CHECK(nscCtx, ptxStatus_Comp_NSC) && (NscMode_HW == nscCtx->NscMode))
    {
        uint8_t value = 0;
        uint8_t synchronized = 0;
        size_t i = 0;

        do
        {
            st = ptxPLAT_SetIntfSpeed(nscCtx->Plat, SYNC_BITRATES[i]);

            if (ptxStatus_Success == st)
            {
                st = ptxNSC_HAL_Rra(nscCtx, VERSION_REG, &value);

                if ((ptxStatus_Success == st) && (VERSION_REG_RST == value))
                {
                    synchronized = 1U;
                }
            }

            i++;

        } while ((0 == synchronized) && (ptxStatus_Success == st) && (i < NR_SYNC_BITRATES));

        if (0 == synchronized)
        {
            st = PTX_STATUS(ptxStatus_Comp_NSC, ptxStatus_InterfaceError);
        }

    } else
    {
        st = PTX_STATUS(ptxStatus_Comp_NSC, ptxStatus_InvalidParameter);
    }

    return st;
}


ptxStatus_t ptxNSC_UARTSetDefaultSpeed(ptxNSC_t *nscCtx)
{
    ptxStatus_t st = ptxStatus_Success;

    ptxNSC_t *nsc_ctx = (NULL != nscCtx) ? (ptxNSC_t *)nscCtx : NULL;

    if (PTX_COMP_CHECK(nsc_ctx, ptxStatus_Comp_NSC) && (NscMode_HW == nsc_ctx->NscMode))
    {
        st = ptxPLAT_SetIntfSpeed(nsc_ctx->Plat, PTX_PLAT_HOST_DEFAULT_UART_SPEED);

    } else
    {
        st = PTX_STATUS(ptxStatus_Comp_NSC, ptxStatus_InvalidParameter);
    }

    return st;
}

