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
    File        : PtxPlatUART.cpp

    Description : UART interface implementation
*/

#include "PtxPlatUART.h"

#include <Arduino.h>

#include <algorithm>

#include "ptxPLAT_INT.h"

namespace {
const uint8_t pinTXD = D11;
const uint8_t pinRXD = D12;
const uint8_t pinRTS = D10;
const uint8_t pinCTS = D13;
// time to wait for immediate response from PTX chip
const uint32_t readTimeout = 100U;
// uart instance to be used
UART Uart(pinTXD, pinRXD, pinRTS, pinCTS);
// uart context object
ptxPLAT_UART_t platUart;

ptxPLAT_UARTState_t checkRx();
}  // namespace

ptxStatus_t ptxPlat_uartInit(ptxPLAT_UART_t **uart,
                             ptxPLAT_ConfigPars_t *config) {
  auto status = ptxStatus_Success;

  if (uart != nullptr) {
    memset(&platUart, 0, sizeof(platUart));
    platUart.IntfSpeed = config->Speed;
    *uart = &platUart;
    Uart.begin(platUart.IntfSpeed);
  } else {
    status = PTX_STATUS(ptxStatus_Comp_PLAT, ptxStatus_InvalidParameter);
  }

  return status;
}

ptxStatus_t ptxPlat_uartDeinit() {
  memset(&platUart, 0, sizeof(platUart));
  Uart.end();

  return ptxStatus_Success;
}

ptxStatus_t ptxPlat_uartTrx(ptxPLAT_UART_t *uart, const uint8_t *txBuf[],
                            size_t txLen[], size_t numTxBuffers,
                            uint8_t *rxBuf[], size_t *rxLen[],
                            size_t numRxBuffers) {
  const size_t numBuffers_max = 5;

  ptxStatus_t status = ptxPLAT_uartSetCleanStateRx(uart);

  if (status == ptxStatus_Success && numTxBuffers < numBuffers_max &&
      numRxBuffers <= numBuffers_max) {
    /* At this point the UART transfer operation is triggered */
    /* Tx operation is required always: to send and to receive anything. */
    if (txBuf && txLen) {
      /* Tx part of the overall transaction. */
      uint8_t index = 0;
      while (index < numTxBuffers) {
        if (txBuf[index] && txLen[index] > 0) {
          Uart.write(txBuf[index], txLen[index]);
        } else {
          status = PTX_STATUS(ptxStatus_Comp_PLAT, ptxStatus_InvalidParameter);
          break;
        }
        index++;
      }
    }

    /* Let's see if there is something to read. */
    if (status == ptxStatus_Success && rxBuf && rxLen) {
      uint8_t index = 0;
      while (status == ptxStatus_Success && index < numRxBuffers) {
        if (rxBuf[index] && rxLen[index] && *rxLen[index] > 0) {
          uint8_t *readPtr = rxBuf[index];
          size_t remaining = *rxLen[index];
          const uint32_t waitStart = millis();

          while (status == ptxStatus_Success && remaining) {
            yield();
            int recvd = Uart.available();
            while (recvd--) {
              *readPtr++ = Uart.read();
              remaining--;
              if (!remaining) break;
            }
            if (millis() - waitStart > readTimeout)
              status = PTX_STATUS(ptxStatus_Comp_PLAT, ptxStatus_TimeOut);
          }
        } else {
          status = PTX_STATUS(ptxStatus_Comp_PLAT, ptxStatus_InvalidParameter);
        }
        index++;
      }
    }
  } else {
    status = PTX_STATUS(ptxStatus_Comp_PLAT, ptxStatus_InvalidParameter);
  }

  return status;
}

ptxStatus_t ptxPLAT_uartSetCleanStateRx(ptxPLAT_UART_t *uart) {
  auto status = ptxStatus_Success;

  if (uart != nullptr) {
    memset(uart->RxBuf, 0, sizeof(uart->RxBuf));
    memset(&uart->RxCtrl, 0, sizeof(uart->RxCtrl));
    while (Uart.available()) {
      /* clean receive buffer */
      Uart.read();
    }
  } else {
    status = PTX_STATUS(ptxStatus_Comp_PLAT, ptxStatus_InvalidParameter);
  }

  return status;
}

ptxStatus_t ptxPLAT_uartSetIntfSpeed(ptxPLAT_UART_t *uart, uint32_t speed) {
  if (uart != nullptr) {
    Uart.end();
    uart->IntfSpeed = speed;
    Uart.begin(speed);
    return ptxStatus_Success;
  } else {
    return PTX_STATUS(ptxStatus_Comp_PLAT, ptxStatus_InvalidParameter);
  }
}

ptxStatus_t ptxPlat_uartTriggerRx(ptxPlat_t *plat) {
  if (plat != nullptr && plat->Uart != nullptr) {
    if (checkRx() == PTX_PLAT_UART_StateDONE) {
      if (plat->RxCb) {
        plat->RxCb(plat->CtxRxCb);
      }

      auto nextMsgIdx =
          static_cast<uint16_t>(plat->Uart->RxBuf[plat->Uart->RxCtrl.MsgIndex] +
                                plat->Uart->RxCtrl.MsgIndex + 1);
      plat->Uart->RxCtrl.MsgIndex =
          static_cast<uint16_t>((nextMsgIdx >= PTX_PLAT_RXBUF_SIZE)
                                    ? (nextMsgIdx - PTX_PLAT_RXBUF_SIZE)
                                    : nextMsgIdx);
    }
    return ptxStatus_Success;
  } else {
    return PTX_STATUS(ptxStatus_Comp_PLAT, ptxStatus_InvalidParameter);
  }
}

uint8_t ptxPlat_uartIsRxPending() {
  return static_cast<uint8_t>(checkRx() != PTX_PLAT_UART_StateIDLE);
}

uint8_t ptxPlat_uartCheckRxActive() {
  return static_cast<uint8_t>(checkRx() == PTX_PLAT_UART_StateONGOING);
}

ptxStatus_t ptxPLAT_uartGetReceivedMessage(ptxPLAT_UART_t *uart,
                                           uint8_t *rxMessageBuffer,
                                           size_t *rxMessageBufferLen) {
  auto status = ptxStatus_Success;

  if (uart != nullptr && rxMessageBuffer != nullptr &&
      rxMessageBufferLen != nullptr) {
    uint8_t *rxBuffer = uart->RxBuf;

    /** Prepare received message. */
    const auto msgIndex = uart->RxCtrl.MsgIndex;
    const auto nextMsgIdx =
        static_cast<uint16_t>(rxBuffer[msgIndex] + msgIndex + 1U);
    const bool fifoOverflow = nextMsgIdx >= PTX_PLAT_RXBUF_SIZE ? true : false;
    const auto len = rxBuffer[msgIndex];

    if (fifoOverflow && len > 0) {
      /** In case of an overflow, bytes must be copied manually */
      uint16_t idx;

      for (uint8_t i = 0U; i < len; i++) {
        idx = static_cast<uint16_t>(msgIndex + 1U + i);
        if (idx >= PTX_PLAT_RXBUF_SIZE) {
          idx = static_cast<uint16_t>(idx - PTX_PLAT_RXBUF_SIZE);
        }
        rxMessageBuffer[i] = rxBuffer[idx];
      }
    } else {
      /** In case of no overflow, copy the received message at once */
      std::copy_n(&rxBuffer[msgIndex + 1U], len, rxMessageBuffer);
    }
    *rxMessageBufferLen = static_cast<size_t>(len);
  } else {
    status = PTX_STATUS(ptxStatus_Comp_PLAT, ptxStatus_InvalidParameter);
  }

  return status;
}

namespace {
ptxPLAT_UARTState_t checkRx() {
  uint8_t *fifoBuff = platUart.RxBuf;

  /* read-in bytes from uart rx buffer */
  while (Uart.available()) {
    fifoBuff[platUart.RxCtrl.Idx++] = Uart.read();
    if (platUart.RxCtrl.Idx >= PTX_PLAT_RXBUF_SIZE) platUart.RxCtrl.Idx = 0;
  }

  uint16_t fifoIndex = platUart.RxCtrl.Idx;
  uint16_t msgIndex = platUart.RxCtrl.MsgIndex;
  ptxPLAT_UARTState_t state = PTX_PLAT_UART_StateIDLE;
  if (msgIndex != fifoIndex) {
    auto newMsgIndex = static_cast<uint16_t>(fifoBuff[msgIndex] + msgIndex + 1);

    state = PTX_PLAT_UART_StateONGOING;

    if (0 == fifoBuff[msgIndex]) {
      state = PTX_PLAT_UART_StateDONE;
    } else if (newMsgIndex >= PTX_PLAT_RXBUF_SIZE) {
      /** Back to the beginning of the FIFO. */
      newMsgIndex = static_cast<uint16_t>(newMsgIndex - PTX_PLAT_RXBUF_SIZE);

      if (fifoIndex >= newMsgIndex) {
        state = PTX_PLAT_UART_StateDONE;
      }
    } else if (fifoIndex >= newMsgIndex) {
      state = PTX_PLAT_UART_StateDONE;
    }
  }

  return state;
}
}  // namespace
