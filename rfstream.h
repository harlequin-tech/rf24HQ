/*-
 * Copyright (c) 2012 Darran Hunt (darran [at] hunt dot net dot nz)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL
 * THE CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _RFSTREAM_H_
#define _RFSTREAM_H_

#include <Arduino.h>
#include <avr/pgmspace.h>
#include <Stream.h>
#include <Print.h>
#include "rf24HQ.h"

#define RFSTREAM_SIZE (RF24_MAX_SIZE-2)

#define RFS_SERIAL_BUF_SIZE 128

#define RFS_MAX_ID		0x7F	/* Maxiumum message sequence ID */
#define RFS_SYNC_PERIOD		5000	/* period for sending keep-alive millisecond during idle */
#define RFS_RX_BUF_COUNT	2	/* Number of receive buffers */

typedef struct {
    uint8_t size;
    uint8_t id;
    uint8_t data[RFSTREAM_SIZE];
} rfstream_buf_t;

class RFStream : public Stream {
public:
    RFStream();
    void begin(rf24 *rf, Print *debugPrint=NULL);
    void reset();
    virtual size_t write(uint8_t byte);
    virtual int read();
    virtual int available();
    virtual void flush();
    virtual int peek();
    void setFlushTime(uint16_t msecs);
    using Print::write;
    rf24 *rf;

    uint32_t packetCount;
    uint8_t lastRxPacketId;
    uint32_t dropCount;

private:
    void idle();
    void transmit();
    void receive();
    void sender();
    void loop();

    uint8_t txDataFill(uint8_t *data, uint8_t size=RFSTREAM_SIZE);
    void txDataSend();
    void rxDataStore(uint8_t *data, uint8_t size=RFSTREAM_SIZE);
    void sendPacket(void *packet);
    void sendMsg(void *packet, uint8_t size=RF24_MAX_SIZE);
    void flushTx();

    uint8_t txDataSize();
    uint8_t rxDataSize();
    uint8_t rxDataSpace();
    bool txDataFull();

    RFDebug debug;
    rfstream_buf_t txBuf;
    rfstream_buf_t rxBuf;

    uint8_t rxData[RFS_SERIAL_BUF_SIZE];
    uint8_t rxDataHead;
    uint8_t rxDataTail;
    uint8_t txData[RFS_SERIAL_BUF_SIZE];
    uint8_t txDataHead;
    uint8_t txDataTail;

    bool waitCTS;
    uint32_t sentRTS;
    uint8_t mode;

    bool sending;
    uint8_t txSeqId;
    uint8_t rxSeqId;
    uint32_t lastRx;
    uint32_t lastTx;
    uint32_t lastWrite;

    uint32_t txAcks;
    uint32_t txRetries;

    uint16_t flushTime;
};
#endif
