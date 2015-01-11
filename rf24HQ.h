/*-
 * Copyright (c) 2013 Darran Hunt (darran [at] hunt dot net dot nz)
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

#ifndef _RF24HQ_H_
#define _RF24HQ_H_

#include <Arduino.h>
#include <avr/pgmspace.h>

#include <Stream.h>
#include <Print.h>

/*
 * Registers
 */
#define RF24_CONFIG	 0x00
#define RF24_EN_AA	 0x01
#define RF24_EN_RXADDR	 0x02
#define RF24_SETUP_AW	 0x03
#define RF24_SETUP_RETR  0x04
#define RF24_RF_CH	 0x05
#define RF24_RF_SETUP	 0x06
#define RF24_STATUS	 0x07
#define RF24_OBSERVE_TX  0x08
#define RF24_CD		 0x09
#define RF24_RPD	 0x09
#define RF24_RX_ADDR_P0  0x0A
#define RF24_RX_ADDR_P1  0x0B
#define RF24_RX_ADDR_P2  0x0C
#define RF24_RX_ADDR_P3  0x0D
#define RF24_RX_ADDR_P4  0x0E
#define RF24_RX_ADDR_P5  0x0F
#define RF24_TX_ADDR	 0x10
#define RF24_RX_PW_P0	 0x11
#define RF24_RX_PW_P1	 0x12
#define RF24_RX_PW_P2	 0x13
#define RF24_RX_PW_P3	 0x14
#define RF24_RX_PW_P4	 0x15
#define RF24_RX_PW_P5	 0x16
#define RF24_FIFO_STATUS 0x17

/*
 * register fields
 */
#define RF24_MASK_RX_DR  (1<<6)
#define RF24_MASK_TX_DS  (1<<5)
#define RF24_MASK_MAX_RT (1<<4)

#define RF24_PRIM_RX     (1<<0)
#define RF24_PWR_UP      (1<<1)
#define RF24_CRCO        (1<<2)
#define RF24_EN_CRC      (1<<3)

#define RF24_ENAA_P0     (1<<0)
#define RF24_ENAA_P1     (1<<1)
#define RF24_ENAA_P2     (1<<2)
#define RF24_ENAA_P3     (1<<3)
#define RF24_ENAA_P4     (1<<4)
#define RF24_ENAA_P5     (1<<5)

#define RF24_ERX_P0      (1<<0)
#define RF24_ERX_P1      (1<<1)
#define RF24_ERX_P2      (1<<2)
#define RF24_ERX_P3      (1<<3)
#define RF24_ERX_P4      (1<<4)
#define RF24_ERX_P5      (1<<5)

#define RF24_AW          (1<<0)

#define RF24_ARD         4
#define RF24_ARC         0

#define RF24_RF_PWR      (1<<1)
#define RF24_RF_DR_HIGH  (1<<3)
#define RF24_PLL_LOCK    (1<<4)
#define RF24_RF_DR_LOW   (1<<5)
#define RF24_LNA_HCURR   (1<<0        )

#define RF24_TX_FULL     (1<<0)
#define RF24_RX_P_NO     1
#define RF24_MAX_RT      (1<<4)
#define RF24_TX_DS       (1<<5)
#define RF24_RX_DR       (1<<6)

#define RF24_PLOS_CNT    4
#define RF24_ARC_CNT     1
#define RF24_TX_REUSE    (1<<6)
#define RF24_FIFO_FULL   (1<<5)
#define RF24_TX_EMPTY    (1<<4)
#define RF24_RX_FULL     (1<<1)
#define RF24_RX_EMPTY    (1<<0)

#if 0
/*
 * Register values
 */

#define RF24_PWR_UP	0x02
#define RF24_PRIM_RX    0x01

#define RF24_EN_CRC     0x08
#define RF24_CRCO       0x04

#define RF24_MASK_RX_DR  0x40
#define RF24_MASK_TX_DS  0x20
#define RF24_MASK_MAX_RT 0x10

#define RF24_AW_MASK	0x03
#define RF24_ARD	4
#define RF24_ARD_MASK	0xF0
#define RF24_RX_DR	0x40
#define RF24_TX_DS	0x20
#define RF24_MAX_RT	0x10
#define RF24_RX_P_NO    1	// bit location
#define RF24_RX_P_NO_MASK 0x0E

#define RF24_PLOS_CNT		4
#define RF24_ARC_CNT_MASK	0x0f

#define RF24_TX_FULL	0x01
#define RF24_TX_REUSE   0x40
#define RF24_FIFO_FULL	0x20
#define RF24_TX_EMPTY	0x10
#define RF24_RX_FULL	0x02
#define RF24_RX_EMPTY	0x01
#endif

#define RF24_FEATURE	0x1d
#define RF24_EN_DYN_ACK	0x01	// enable dynamic ack (W_TX_PAYLOAD_NOACK)
#define RF24_EN_ACK_PAY	0x02	// enable ack payload
#define RF24_EN_DPL	0x04	// enable dynamic payload length

#define RF24_W_REGISTER    	0x20
#define RF24_R_RX_PL_WID	0x60	// read fifo size for top RX packet
#define RF24_R_RX_PAYLOAD  	0x61
#define RF24_W_TX_PAYLOAD  	0xA0
#define RF24_W_TX_PAYLOAD_NOACK	0xB0	// disables autoack on the next TX packet
#define RF24_FLUSH_TX      	0xE1
#define RF24_FLUSH_RX      	0xE2
#define RF24_REUSE_TX_PL  	0xE3

#if 0
#define RF24_CONFIG      0x00
#define RF24_EN_AA       0x01
#define RF24_RF_CH       0x05
#define RF24_RF_SETUP    0x06
#define RF24_RPD         0x09
#endif

#define RF24_PWRSTATE_MASK  (RF24_PWR_UP | RF24_PRIM_RX)

#define RF24_SPEED_250KBPS  (1<<5)
#define RF24_SPEED_1MBPS    0
#define RF24_SPEED_2MBPS    (1<<3)
#define RF24_SPEED_MASK     ((1<<5) | (1<<3))

#define RF24_POWER_0DBM        0x06
#define RF24_POWER_MINUS6DBM   0x04
#define RF24_POWER_MINUS12DBM  0x02
#define RF24_POWER_MINUS18DBM  0x00
#define RF24_POWER_MASK	       0x06

#define RF24_CRC_8		RF24_EN_CRC
#define RF24_CRC_16		(RF24_EN_CRC | RF24_CRCO)
#define RF24_CRC_OFF		0
#define RF24_CRC_MASK		(RF24_EN_CRC | RF24_CRCO)

#define RF24_ADDR_LEN 5
#define RF24_MAX_SIZE 32	/* Maximum message payload size */
#define RF24_NOACK	0x01	// disable ACK
#define RF24_BLOCK	0x02	// wait until message is sent and ACKed

class RFDebug : public Stream {
public:
    RFDebug();
    void begin(Stream *debugPrint);

    virtual size_t write(uint8_t byte);
    virtual int read() { return debug->read(); }
    virtual int available() { return debug->available(); }
    virtual void flush() { return debug->flush(); }
    virtual int peek() { return debug->peek(); }

    using Print::write;
private:
    Stream *debug;
};

class rf24 {
public:
    rf24(uint8_t cePin=8, uint8_t csnPin=9, uint8_t channel=80, uint8_t payload=RF24_MAX_SIZE);
    
    bool begin(Stream *debugPrint=NULL);
    bool begin(uint32_t dataRate, Stream *debugPrint);

    void setRxAddr(uint8_t id, const void *addr);
    void setTxAddr(const void *addr, bool noAck=false);
    char *getRxAddr(char *addr);
    char *getTxAddr(char *addr);
    void setPacketSize(uint8_t size);
    uint8_t getPacketSize();
    void setMaxChannel(uint8_t chan);
    void setChannel(uint8_t chan);
    uint8_t getChannel(void);
    void setCRC(uint8_t value);
    void setSpeed(uint8_t setting);
    void setPowerReg(uint8_t power);
    uint8_t getSpeed(void);
    uint8_t getTxPower(void);
    void setTxPower(int8_t dBm);

    void readReg(uint8_t reg, void *value, uint8_t size=1);
    uint8_t readReg(uint8_t reg);
    void writeReg(uint8_t reg);
    void writeReg(uint8_t reg, uint8_t value);
    void writeReg(uint8_t reg, const void *value, uint8_t size);
    void updateReg(uint8_t reg, uint8_t mask, uint8_t value);

    void flushRx(void);
    void flushTx(void);
    void enableRx(bool force=false);
    void enableTx(void);
    void powerDown(void);
    void powerUp(bool rx=false);
    void enableAck(uint8_t retry=5, bool enableAutoTxAddr=true, uint16_t delay=0);
    void disableAck(void);

    bool send(void *data, uint8_t size=RF24_MAX_SIZE, uint8_t flags=0, uint16_t timeout=100);
    void read(void *data, uint8_t size=RF24_MAX_SIZE);
    void resend(void);

    bool rxFifoAvailable(void);
    bool txFifoEmpty(void);
    bool available(void);
    bool available(uint32_t timeout);
    bool isSending(bool enableReceive=true);
    bool gotAck(void);
    bool txFull(void);
    uint8_t getTxRetries(void);
    uint8_t getTxLoss(bool clear=true);

    void scan(uint8_t *chans, uint8_t start=0, uint8_t count=125, uint8_t depth=128, const uint8_t ledPin=255);

    void dumpRegisters(void);

  private:
    RFDebug debug;	// debug print

    void setConfig(uint8_t value);
    uint16_t _scrubDelay(uint16_t delay);
    uint8_t _convertSpeedToReg(uint32_t rfspd);
    uint32_t _convertRegToSpeed(uint8_t rfspdreg);
    void chipEnable(void);
    void chipDisable(void);
    void chipPulse(void);

    uint8_t chipEnablePin;
    uint8_t chipSelectPin;
    bool chipEnabled;
    uint8_t channel;
    uint8_t packetSize;
    bool acked;
    bool autoAck;	/* Auto-ack feature enabled */
    bool autoTxAddr;
    uint8_t rfSpeed;

    uint8_t maxChan;	// Maximum supported channel
    uint8_t config;
};

#endif
