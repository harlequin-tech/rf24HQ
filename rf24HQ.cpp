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

/* Release history
 *
 * Version  Date         Description
 * 0.1      22-Apr-2012  First release.
 *
 */

#include <Arduino.h>
#include <avr/pgmspace.h>
#include <SPI.h>
#include "rf24HQ.h"

#define CHIP_SELECT	LOW
#define CHIP_DESELECT	HIGH
#define CHIP_ENABLE	HIGH
#define CHIP_DISABLE	LOW
#define ENABLE_RX	true
#define ENABLE_TX	false
#define FORCE		true

#define TX_ACTIVE(config) ((config & RF24_PWRSTATE_MASK) == RF24_PWR_UP)
#define RX_ACTIVE(config) ((config & RF24_PWRSTATE_MASK) == (RF24_PWR_UP | RF24_PRIM_RX))


rf24::rf24(uint8_t cePin, uint8_t csnPin, uint8_t channelSet, uint8_t size)
{
    acked = false;
    autoAck = false;
    autoTxAddr = false;
    channel = channelSet;
    packetSize = size;
    chipEnablePin = cePin;
    chipSelectPin = csnPin;
    maxChan = 83;	// NZ, US, 2.483 GHz max allowed channel

    /* Default to interrupts masked, CRC16 enabled, powered down */
    config = RF24_MASK_RX_DR | RF24_MASK_TX_DS | RF24_MASK_MAX_RT | RF24_CRC_16;
}

void rf24::chipDisable()
{
    digitalWrite(chipEnablePin, CHIP_DISABLE);
    chipEnabled = false;
}

void rf24::chipEnable()
{
    digitalWrite(chipEnablePin, CHIP_ENABLE);
    chipEnabled = true;
}


void rf24::chipPulse()
{
    chipEnable();
    delayMicroseconds(10);
    chipDisable();
}

/**
* Initialize the nRF24L01+.
* @param dataRate the data rate to use. 250000, 1000000, or 2000000.
* @param debugPrint optional debug print stream.
*/
bool rf24::begin(uint32_t dataRate, Stream *debugPrint)
{
    debug.begin(debugPrint);

    pinMode(chipEnablePin,OUTPUT);
    pinMode(chipSelectPin,OUTPUT);

    chipDisable();
    digitalWrite(chipSelectPin, CHIP_DESELECT);

    SPI.begin();
    SPI.setDataMode(SPI_MODE0);
    SPI.setClockDivider(SPI_CLOCK_DIV2);

    acked = false;

    delay(100);		// 100ms power on reset
    powerDown();

    writeReg(RF24_CONFIG, config);
    setChannel(channel);
    setPacketSize(packetSize);
    setSpeed(RF24_SPEED_1MBPS);
    setTxPower(-6);
    enableAck(5);	// 5 msec max

    /* Make sure the module is working */
    if ((getChannel() != channel) || (readReg(RF24_RX_PW_P0) != packetSize)) {
	debug.println(F("rf24: failed to initialise"));
	return false;
    }

    writeReg(RF24_STATUS, RF24_RX_DR | RF24_TX_DS | RF24_MAX_RT); 

    // Start receiver 
    enableRx();

    // clear out any old packets 
    flushRx();
    flushTx();

    return true;
}

/**
* Initialize the nRF24L01+.
*/
bool rf24::begin(Stream *debugPrint)
{
    return begin(1000000, debugPrint);
}

uint8_t rf24::getPacketSize()
{
    return packetSize;
}

void rf24::setPacketSize(uint8_t size)
{
    for (uint8_t reg=RF24_RX_PW_P0; reg <= RF24_RX_PW_P5; reg++) {
	writeReg(reg, size);
    }
}

void rf24::setCRC(uint8_t value)
{
    config = (config & ~RF24_CRC_MASK) | (value & RF24_CRC_MASK);
    writeReg(RF24_CONFIG, config);
}

void rf24::setSpeed(uint8_t speed)
{
    rfSpeed = speed;
    updateReg(RF24_RF_SETUP, RF24_SPEED_MASK, speed);
}

uint8_t rf24::getSpeed()
{
    rfSpeed = readReg(RF24_RF_SETUP) & RF24_SPEED_MASK;
    return rfSpeed;
}


uint8_t rf24::getTxPower()
{
    return (readReg(RF24_RF_SETUP) & RF24_POWER_MASK);
}

/** Set the transmit power level
 * @param dBm power level. Valid values are -18, -12, -6, and 0. Setting will be rounded down to nearest. */
void rf24::setTxPower(int8_t dBm)
{
    if (dBm < -12) {
        dBm = RF24_POWER_MINUS18DBM;
    } else if (dBm <= -6) {
        dBm = RF24_POWER_MINUS12DBM;
    } else if (dBm <= 0) {
        dBm = RF24_POWER_MINUS6DBM;
    } else {
        dBm = RF24_POWER_0DBM;
    }
    updateReg(RF24_RF_SETUP, RF24_POWER_MASK, dBm);
}

/**
 * Power up the rf unit into standby I mode (CE low) or standby II mode (CE high)
 * @param rx - if true then enable rx, else enable tx
 * @note 26uA consumed when powered up and chip disabled (standby I mode)
 *       0.32 mA in standby II mode (chip enabled, tx mode, tx FIFO empty)
 *       7 to 11.3 mA when transmitting (depends on dBm setting)
 *       12.6 to 13.5 mA when receiving (depends on data rate)
 */
void rf24::powerUp(bool rx)
{
    bool powerChange = (config & RF24_PWR_UP) == 0;	// going from power down to power up?
    config = (config & ~RF24_PWRSTATE_MASK) | RF24_PWR_UP | (rx ? RF24_PRIM_RX : 0);
    writeReg(RF24_CONFIG, config);
    if (powerChange) {
	// allow time for crystal oscillator to start up
	delayMicroseconds(1500);
    }
}

/**
 * Put the rf unit into power-down mode
 * 0.9 uA current draw
 */
void rf24::powerDown()
{
    config &= ~RF24_PWRSTATE_MASK;
    writeReg(RF24_CONFIG, config);
}

/** Enable receive (disables transmit) */
void rf24::enableRx(bool force)
{
    if (force || !RX_ACTIVE(config) || !chipEnabled) {
	chipDisable();
	powerUp(ENABLE_RX);
	writeReg(RF24_STATUS, RF24_RX_DR | RF24_TX_DS | RF24_MAX_RT); 
	chipEnable();
	delayMicroseconds(130);   // Tstby2a - minimum delay ("RX settling")
    }
}

/** Enable transmit (disables receive) */
void rf24::enableTx()
{
    if (!TX_ACTIVE(config)) {
	powerUp(ENABLE_TX);
	delayMicroseconds(130);
    }
}

/** Flush the receive buffers. All received data is discarded */
void rf24::flushRx()
{
    writeReg(RF24_FLUSH_RX);
}

/** Flush the transmit buffers. All pending tx data is discarded */
void rf24::flushTx()
{
    writeReg(RF24_FLUSH_TX);
}


/** Set the receive address for a queue 
 * @param id The id of the queue to set the address for. Default
 *           receive queue is 1.
 * @param addr Pointer to the 5 byte address to set.
 * @ note first 4 bytes of addresses 2 through 5 must match address 1
 */
void rf24::setRxAddr(uint8_t id, const void *addr)
{
    if (id < 2) {
	writeReg(RF24_RX_ADDR_P0+id, (const uint8_t *)addr, RF24_ADDR_LEN);
    } else {
	writeReg(RF24_RX_ADDR_P0+id, ((uint8_t *)addr)[4]);
    }
    updateReg(RF24_EN_RXADDR, 1<<id, 1<<id);	// enable the receive pipe for the address
}

/** Set the transmit address. Also sets the receive address to the same 
 * to support the auto-ack feature.
 * @param addr pointer to the 5 byte address to set
 */
void rf24::setTxAddr(const void *addr, bool noAck)
{
    writeReg(RF24_TX_ADDR, (const uint8_t *)addr, RF24_ADDR_LEN);
    if (autoAck && autoTxAddr && !noAck) {
	/* 
	 * RX_ADDR_P0 is used for the auto ack feature, and 
	 * needs to be the same as the TX address 
	 */
	writeReg(RF24_RX_ADDR_P0, (const uint8_t *)addr, RF24_ADDR_LEN);
    }
}

char *rf24::getTxAddr(char *addr)
{
    readReg(RF24_TX_ADDR, addr, RF24_ADDR_LEN);
    addr[RF24_ADDR_LEN] = 0;
    return addr;
}

char *rf24::getRxAddr(char *addr)
{
    readReg(RF24_RX_ADDR_P1, addr, RF24_ADDR_LEN);
    addr[RF24_ADDR_LEN] = 0;
    return addr;
}

/** Read the value of a multi-byte register.
 * @param reg The register to read
 * @param value pointer to a buffer to store the value
 * @param size number of bytes to read
 */
void rf24::readReg(uint8_t reg, void *value, uint8_t size)
{
    digitalWrite(chipSelectPin, CHIP_SELECT);
    SPI.transfer(reg);
    do {
	size--;
	((uint8_t *)value)[size] = SPI.transfer(0);
    } while (size);
    digitalWrite(chipSelectPin, CHIP_DESELECT);
}

/** Read the value of byte register. */
uint8_t rf24::readReg(uint8_t reg)
{
    uint8_t data;
    readReg(reg, &data, 1);
    return data;
}

void rf24::updateReg(uint8_t reg, uint8_t mask, uint8_t value)
{
    writeReg(reg, (readReg(reg) & ~mask) | (value & mask));
}

/** Write a command */
void rf24::writeReg(uint8_t reg)
{
    writeReg(reg, NULL, 0);
}

/** Write a value to a single-byte register
 * @param reg The register to write
 * @param value the value to write
 */
void rf24::writeReg(uint8_t reg, uint8_t value)
{
    writeReg(reg, &value, 1);
}

/** Write a value to a multi-byte register
 * @param reg The register to write
 * @param value pointer to the value to write
 * @param size number of bytes to write
 */
void rf24::writeReg(uint8_t reg, const void *value, uint8_t size)
{
    if (size) {
	reg |= RF24_W_REGISTER;
    }
    digitalWrite(chipSelectPin, CHIP_SELECT);
    SPI.transfer(reg);
    if (size) {
	do {
	    size--;
	    SPI.transfer(((uint8_t *)value)[size]);
	} while (size);
    }
    digitalWrite(chipSelectPin, CHIP_DESELECT);
}

/** Check to see if the rf24 is sending a packet.
 * If enableReceive is true and the rf24 is not sending, enable the receiver.
 * @param enableReceive if true then enable the receiver if not sending.
 * @returns true if sending a packet, false otherwise.
 */
bool rf24::isSending(bool enableReceive)
{
    // see if TX is enabled
    if (!TX_ACTIVE(config)) {
	if (enableReceive) {
	    enableRx();
	}
        return false;
    }

    if (txFifoEmpty()) {
	acked = ((readReg(RF24_STATUS) & RF24_TX_DS) != 0);
	writeReg(RF24_STATUS, RF24_TX_DS | RF24_MAX_RT); 
	if (enableReceive) {
	    enableRx();
	}
	return false;
    } else {
	uint8_t status = readReg(RF24_STATUS);

	if (status & (RF24_TX_DS | RF24_MAX_RT)) {
	    acked = ((status & RF24_TX_DS) != 0);
	    writeReg(RF24_STATUS, RF24_TX_DS | RF24_MAX_RT); 
	    if (enableReceive) {
		enableRx();
	    }
#ifdef DEBUG
	    debug.print(F("isSending: txFifo not empty, acked="));
	    debug.print(acked);
	    debug.print(F(", CONFIG=0x"));
	    debug.print(readReg(RF24_CONFIG),HEX);
	    debug.print(F(", STATUS=0x"));
	    debug.print(readReg(RF24_STATUS),HEX);
	    debug.print(F(", OBSERVE_TX=0x"));
	    debug.print(readReg(RF24_OBSERVE_TX),HEX);
	    debug.print(F(", FIFO_STATUS=0x"));
	    debug.println(readReg(RF24_FIFO_STATUS),HEX);
#endif
	    return false;
	}
    }

    return true;
}

/** Check to see if an ACK was received for the last packet sent.
 * @returns true if an ACK was received, else false.
 */
bool rf24::gotAck()
{
    return acked;
}

/** Resend last failed transmission */
void rf24::resend()
{
    chipPulse();
}

uint8_t rf24::getTxRetries()
{
    return readReg(RF24_OBSERVE_TX) & 0xF;
}

uint8_t rf24::getTxLoss(bool clear)
{
    uint8_t loss = readReg(RF24_OBSERVE_TX) >> 4;
    if (clear) {
	writeReg(RF24_RF_CH, channel);	// reset loss count
    }

    return loss;
}

/** Send a packet
 * @param data pointer to the data to send
 * @param size number of bytes to send
 * @param flags bit-field used to adjust send behaviour
 * 		RF24_ACK wait for an ACK
 * 		RF24_BLOCK wait for send to complete
 * @param timeout time out send if blocking and no ack in this many milliseconds
 * @note the packet will always contain packetSize bytes, 
 *       if size > packetSize then the data is truncated,
 *       if size < packetSize then the packet is padded with 0.
 */
bool rf24::send(void *data, uint8_t size, uint8_t flags, uint16_t timeout) 
{
    uint8_t *dp = (uint8_t *)data;
    uint8_t pad = 0;

#if 0
    if (flags & RF24_NOACK) {
	debug.println(F("send: NOACK"));
    } else {
	debug.println(F("send: ACK"));
    }
#endif


    if (isSending(false)) {
	uint32_t start = millis();
	while (isSending(false)) {
	    if ((millis() - start) > timeout) {
		debug.println(F("rf24::send() timed out on initial isSending check"));
		dumpRegisters();
		return false;
	    }
	}

	debug.print(F("rf24::send() delayed "));
	debug.print(millis()-start);
	debug.println(F(" msecs due to isSending()"));
    }

    chipDisable();
    enableTx();

    pad = packetSize - size;
    if (pad > packetSize) {
	pad = 0;
    }


    digitalWrite(chipSelectPin, CHIP_SELECT);
    SPI.transfer(RF24_W_TX_PAYLOAD);
    while (size--) {
	SPI.transfer(*dp++);
    }
    while (pad--) {
	SPI.transfer(0);
    }
    digitalWrite(chipSelectPin, CHIP_DESELECT);

    if (autoAck && (flags & RF24_NOACK)) {
	// tell receiver not to ACK this packet
	writeReg(RF24_W_TX_PAYLOAD_NOACK);
    }

    chipPulse();
    acked = false;

    if (flags & RF24_BLOCK) {
	uint32_t start = millis();

	// Wait for send to complete
	while (isSending()) {
	    if ((millis() - start) > timeout) {
		debug.println(F("rf24::send() timed out"));
		return false;
	    }
	}

	if (autoAck && !(flags & RF24_NOACK)) {
	    return gotAck();
	} else {
	    return true;
	}
    }

    return true;
}


/** Read a packet
 *@param data Pointer to buffer to read packet into
 *@param size the size of the buffer, max amount of data to read
 */
void rf24::read(void *data, uint8_t size) 
{
    uint8_t *dp = (uint8_t *)data;
    digitalWrite(chipSelectPin, CHIP_SELECT);
    SPI.transfer(RF24_R_RX_PAYLOAD);
    while (size--) {
	*dp++ = SPI.transfer(0);
    }
    digitalWrite(chipSelectPin, CHIP_DESELECT);
    writeReg(RF24_STATUS, RF24_RX_DR);
}

/** See if the Rx FIFO has data available */
bool rf24::rxFifoAvailable()
{
    return (readReg(RF24_FIFO_STATUS) & RF24_RX_EMPTY) == 0;
}

bool rf24::txFifoEmpty()
{
    return (readReg(RF24_FIFO_STATUS) & RF24_TX_EMPTY) != 0;
}


/** Return true if there is a packet available to read */
bool rf24::available() 
{
    if (((readReg(RF24_STATUS) & RF24_RX_DR) != 0) || rxFifoAvailable()) {
	return true;
    } else {
	if (!RX_ACTIVE(config) || !chipEnabled) {
	    // not currently receiving
	    // check if transmitting still
	    if (isSending()) {
		return false;
	    }
	}
	return false;
    }
}

bool rf24::txFull()
{
    return (readReg(RF24_STATUS) & RF24_TX_FULL) != 0;
}

/** Return true if a packet becomes available to read within the time
 * specified by timeout.
 * @param timeout The number of milliseconds to wait for a packet
 * @returns true if packet is available to read, false if not
 */
bool rf24::available(uint32_t timeout) 
{
    uint32_t start = millis();

    do {
	if (available()) {
	    return true;
	}
    } while ((start - millis()) < timeout);

    return false;
}

/** Set the RF channel. Sets the tx and rx frequency to 
 * 2.400 GHz + chan MHz.
 * @param chan The RF channel, 0 through maxChan (device can support up to 125).
 */
void rf24::setChannel(uint8_t chan)
{
    if (chan > maxChan) {
	chan = maxChan;
    }
    writeReg(RF24_RF_CH, chan);
    channel = chan;
}

void rf24::setMaxChannel(uint8_t chan)
{
    if (chan > 125) {
	maxChan = 125;
    } else {
	maxChan = chan;
    }
}

uint8_t rf24::getChannel(void)
{
    channel = readReg(RF24_RF_CH);
    return channel;
}


/** Enable the auto-ack feature to improve packet reliability. With this
 * feature enabled the rf24 will wait for an ACK from the receiving unit,
 * and it will resend the packet if it does not receive one.
 * @param retry How many times to retransmit a packet (max is 15).
 * @param delay How long to wait for an ACK (in microseconds). 0 = default
 * @note the delay resolution is 250 microseconds, values are rounded up
 *       to the nearest multiple of 250. Max is 4000 microseconds.
 */
void rf24::enableAck(uint8_t retry, bool enableAutoTxAddr, uint16_t delay)
{
    uint8_t addr[RF24_ADDR_LEN];

    // set appropriate delay
    // Note: payload ACKs are not yet implemented,
    // so the ACK payload size is always 0 bytes.
    if (delay == 0) {
	if (rfSpeed == RF24_SPEED_250KBPS) {
	    delay = 500; // 500us
	} else {
	    delay = 250; // 250us
	}
    }

    writeReg(RF24_EN_AA, 0x3F); /* enable auto-ack */
    // delay is rounded up to nearest 250 microseconds
    writeReg(RF24_SETUP_RETR, (uint8_t)(((delay-1) / 250) & 0x0F) << 4 | (retry & 0x0F));
    writeReg(RF24_FEATURE, RF24_EN_DYN_ACK);

    autoTxAddr = enableAutoTxAddr;
    /* 
     * RX_ADDR_P0 is used for the auto ack feature, and 
     * needs to be the same as the TX address 
     */
    if (autoTxAddr) {
	readReg(RF24_TX_ADDR, addr, RF24_ADDR_LEN);
	writeReg(RF24_RX_ADDR_P0, addr, RF24_ADDR_LEN);
    }
    autoAck = true;
}

/** Disable the auto-ack feature */
void rf24::disableAck()
{
    writeReg(RF24_EN_AA, 0);
    autoAck = false;
    autoTxAddr = false;
}

/**
 * Scan for active channels. Fills in the chans array with an estimate of
 * received signal strength for each channel.
 * The estimate is a count of how many times a >-64dBm signal was seen on the channel.
 *
 * Based on Rolf Henkel's concept and work.
 * http://arduino.cc/forum/index.php/topic,54795.0.html
 *
 * @param chans - channel array to fill with signal estimate
 * @param start - start scanning from this channel
 * @param count - scan this many channels
 * @param depth - number of samples for each channel
 */
void rf24::scan(uint8_t *chans, uint8_t start, uint8_t count, uint8_t depth, const uint8_t ledPin)
{
    uint8_t end = start+count;
    uint8_t configuredChan = getChannel();
    bool led=false;

    if (end > 125) {
	end = 125;
    }

    chipDisable();
    powerDown();

    memset(chans, 0, count);
    enableRx();

    for (uint8_t rep=0; rep<depth; rep++) {
	if (ledPin < 255) {
	    digitalWrite(ledPin, led);
	    led = !led;
	}
        for (uint8_t chan=start; chan < end; chan++) {
	    setChannel(chan);
	    enableRx();
	    delayMicroseconds(40);
	    chipDisable();

	    if (readReg(RF24_RPD) & 0x01) {
	        chans[chan]++;
	    }
	}
    }

    setChannel(configuredChan);
}

/* Register addresses, sizes, and names for dump */
static const struct {
    uint8_t reg;
    uint8_t size;
    char name[12];
} regs[] __attribute__((__progmem__)) = {
    { RF24_CONFIG,      1, "CONFIG" },
    { RF24_EN_AA,       1, "EN_AA" },
    { RF24_EN_RXADDR,   1, "EN_RXADDR" },
    { RF24_SETUP_AW,    1, "SETUP_AW" },
    { RF24_SETUP_RETR,  1, "SETUP_RETR" },
    { RF24_RF_CH,       1, "RF_CH" },
    { RF24_RF_SETUP,    1, "RF_SETUP" },
    { RF24_STATUS,      1, "STATUS" },
    { RF24_OBSERVE_TX,  1, "OBSERVE_TX" },
    { RF24_RPD,         1, "RPD" },
    { RF24_RX_ADDR_P0,  5, "RX_ADDR_P0" },
    { RF24_RX_ADDR_P1,  5, "RX_ADDR_P1" },
    { RF24_RX_ADDR_P2,  1, "RX_ADDR_P2" },
    { RF24_RX_ADDR_P3,  1, "RX_ADDR_P3" },
    { RF24_RX_ADDR_P4,  1, "RX_ADDR_P4" },
    { RF24_RX_ADDR_P5,  1, "RX_ADDR_P5" },
    { RF24_TX_ADDR,     5, "TX_ADDR" },
    { RF24_RX_PW_P0,    1, "RX_PW_P0" },
    { RF24_RX_PW_P1,    1, "RX_PW_P1" },
    { RF24_RX_PW_P2,    1, "RX_PW_P2" },
    { RF24_RX_PW_P3,    1, "RX_PW_P3" },
    { RF24_RX_PW_P4,    1, "RX_PW_P4" },
    { RF24_RX_PW_P5,    1, "RX_PW_P5" },
    { RF24_FIFO_STATUS, 1, "FIFO_STATUS" }
};

void rf24::dumpRegisters(void)
{
    uint8_t ind;
    uint8_t data[5];
    uint8_t dind;

    for (ind=0; ind<(sizeof(regs)/sizeof(regs[0])); ind++) {
	uint8_t reg = pgm_read_byte(&regs[ind].reg);
	uint8_t size = pgm_read_byte(&regs[ind].size);
	debug.print((const __FlashStringHelper *)regs[ind].name);
	/* Pad the output to 12 spaces */
	for (dind=12-strlen_P(regs[ind].name); dind; dind--) {
	    debug.print(' ');
	}
	debug.print('(');
	if (reg < 0x10)
	    debug.print('0');
	debug.print(reg,HEX);
	debug.print(F("): "));
	readReg(reg, data, size);

	for (dind=0; dind<size; dind++) {
	    if (data[dind] < 0x10)
		debug.print('0');
	    debug.print(data[dind],HEX);
	    debug.print(' ');
	}

	if ((reg >= RF24_RX_ADDR_P0) && (reg <= RF24_TX_ADDR)) {
	    debug.print(' ');
	    for (dind=0; dind<size; dind++) {
		if (isprint(data[dind])) {
		    debug.print((char)data[dind]);
		} else {
		    debug.print('.');
		}
	    }
	}
	debug.println();
    }
}

RFDebug::RFDebug()
{
    debug = NULL;
}

void RFDebug::begin(Stream *debugPrint)
{
    debug = debugPrint;
}

size_t RFDebug::write(uint8_t data)
{
    if (debug != NULL) {
	return debug->write(data);
    }

    return 0;
}

