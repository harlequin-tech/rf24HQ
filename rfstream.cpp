/*
 * rf24HQ Serial interface
 * 
 */

#include <avr/pgmspace.h>

/* Work around a bug with PROGMEM and PSTR where the compiler always
 * generates warnings.
 */
#undef PROGMEM 
#define PROGMEM __attribute__(( section(".progmem.data") )) 
#undef PSTR 
#define PSTR(s) (__extension__({static prog_char __c[] PROGMEM = (s); &__c[0];})) 

#include <Print.h>
#include "rfstream.h"

#define SYNC_PERIOD	2000

#define MSG_DATA	0x7F
#define MSG_IDLE	0x80
#define MSG_RTS		0x81	/* Sender has data to send */
#define MSG_CTS		0x82	/* Sender ready to receive */

#define MODE_IDLE	0
#define MODE_TX		1
#define MODE_RX		2

#define MODE_TIMEOUT	1000	// XXX 100


RFStream::RFStream()
{
    rf = NULL;
    flushTime = 10;	/* Default 10 msecs */

    reset();
}

void RFStream::reset()
{
    if (rf) {
	rf->powerDown();
	rf->flushRx();
	rf->enableRx(true);
	rf->flushTx();
    }

    lastRxPacketId = -1;
    waitCTS = false;
    sentRTS = 0;

    mode = MODE_IDLE;

    txDataHead=0;
    txDataTail=0;
    rxDataHead=0;
    rxDataTail=0;

    sending=false;
    txSeqId = 0;
    rxSeqId = 0;
    lastRx = 0;
    lastTx = 0;
    lastWrite = 0;

    txAcks = 0;
    txRetries = 0;
    dropCount = 0;
    packetCount = 0;
}

void RFStream::begin(rf24 *rfp, Print *debugPrint)
{
    rf = rfp;
    debug.begin(debugPrint);
    reset();
}


uint8_t RFStream::rxDataSize()
{
    if (rxDataTail > rxDataHead) {
	return (sizeof(rxData) - rxDataTail + rxDataHead);
    } else {
	return rxDataHead - rxDataTail;
    }
}

uint8_t RFStream::rxDataSpace()
{
    if (rxDataTail > rxDataHead) {
	return sizeof(rxData) - (sizeof(rxData) - rxDataTail + rxDataHead) - 1;
    } else {
	return sizeof(rxData) - (rxDataHead - rxDataTail) - 1;
    }
}

int RFStream::read()
{
    if (rxDataSize() > 0) {
        uint8_t data = rxData[rxDataTail++];
	if (rxDataTail >= sizeof(rxData)) {
	    rxDataTail = 0;
	}

	return data;
    }

    return -1;
}

/* Flush received data */
void RFStream::flush()
{
    rxDataHead = rxDataTail = 0;
}

void RFStream::flushTx()
{
    txDataHead = txDataTail = 0;
}

uint8_t RFStream::txDataSize()
{
    if (txDataTail > txDataHead) {
	return (sizeof(txData) - txDataTail + txDataHead);
    } else {
	return txDataHead - txDataTail;
    }
}



uint8_t RFStream::txDataFill(uint8_t *data, uint8_t size)
{
    uint8_t avail = txDataSize();

    if (avail < size) size = avail;

    if (size < 1) return 0;

    if ((sizeof(txData) - txDataTail) >= size) {
	/* enough data for a packet, copy it */
	memcpy(data, &txData[txDataTail], size);
    } else {
	uint8_t part = sizeof(txData) - txDataTail;
	memcpy(data, &txData[txDataTail], part);
	memcpy(&data[part], txData, size - part);
    }

    txDataTail += size;

    if (txDataTail >= sizeof(txData)) {
	txDataTail -= sizeof(txData);
    }
#if 0
    Serial.print(F("tx."));
    Serial.print(txSeqId);
    Serial.print(F(" [0]=0x"));
    Serial.print(data[0],HEX);
    Serial.print(F(" ["));
    Serial.print(size-1);
    Serial.print(F("]=0x"));
    Serial.println(data[size-1],HEX);
#endif

    return size;
}

bool RFStream::txDataFull()
{
    return txDataSize() >= (sizeof(txData) - 1);
}

void RFStream::txDataSend()
{

    if (txDataSize() < 1) return;

    if (mode == MODE_TX) {
	txBuf.size = txDataFill(txBuf.data);
	if (txBuf.size) {
	    txBuf.id = (txSeqId++) & 0x7f;
	    sendPacket(&txBuf);
	    lastTx = millis();
	}
    } else if (mode == MODE_IDLE) {
        if ((millis() - sentRTS) > 100) {
	    txBuf.id = MSG_RTS;
	    sendPacket(&txBuf);
	    waitCTS = true;
	    sentRTS = millis();
	    Serial.println(F("sent RTS"));
	}
    }
}

void RFStream::rxDataStore(uint8_t *data, uint8_t size)
{
    if (size < 1) return;

    if (rxDataSpace() < size) {
	Serial.print(F("discarding "));
	Serial.print(size - rxDataSpace());
	Serial.println(F(" bytes"));

	rxDataTail += size - rxDataSpace();
	if (rxDataTail >= sizeof(rxData)) {
	    rxDataTail -= sizeof(rxData);
	}
    }

    if ((rxDataHead + size) < sizeof(rxData)) {
	memcpy(&rxData[rxDataHead], data, rxBuf.size);
    } else {
	uint8_t part = sizeof(rxData) - rxDataHead;
	memcpy(&rxData[rxDataHead], data, part);
	memcpy(rxData, &data[part], rxBuf.size - part);
    }

    rxDataHead += size;
    if (rxDataHead >= sizeof(rxData)) {
	rxDataHead -= sizeof(rxData);
    }
}

size_t RFStream::write(uint8_t data)
{
    uint8_t size = txDataSize();

    lastWrite = millis();

    txData[txDataHead++] = data;

    if (txDataHead >= sizeof(txData)) txDataHead = 0;

    if (size > (sizeof(txData)-1)) {
    	/* Drop oldest data */
	Serial.println(F("writeByte: dropped byte"));
	txDataTail++;
	if (txDataTail >= sizeof(txData)) txDataTail = 0;
	dropCount++;
    }

    if ((size+1) >= sizeof(txBuf.data)) {
        txDataSend();
    }
    return 1;
}

void RFStream::sendPacket(void *packet)
{
    rf->send(packet);
    sending = true;
}


void RFStream::sendMsg(void *packet, uint8_t size)
{
    rf->send(packet, size, true);
}

void RFStream::idle()
{
    uint32_t now = millis();

    if ((now - lastWrite) > flushTime) {
	txDataSend();
    }

    if (waitCTS && ((now - sentRTS) > 50)) {
        txBuf.id = MSG_RTS;
	sendMsg(&txBuf);
	sentRTS = millis();
	Serial.println(F("sent RTS"));
    }

    if (rf->available()) {
	rf->read(&rxBuf);
	lastRx = now;
	Serial.print(F("(IDLE): rx id 0x")); Serial.println(rxBuf.id,HEX);

	switch (rxBuf.id) {
	case MSG_IDLE:
	    Serial.println(F("IDLE.2"));
	    break;

	case MSG_CTS:
	    waitCTS = false;
	    mode = MODE_TX;
	    lastTx = now;
	    txSeqId = 0;
	    Serial.println(F("received CTS"));
	    break;

	case MSG_RTS:
	    txBuf.id = MSG_CTS;
	    sendMsg(&txBuf);
	    mode = MODE_RX;
	    lastRx = now;
	    rxSeqId = 0;
	    Serial.println(F("sending CTS"));
	    break;

	default:
	    Serial.print(F("rx: bad id 0x"));
	    Serial.println(rxBuf.id,HEX);
	    break;
	}
    }
}

/* Remote has data to send */
void RFStream::receive()
{
    uint32_t now = millis();

    if ((now - lastRx) > MODE_TIMEOUT) {
	Serial.println(F("receive: timeout"));
	mode = MODE_IDLE;
	Serial.println(F("IDLE"));
    }

    /* Expecting data from remote */
    if (rf->available()) {
	rf->read(&rxBuf);
	lastRx = now;

	if (rxBuf.id <= MSG_DATA) {
	    if (rxBuf.id != rxSeqId) {
		Serial.print(F("rx: seq "));
		Serial.print(rxBuf.id);
		Serial.print(F("rx: seq "));
		Serial.println(rxSeqId);
		rxSeqId = rxBuf.id;
	    }
	    rxSeqId = (rxSeqId + 1) & 0x7f;
	    packetCount++;
	    rxDataStore(rxBuf.data, rxBuf.size);
	} else {
	    switch (rxBuf.id) {
	    case MSG_IDLE:
		mode = MODE_IDLE;
		Serial.println(F("IDLE"));
		break;

	    case MSG_RTS:
		txBuf.id = MSG_CTS;
		sendMsg(&txBuf);
		mode = MODE_RX;
		lastRx = now;
		rxSeqId = 0;
		Serial.println(F("sending CTS"));
		break;

	    default:
		Serial.print(F("rx: bad id 0x"));
		Serial.println(rxBuf.id,HEX);
		break;
	    }
	}
    }
}

void RFStream::transmit()
{
    uint32_t now = millis();

    if ((now - lastTx) > MODE_TIMEOUT) {
	Serial.println(F("TX: timeout"));
	mode = MODE_IDLE;
	Serial.println(F("IDLE"));
    }

    if ((now - lastWrite) > flushTime) {
	txDataSend();
    }
}

void RFStream::loop() 
{
    if (sending) {
	if (!rf->isSending()) {
	    static uint8_t retry=0;
	    if (!rf->gotAck()) {
		Serial.print(F("tx: failed to send"));
		Serial.print(F(", CONFIG=0x"));
		Serial.print(rf->readReg(CONFIG),HEX);
		Serial.print(F(", STATUS=0x"));
		Serial.print(rf->readReg(STATUS),HEX);
		Serial.print(F(", OBSERVE_TX=0x"));
		Serial.print(rf->readReg(OBSERVE_TX),HEX);
		Serial.print(F(", FIFO_STATUS=0x"));
		Serial.println(rf->readReg(FIFO_STATUS),HEX);
		txRetries += rf->getTxRetries();
	    } else {
	        retry = 0;
		txRetries += rf->getTxRetries();
		txAcks++;
		sending = false;
	    }
	}
    }

    switch (mode) {
    case MODE_IDLE:
        idle();
	break;

    case MODE_RX:
        receive();
	break;

    case MODE_TX:
        transmit();
	break;

    default:
        Serial.print(F("invalid mode "));
	Serial.println(mode);
        mode = MODE_IDLE;
	break;
    }
}

int RFStream::available()
{
    static uint32_t debugTick=0;
    uint32_t now = millis();
    if ((now - debugTick) > 5000) {
	debug.print(F("available: mode 0x"));
	debug.print(mode,HEX);
	debug.print(F(" CONFIG=0x"));
	debug.print(rf->readReg(CONFIG),HEX);
	debug.print(F(", STATUS=0x"));
	debug.print(rf->readReg(STATUS),HEX);
	debug.print(F(", OBSERVE_TX=0x"));
	debug.print(rf->readReg(OBSERVE_TX),HEX);
	debug.print(F(", FIFO_STATUS=0x"));
	debug.println(rf->readReg(FIFO_STATUS),HEX);
	debugTick = now;
    }

    loop();

    return rxDataSize();
}

void RFStream::setFlushTime(uint16_t msecs)
{
    flushTime = msecs;
}

int RFStream::peek()
{
    if (rxDataSize() > 0) {
	return rxData[rxDataTail];
    } else {
	return -1;
    }
}
