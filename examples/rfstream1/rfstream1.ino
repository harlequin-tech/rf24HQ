/*
 * rfstream1.ino rf24HQ stream demo.
 *
 * Sends all characters received from the serial monitor to the partner radio.
 * Sends all characters received from the partner radio to the serial monitor.
 *
 * Run rfstream2.ino on the partner radio to connect to this sketch.
 *
 * Released to the public domain.
 *
 */

/*
 * The nrf24l01+ module should be connected to as follows:
 *
 *    rf24    atmega328
 *    ----    ---------
 *    GND  -> GND
 *    VCC  -> 3.3V
 *    CE   -> digital pin 8
 *    CSN  -> digital pin 9
 *    SCK  -> digital pin 13
 *    MOSI -> digital pin 11
 *    MISO -> digital pin 12
 */

#include <SPI.h>
#include <rf24HQ.h>

rf24 rf(8,9,100,RF24_MAX_SIZE);
RFStream rfstream;

const char myAddress[] = "rad01";			/* The rx address for this radio */
const char txAddress[] = "rad02";			/* Send data to this address */

void setup() 
{
    Serial.begin(115200);
    rf.begin((Print *)&Serial);
    rfstream.begin(&rf, (Print *)&Serial);

    rf.setRxAddr(1, myAddress);
    rf.setTxAddr(txAddress);

    /* Turn on auto-ack, 1000 microsecond timeouts, 15 retries */
    rf.enableAck(1000, 15);

    Serial.print(myAddress);
    Serial.println(" ready");

    rfstream.print(myAddress);
    rfstream.println(F(" ready"));
}

void loop() 
{
    if (rfstream.available()) {
        Serial.write(rfstream.read());
    }
    if (Serial.available()) {
        char ch = Serial.read();
        rfstream.write(ch);
	Serial.write(ch);
    }
}
