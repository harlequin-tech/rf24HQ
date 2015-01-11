/*
 * rfscan.ino rf24HQ Channel scanner example sketch.
 * 
 * Demonstrates the "Poor Man's 2.4GHz Scanner".
 *
 * Based on Rolf Henkel's concept and work.
 * http://arduino.cc/forum/index.php/topic,54795.0.html
 *
 * This sketch is released into the public domain.
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

#define CHANNELS  125
uint8_t channel[CHANNELS];

rf24 rf(8,9,100,RF24_MAX_SIZE);

void setup() 
{
    Serial.begin(115200);

    rf.begin(&Serial);

    rf.setTxAddr("test1");
    rf.setRxAddr(1, "serv1");

    rf.dumpRegisters();
    Serial.println("Wireless initialized.");
}

void loop() 
{
    uint8_t chan;

    Serial.println("Scanning");
    rf.scan(channel, 0, sizeof(channel), 200);

    Serial.println("Results:");
    /* Draw a bar graph of the approximate signal strength for each channel, 
     * skipping channels with no signal.
     */
    for (chan=0; chan<sizeof(channel); chan++) {
        if (channel[chan] != 0) {
	    Serial.print(2400+chan);
	    Serial.print("MHz = ");
	    if (channel[chan] < 100) Serial.print(' ');
	    if (channel[chan] < 10) Serial.print(' ');
	    Serial.print(channel[chan]);
	    Serial.print(' ');
	    for (int level=0; level < channel[chan]; level++) {
	        Serial.print("-");
	    }
	    Serial.println();
	}
    }
}
