/*
 * Sketch to transfer data between the serial port and an RFM12B. This
 * sketch can be used to emulate an XBee or Ciseco XRF radio link.
 * You will need to use a pair RFM12B modules with similar software at
 * both ends of the link. The firmware size is under 8kB when compiled
 * for the ATtiny84 so it may be possible to run the sketch on the
 * RFM12B to Pi expansion board,
 * http://shop.openenergymonitor.com/raspberry-pi/.
 *
 * Copyright S R Marple, 2013.
 * Released under MIT License, http://opensource.org/licenses/mit-license.php
 */


#if defined(UBRRH) || defined(UBRR0H) || defined(USBCON)
Stream &mySerial = Serial;
#else
// No hardware serial or USB interface
#include <SoftwareSerial.h>
#endif

//#include <MultiReadCircBuffer.h>
#include <CircBuffer.h>
#include <AsyncDelay.h>
//#include <SPI.h>
#include <RF12.h>
#include <RF12_Stream.h>

#ifdef CALUNIUM
// Calunium, use pin mapping for v2.0
#define RFM12B_CS 14
#define RFM12B_IRQ_PIN 6
#define RFM12B_IRQ_NUM 2

#elif defined(__AVR_ATmega168__) || defined(__AVR_ATmega328__) || defined (__AVR_ATmega328P__)
// Jeenode mapping?
#define RFM12B_CS 10
#define RFM12B_IRQ_PIN 2
#define RFM12B_IRQ_NUM 0

#elif defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny44__)
// Assume pin-mapping used for Martin Harizanov's RFM2Pi board
#define RFM12B_CS 1
#define RFM12B_IRQ_PIN 2
#define RFM12B_IRQ_NUM 0

SoftwareSerial swSerial(7, 3);
Stream &mySerial = swSerial;

#endif

#if RAMEND >= 4096
// eg ATmega644(P), ATmega1284(P), ATmega1280 or ATmega2560
uint8_t rxBuffer[1024];
uint8_t txBuffer[1024];

#elif RAMEND >= 2048
// eg ATmega328
uint8_t rxBuffer[1024];
uint8_t txBuffer[1024];

#else
// eg ATtiny84
uint8_t rxBuffer[60];
uint8_t txBuffer[60];
RF12_Stream rfm12b(rxBuffer, sizeof(rxBuffer),
		   txBuffer, sizeof(txBuffer));
#endif

unsigned long sendInterval_ms = 2000UL;

AsyncDelay sendDelay;

void setup(void)
{

#ifdef SS
  pinMode(SS, OUTPUT);
#endif

#if defined(UBRRH) || defined(UBRR0H) || defined(USBCON)
  Serial.begin(9600);
#else
  swSerial.begin(9600);
#endif

  if (rfm12b.begin(RFM12B_CS, RFM12B_IRQ_PIN, RFM12B_IRQ_NUM,
		   1, RF12_433MHZ)) {
    sendDelay.start(sendInterval_ms, AsyncDelay::MILLIS);
  }
  else 
    while (1)
      mySerial.println("RFM12B not found");
  
}

void loop(void)
{

  rfm12b.poll();
  if (sendDelay.isExpired()) {
    long myMillis = millis();

    // Serial << "TX: Millis is " << myMillis << endl;
    //rfm12b << "Millis is " << myMillis << endl;
    // sendDelay.start(sendInterval_ms, AsyncDelay::MILLIS);
  }

  // TODO: wait for a while for RFM12B to catch up
  while (mySerial.available() && rfm12b.getRxBuffer().getSizeRemaining())
    rfm12b.write(char(mySerial.read()));
  
  while (rfm12b.available())
    mySerial.write(char(rfm12b.read()));

  /*
  if (rfm12b.available()) {
    Serial << "RX: ";
    while (rfm12b.available())
      Serial << char(rfm12b.read());
    Serial << endl;
  }
  */

}

