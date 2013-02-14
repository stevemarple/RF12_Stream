/*
 * Sketch to demonstrate RF12_Stream. Prints incoming
 * data. Periodically sends data and prints statistics about the link
 * (number of packets sent and received, and number of retries).  The
 * firmware size is under 8kB when compiled for the ATtiny84 so it may
 * be possible to run the sketch on the RFM12B to Pi expansion board,
 * http://shop.openenergymonitor.com/raspberry-pi/.
 *
 * Copyright S R Marple, 2013.
 * Released under MIT License, http://opensource.org/licenses/mit-license.php
 */

#include <CircBuffer.h>
#include <AsyncDelay.h>
#include <RF12.h>
#include <RF12_Stream.h>



#if defined(UBRRH) || defined(UBRR0H) || defined(USBCON)
Stream &mySerial = Serial;
#else
// No hardware serial or USB interface
#define USE_SW_SERIAL
#include <SoftwareSerial.h>
#endif


#ifdef CALUNIUM
// Calunium, use pin mapping for v2.0/v2.1
#define RFM12B_CS 14
#define RFM12B_IRQ_PIN 6
#define RFM12B_IRQ_NUM 2
#define LED_PIN LED_BUILTIN

#elif defined(__AVR_ATmega168__) || defined(__AVR_ATmega328__) || defined (__AVR_ATmega328P__)
// Jeenode mapping?
#define RFM12B_CS 10
#define RFM12B_IRQ_PIN 2
#define RFM12B_IRQ_NUM 0
#define LED_PIN LED_BUILTIN

#elif defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny44__)
// Assume pin-mapping used for Martin Harizanov's RFM2Pi board
#define RFM12B_CS 1
#define RFM12B_IRQ_PIN 2
#define RFM12B_IRQ_NUM 0
#define LED_PIN 8
#endif

#ifdef USE_SW_SERIAL
SoftwareSerial swSerial(7, 3);
Stream &mySerial = swSerial;
#endif

#if RAMEND >= 4096
// eg ATmega644(P), ATmega1284(P), ATmega1280 or ATmega2560
uint8_t rxBuffer[1024];
uint8_t txBuffer[1024];

#elif RAMEND >= 2048
// eg ATmega328
uint8_t rxBuffer[512];
uint8_t txBuffer[512];

#else
// eg ATtiny84
uint8_t rxBuffer[30];
uint8_t txBuffer[30];
#endif

RF12_Stream rfm12b(rxBuffer, sizeof(rxBuffer),
		   txBuffer, sizeof(txBuffer));


unsigned long sendInterval_ms = 2000UL;
unsigned long statsInterval_ms = 20000UL;

AsyncDelay sendDelay;
AsyncDelay statsDelay;
#ifdef LED_PIN
AsyncDelay activityDelay;
#endif

void setup(void)
{
#ifdef SS
  pinMode(SS, OUTPUT);
#endif

#ifdef LED_PIN
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
#endif

#ifdef USE_SW_SERIAL
  swSerial.begin(9600);
#else
  Serial.begin(9600);
#endif

  if (rfm12b.begin(RFM12B_CS, RFM12B_IRQ_PIN, RFM12B_IRQ_NUM, 1, RF12_433MHZ)) {
    sendDelay.start(sendInterval_ms, AsyncDelay::MILLIS);
  }
  else 
    while (1)
      mySerial.println("RFM12B not found");
  

#ifdef LED_PIN
  digitalWrite(LED_PIN, LOW);
#endif
  activityDelay.expire();
}

void loop(void)
{
  bool activity = false;
  
  rfm12b.poll();
  if (sendDelay.isExpired()) {
    long myMillis = millis();

    mySerial.print("TX: Millis is ");
    mySerial.println(myMillis);
    rfm12b.print("Millis is ");
    rfm12b.println(myMillis);
    sendDelay.start(sendInterval_ms, AsyncDelay::MILLIS);
  }

  if (rfm12b.available()) {
    mySerial.print("RX: ");
    while (rfm12b.available())
      mySerial.write(char(rfm12b.read()));
    mySerial.println();
    activity = true;
  }

  if (statsDelay.isExpired()) {
    mySerial.println("--");
    mySerial.print("TX packets: ");
    mySerial.println(RF12_Stream::txPackets);
    
    mySerial.print("RX packets: ");
    mySerial.println(RF12_Stream::rxPackets);

    mySerial.print("Retries: ");
    mySerial.println(RF12_Stream::retries);
    statsDelay.start(statsInterval_ms, AsyncDelay::MILLIS);
  }

#ifdef LED_PIN
  if (activityDelay.isExpired()) {
    if (activity) {
      digitalWrite(LED_PIN, HIGH);
      activityDelay.start(250, AsyncDelay::MILLIS);
    }
    else
      digitalWrite(LED_PIN, LOW);
  }
#endif
}

