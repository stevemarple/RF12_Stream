/*
 * Example sketch to demonstrate the concept of how a single firmware
 * image can support the RFM12B and Ciseco XRF (or XBee) modules,
 * autoselecting between them depending on the hardware fitted. This
 * example assumes that the XRF is connected to Serial1, leaving
 * Serial free for debugging information.
 *
 */


#include <AsyncDelay.h>
#include <CircBuffer.h>
#include <RF12.h>
#include <RF12_Stream.h>

// Pin mapping is for Calunium, adjust to suit your hardware.
const uint8_t rfm12bCs = 14;
const uint8_t rfm12bIrqPin = 6;
const uint8_t rfm12bIrqNum = 2;

uint8_t rxBuffer[60];
uint8_t txBuffer[60];
RF12_Stream rfm12b(rxBuffer, sizeof(rxBuffer), txBuffer, sizeof(txBuffer));

/* Reference to the radio module. Default to RFM12B since Serial might
 * not exist (eg ATtiny). Use RF12_Stream::begin() in setup() to point
 * the reference to Serial if the RFM12B is not found.
 */
Stream *radioPtr;

AsyncDelay txTimer;

void setup(void)
{
#ifdef SS
  pinMode(SS, OUTPUT);
#endif
  Serial.begin(9600);
  if (rfm12b.begin(rfm12bCs, rfm12bIrqPin, rfm12bIrqNum, 1, RF12_433MHZ)) {
    radioPtr = &rfm12b;
    Serial.println("Found RFM12B");
  }
  else {
    Serial.println("RFM12B not found");
    Serial1.begin(9600);
    radioPtr = &Serial1;
  }
}


void loop(void)
{
  if (RF12_Stream::isPresent)
    rfm12b.poll();
  
  // Do your stuff here

  if (txTimer.isExpired()) {
    // Send message every second
    Serial.println("Sending message"); 
    radioPtr->print("Send a message");
    txTimer.start(1000, AsyncDelay::MILLIS);
  }

}
