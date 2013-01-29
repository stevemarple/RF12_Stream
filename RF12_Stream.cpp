#include "Arduino.h"

#include <RF12.h>
#include "RF12_Stream.h"

uint8_t RF12_Stream::packetDataLength = 12;
unsigned long RF12_Stream::retryDelay_ms = 50;
unsigned long RF12_Stream::txDelay_ms = 10;
uint8_t RF12_Stream::maxRetriesPerPacket = 3;
uint32_t RF12_Stream::retries = 0;
uint32_t RF12_Stream::rxPackets = 0;
uint32_t RF12_Stream::txPackets = 0;

volatile bool RF12_Stream::found = false;

RF12_Stream::RF12_Stream(void *rxBuffer, int rxBufferLen,
		void *txBuffer, int txBufferLen)
  : rxNum(-1),
    txNum(0),
    ackNum(0),
    sendAck(false),
    txBytesPending(0),
    txRetries(0),
    //rxBuf(rxBuffer, rxBufferLen, true, false),
    //txBuf(txBuffer, txBufferLen, true, false) 
    rxBuf(rxBuffer, rxBufferLen),
    txBuf(txBuffer, txBufferLen) 
{
  ;
}


bool RF12_Stream::begin(uint8_t cs, uint8_t irqPin, uint8_t irqNum,
			uint8_t id, uint8_t band, uint8_t group)
{
  pinMode(cs, OUTPUT);
  pinMode(irqPin, INPUT);
  rf12_set_cs(cs);
  rf12_set_irq(irqPin, irqNum);
  rf12_initialize(id, band, group);

  // Check RFM12B is present. Use our own interrupt handler to respond
  // to a software reset.
  attachInterrupt(irqNum, RF12_Stream::detectHandler, LOW);
  rf12_control(0xCA82); // Set up sensitive reset mode
  rf12_control(0xFE00); // Software reset
  
  AsyncDelay detectTimeout;
  detectTimeout.start(100, AsyncDelay::MILLIS);
  while (!detectTimeout.isExpired() && !found)
    ; // Loop

  // Restore settings
  rf12_initialize(id, band, group);
  return found;
}


void RF12_Stream::poll(void)
{
  // Check for received packet
  if (rf12_recvDone() && rf12_crc == 0) {
    if ((rf12_hdr & RF12_HDR_CTL) && rf12_len == sizeof(txNum)) {
      // ACK received
      uint16_t recdAckNum = (rf12_data[0] << 8) | rf12_data[1];
      if (recdAckNum == txNum) {
	txBuf.skip(txBytesPending);
	txBytesPending = 0;
	txRetries = 0;
	++txNum;
      }
    }
    else if ((rf12_hdr & RF12_HDR_CTL) == 0 && rf12_len > sizeof(txNum) &&
	     rf12_len <= (packetDataLength + sizeof(txNum))) {
      // Data
      ackNum = (rf12_data[0] << 8) | rf12_data[1];
      sendAck = true;
      if (rxNum != ackNum) {
	rxNum = ackNum;
	rxBuf.write((const uint8_t*) rf12_data + 2, rf12_len - 2);
	++rxPackets;
      }
      
    }

    // if (RF12_WANTS_ACK)
    // rf12_sendStart(RF12_ACK_REPLY, NULL, 0);

    // Must wait for tx delay to expire before can transmit
    txDelay.start(txDelay_ms, AsyncDelay::MILLIS);
  }

  if (sendAck && txDelay.isExpired() && rf12_canSend()) {
    uint8_t ackData[sizeof(ackNum)];
    ackData[1] = ackNum & 0xFF;
    ackData[0] = (ackNum >> 8) & 0xFF;
    rf12_sendStart(RF12_ACK_REPLY, ackData, sizeof(ackData));
    txDelay.start(txDelay_ms, AsyncDelay::MILLIS);
    sendAck = false;
  }
 
  if (txRetries >= maxRetriesPerPacket) {
    // Enough tries on this portion of data
    txBuf.skip(txBytesPending);
    txBytesPending = 0;
    txRetries = 0;
    ++txNum;
  }
      
  // Check if data to send.
  // txBytesPending == 0 means no unacknowledged data
  if ((txBytesPending == 0 || retryDelay.isExpired()) &&
      txDelay.isExpired() &&
      txBuf.getSize() && rf12_canSend()) {
    if (txRetries > 1)
      ++retries;
    else
      ++txPackets;
    
    ++txRetries;
    uint8_t message[sizeof(txNum) + packetDataLength];
    // First two bytes are the TX packet number, sent in network order
    message[1] = txNum & 0xFF;
    message[0] = (txNum >> 8) & 0xFF;
    txBytesPending = txBuf.peek(message+sizeof(txNum), packetDataLength);
    uint8_t hdr = RF12_HDR_ACK; // RF12_HDR_DST | 0x02;
    rf12_sendStart(hdr, message, sizeof(txNum) + txBytesPending);
    txDelay.start(txDelay_ms, AsyncDelay::MILLIS);
    retryDelay.start(retryDelay_ms, AsyncDelay::MILLIS);
  }

}


int RF12_Stream::available(void)
{
  return rxBuf.getSize();
}

int RF12_Stream::peek(void)
{
  uint8_t c;
  if (rxBuf.peek(&c, sizeof(c)))
    return c;
  else
    return -1;
}

int RF12_Stream::read(void)
{
  uint8_t c;
  if (rxBuf.read(&c, sizeof(c)))
    return c;
  else
    return -1;
}

void RF12_Stream::flush()
{
  // Not sure that flush() is very useful given the requirement to
  // call poll(). Support but do nothing.
  ;
}

size_t RF12_Stream::write(uint8_t c)
{
  return txBuf.write(&c, sizeof(c));
}

void RF12_Stream::detectHandler(void)
{
  if (rf12_control(0x0000) & 0x4000)
    found = true;
}
