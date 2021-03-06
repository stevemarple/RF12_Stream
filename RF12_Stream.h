/*
 * Arduino library to provide a software emulation of a transparent
 * serial link using the RFM12B radio module.
 *
 * Copyright S R Marple, 2013.
 * Released under MIT License, http://opensource.org/licenses/mit-license.php
 */

#ifndef RF12_STREAM_H
#define RF12_STREAM_H

#include <inttypes.h>
#include <CircBuffer.h> 
#include <AsyncDelay.h>
#include "Stream.h"

class RF12_Stream : public Stream
{
public:
  static volatile bool isPresent; // Value valid only after begin()
  static uint8_t packetDataLength;
  static uint16_t retryDelay_ms;
  static uint16_t txDelay_ms;
  static uint16_t flushTimeout_ms;
  static uint8_t maxRetriesPerPacket;

  // Transmission statistics
  static uint32_t retries;
  static uint32_t rxPackets;
  static uint32_t txPackets;
  
  RF12_Stream(void *rxBuffer, int rxBufferLen,
		void *txBuffer, int txBufferLen);
  // void begin(void);
  bool begin(uint8_t cs, uint8_t irqPin, uint8_t irqNum,
	     uint8_t id, uint8_t band, uint16_t channel,
	     uint8_t group=0xD4);

  // Deprecated, for compatibility with initial version. channel=1600.
  // inline bool begin(uint8_t cs, uint8_t irqPin, uint8_t irqNum,
  // 		    uint8_t id, uint8_t band, uint8_t group=0xD4) {
  //   return begin(cs, irqPin, irqNum,
  // 		 id, band, 1600, group);
  // }

  void poll(void);

  virtual int available(void);
  virtual int peek(void);
  virtual int read(void);
  virtual void flush(void);
  virtual size_t write(uint8_t);
  using Print::write; // pull in write(str) and write(buf, size) from Print

  inline const CircBuffer& getRxBuffer(void) const {
    return rxBuf;
  }

  inline const CircBuffer& getTxBuffer(void) const {
    return txBuf;
  }

  bool powerOn(void);
  bool powerOff(void);
  
  private:
  // Packet numbers, to avoid duplicates
  uint16_t rxNum;
  uint16_t txNum;
  uint16_t ackNum;
  bool sendAck;
  uint8_t txBytesPending; // number of bytes sent with no ACK
  uint8_t packetTries;
  CircBuffer rxBuf;
  CircBuffer txBuf;
  AsyncDelay retryDelay;
  AsyncDelay txDelay;

  static void detectHandler(void);
};



#endif
