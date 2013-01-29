#ifndef RF12_STREAM_H
#define RF12_STREAM_H

//#include <stdint.h>
#include <inttypes.h>
//#include <MultiReadCircBuffer.h>
#include <CircBuffer.h> 
#include <AsyncDelay.h>
#include "Stream.h"

class RF12_Stream : public Stream
{
public:
  static volatile bool found; // Value valid only after begin()
  static uint8_t packetDataLength;
  static unsigned long retryDelay_ms;
  static unsigned long txDelay_ms;
  static uint8_t maxRetriesPerPacket;

  // Transmission statistics
  static uint32_t retries;
  static uint32_t rxPackets;
  static uint32_t txPackets;
  
  RF12_Stream(void *rxBuffer, int rxBufferLen,
		void *txBuffer, int txBufferLen);
  // void begin(void);
  bool begin(uint8_t cs, uint8_t irqPin, uint8_t irqNum,
	     uint8_t id, uint8_t band, uint8_t group=0xD4);
  
  void poll(void);

  virtual int available(void);
  virtual int peek(void);
  virtual int read(void);
  virtual void flush(void);
  virtual size_t write(uint8_t);
  using Print::write; // pull in write(str) and write(buf, size) from Print

  //inline const MultiReadCircBuffer& getRxBuffer(void) const {
  inline const CircBuffer& getRxBuffer(void) const {
    return rxBuf;
  }
  //inline const MultiReadCircBuffer& getTxBuffer(void) const {
  inline const CircBuffer& getTxBuffer(void) const {
    return txBuf;
  }
  
  private:
  // Packet numbers, to avoid duplicates
  uint16_t rxNum;
  uint16_t txNum;
  uint16_t ackNum;
  bool sendAck;
  uint8_t txBytesPending; // number of bytes sent with no ACK
  uint8_t txRetries;
  //MultiReadCircBuffer rxBuf;
  //MultiReadCircBuffer txBuf;
  CircBuffer rxBuf;
  CircBuffer txBuf;
  AsyncDelay retryDelay;
  AsyncDelay txDelay;

  static void detectHandler(void);
};



#endif
