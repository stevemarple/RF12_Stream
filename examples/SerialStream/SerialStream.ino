/*
 * Sketch to transfer data between the serial port and an RFM12B. This
 * sketch can be used to emulate an XBee or Ciseco XRF radio link.
 * You will need to use a pair RFM12B modules with similar software at
 * both ends of the link.
 *
 * Supported hardware:
 *   # Calunium v2.0, v2.1 with ATmega1284P or similar microcontroller.
 *
 *   # RFM12B shield for Raspberry Pi with an ATmega328 or ATmega328P
 *     microcontroller.
 *
 * The firmware size is under 8kB when compiled for the ATtiny84 so it
 * may be possible to run the sketch on the RFM12B to Pi expansion
 * board, http://shop.openenergymonitor.com/raspberry-pi/.
 *
 * Copyright S R Marple, 2013.
 * Released under MIT License, http://opensource.org/licenses/mit-license.php
 */

#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <util/crc16.h>
#include <string.h>

#include <CircBuffer.h>
#include <AsyncDelay.h>
#include <RF12.h>
#include <RF12_Stream.h>


#define FACTORY_BAND RF12_433MHZ
#define FACTORY_CHANNEL 1600
#define FACTORY_GROUP 0xD4
#define FACTORY_LOCAL_NODE_ID 1
#define FACTORY_REMOTE_NODE_ID 2

#define SS_EEPROM_ADDR 0x20
#define SS_EEPROM_SIZE (SS_EEPROM_ADDR - SS_EEPROM_END)
#define SS_EEPROM_CRC SS_EEPROM_ADDR
#define SS_EEPROM_CRC_SIZE 2
#define SS_EEPROM_BAND (SS_EEPROM_CRC + SS_EEPROM_CRC_SIZE)
#define SS_EEPROM_BAND_SIZE 1
#define SS_EEPROM_CHANNEL (SS_EEPROM_BAND + SS_EEPROM_BAND_SIZE)
#define SS_EEPROM_CHANNEL_SIZE 2 
#define SS_EEPROM_GROUP (SS_EEPROM_CHANNEL + SS_EEPROM_CHANNEL_SIZE)
#define SS_EEPROM_GROUP_SIZE 1
#define SS_EEPROM_LOCAL_NODE_ID (SS_EEPROM_GROUP + SS_EEPROM_GROUP_SIZE)
#define SS_EEPROM_LOCAL_NODE_ID_SIZE 1 
#define SS_EEPROM_REMOTE_NODE_ID (SS_EEPROM_LOCAL_NODE_ID + SS_EEPROM_LOCAL_NODE_ID_SIZE)
#define SS_EEPROM_REMOTE_NODE_ID_SIZE 1
#define SS_EEPROM_PACKET_DATA_LENGTH (SS_EEPROM_REMOTE_NODE_ID + SS_EEPROM_REMOTE_NODE_ID_SIZE)
#define SS_EEPROM_PACKET_DATA_LENGTH_SIZE 1
#define SS_EEPROM_RETRY_DELAY (SS_EEPROM_PACKET_DATA_LENGTH + SS_EEPROM_PACKET_DATA_LENGTH_SIZE)
//#define SS_EEPROM_RETRY_DELAY (SS_EEPROM_PACKET_DATA_LENGTH + 99)
//#define SS_EEPROM_RETRY_DELAY (SS_EEPROM_PACKET_DATA_LENGTH_SIZE)
//#define SS_EEPROM_RETRY_DELAY 99
#define SS_EEPROM_RETRY_DELAY_SIZE 2
#define SS_EEPROM_TX_DELAY (SS_EEPROM_RETRY_DELAY + SS_EEPROM_RETRY_DELAY_SIZE)
#define SS_EEPROM_TX_DELAY_SIZE 2
#define SS_EEPROM_MAX_RETRIES (SS_EEPROM_TX_DELAY + SS_EEPROM_TX_DELAY_SIZE)
#define SS_EEPROM_MAX_RETRIES_SIZE 1

#define SS_EEPROM_END (SS_EEPROM_MAX_RETRIES + SS_EEPROM_MAX_RETRIES_SIZE)

// By default include command mode. Automatically disabled for MCUs
// with low RAM (eg ATtiny series).
#define CMD_MODE

#if defined(UBRRH) || defined(UBRR0H) || defined(USBCON)
Stream &mySerial = Serial;
#else
// No hardware serial or USB interface
#define USE_SW_SERIAL
#include <SoftwareSerial.h>
#endif

#ifdef CALUNIUM
// Calunium, use pin mapping for v2.0
#define RFM12B_CS 14
#define RFM12B_IRQ_PIN 6
#define RFM12B_IRQ_NUM 2
//#define LED_PIN LED_BUILTIN
#define LED_PIN 8

#elif defined(__AVR_ATmega168__) || defined(__AVR_ATmega328__) || defined (__AVR_ATmega328P__)
// Mapping used by Jeenode and RFM12B shield for Raspberry Pi.
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
#undef CMD_MODE
uint8_t rxBuffer[50];
uint8_t txBuffer[50];
#endif


#ifdef CMD_MODE
/* Command-mode variables */

// The character which must be sent three times to enter command mode.
const char guardChar = '+';

// Time before and after guard characters have been sent with no other
// characters sent.
const uint16_t guardTime_ms = 1000;

// Duration in which all 3 guard characters must be sent.
const uint16_t interGuardTime_ms = 500;

// Maximum period to remain in command mode with no characters received.
const uint16_t cmdModeTimeout_ms = 5000;

static const uint8_t inCmdModeStateNum = 4;
uint8_t cmdModeState = 0;

AsyncDelay guardTimer; // Measure delays for guard time

AsyncDelay cmdModeTimer; // Measure inter-guard time and cmd mode timeout

const uint8_t cmdBufferLen = 40;
char cmdBuffer[cmdBufferLen] = {'\0'};
char *cmdPtr = cmdBuffer;

enum ledMode_t {
  ledModeOff = '0',
  ledModeOn = '1',
  ledModeActivity = 'A',
  ledModeHeartbeat = 'H',
  ledModeDebug = 'D',
};
bool ledState = LOW;
ledMode_t ledMode = ledModeDebug;
#endif


static uint8_t band = FACTORY_BAND;
static uint16_t channel = FACTORY_CHANNEL;
// RFM12 only supports group 212 (0xD4), RFM12B supports groups 0-250
// inclusive. Group 0 is special, it can receive communications from
// all groups.
static uint8_t group = FACTORY_GROUP;
static uint8_t localNodeId = FACTORY_LOCAL_NODE_ID;
static uint8_t remoteNodeId = FACTORY_REMOTE_NODE_ID;

RF12_Stream rfm12b(rxBuffer, sizeof(rxBuffer),
		   txBuffer, sizeof(txBuffer));


const char *firmwareVersion = "SerialStream-2.0";
unsigned long sendInterval_ms = 2000UL;
AsyncDelay activityDelay;
uint16_t heartbeatDelay_ms = 500;

const char* okStr = "OK";
const char* errStr = "ERR"; 

#ifdef CMD_MODE
void readEepromConfig(void)
{

  uint16_t crc = 0;
  for (uint16_t p = SS_EEPROM_CRC + SS_EEPROM_CRC_SIZE; p < SS_EEPROM_END; ++p)
    crc = _crc16_update(crc, eeprom_read_byte((const uint8_t *)p));

  mySerial.print("Computed CRC: ");
  mySerial.println(crc);
  mySerial.print("CRC in EEPROM: ");
  mySerial.println(eeprom_read_word((const uint16_t*)SS_EEPROM_CRC));

  
  if (crc == eeprom_read_word((const uint16_t*)SS_EEPROM_CRC)) {
    band = eeprom_read_byte((const uint8_t*)SS_EEPROM_BAND);
    channel = eeprom_read_word((const uint16_t*)SS_EEPROM_CHANNEL);
    group = eeprom_read_byte((const uint8_t*)SS_EEPROM_GROUP);
    localNodeId = eeprom_read_byte((const uint8_t*)SS_EEPROM_LOCAL_NODE_ID);
    remoteNodeId = eeprom_read_byte((const uint8_t*)SS_EEPROM_REMOTE_NODE_ID);
    RF12_Stream::packetDataLength =
      eeprom_read_byte((const uint8_t*)SS_EEPROM_PACKET_DATA_LENGTH);
    RF12_Stream::retryDelay_ms =
      eeprom_read_word((const uint16_t*)SS_EEPROM_RETRY_DELAY);
    RF12_Stream::txDelay_ms =
      eeprom_read_word((const uint16_t*)SS_EEPROM_TX_DELAY);
    
  }
  else
    // EEPROM settings not valid, so write current settings to EEPROM
    writeEepromConfig();

}

void writeEepromConfig(void)
{
  eeprom_update_byte((uint8_t *)SS_EEPROM_BAND, band);
  eeprom_update_word((uint16_t *)SS_EEPROM_CHANNEL, channel);
  eeprom_update_byte((uint8_t*)SS_EEPROM_GROUP, group);
  eeprom_update_byte((uint8_t*)SS_EEPROM_LOCAL_NODE_ID, localNodeId);
  eeprom_update_byte((uint8_t*)SS_EEPROM_REMOTE_NODE_ID, remoteNodeId);
  eeprom_update_byte((uint8_t*)SS_EEPROM_PACKET_DATA_LENGTH,
  		    (RF12_Stream::packetDataLength));
  eeprom_update_word((uint16_t*)SS_EEPROM_RETRY_DELAY,
		    RF12_Stream::retryDelay_ms);
  eeprom_update_word((uint16_t*)SS_EEPROM_TX_DELAY,
		    RF12_Stream::txDelay_ms);
  
  // Compute and write CRC
  uint16_t crc = 0;
  for (uint16_t p = SS_EEPROM_CRC + SS_EEPROM_CRC_SIZE; p < SS_EEPROM_END; ++p)
    crc = _crc16_update(crc, eeprom_read_byte((const uint8_t *)p));

  eeprom_update_word((uint16_t *)SS_EEPROM_CRC, crc);
  mySerial.print("Writing CRC: ");
  mySerial.println(crc);
}

void processIncomingChar(char c)
{
  switch (cmdModeState) {
  case 0:
    if (c == guardChar && guardTimer.isExpired()) {
      // Have seen initial guard time and first guard character. The
      // remaining two guard characters must be seen within the
      // interguard time.
      guardTimer.start(guardTime_ms, AsyncDelay::MILLIS);
      cmdModeTimer.start(interGuardTime_ms, AsyncDelay::MILLIS);
      ++cmdModeState;
      return;
    }
    break;

  case 1:
  case 2:
    if (c == guardChar && !cmdModeTimer.isExpired()) {
      if (cmdModeState == 2) {
	guardTimer.start(guardTime_ms, AsyncDelay::MILLIS);
	cmdModeTimer.start(cmdModeTimeout_ms, AsyncDelay::MILLIS);
      }
      ++cmdModeState;
      return;
    }
    else {
      // Send the partial guard string for transmission before sending c.
      for (uint8_t i = 0; i < cmdModeState; ++i)
	rfm12b.write(guardChar);
    }
    break;

  case 3:
    if (guardTimer.isExpired()) {
      // Second guard time has been seen, but has the command mode
      // timeout been exceeded?

      if (cmdModeTimer.isExpired()) {
	// Correctly went into command mode but no commands sent
	// before command mode timeout. Discard the guard characters
	// and transmit the current character.
	break;
      }
      
      // Switch into command mode.
      guardTimer.start(guardTime_ms, AsyncDelay::MILLIS);
      cmdModeTimer.start(cmdModeTimeout_ms, AsyncDelay::MILLIS);
      cmdPtr = cmdBuffer; // Ensure previous characters are discarded
      ++cmdModeState;
      processCmdBuffer(c);
      return;
    }
    else {
      // Send the guard string for transmission before sending c.
      for (uint8_t i = 0; i < cmdModeState; ++i)
	rfm12b.write(guardChar);
    }
    break;

  case 4:
    if (!cmdModeTimer.isExpired()) {
      guardTimer.start(guardTime_ms, AsyncDelay::MILLIS);
      cmdModeTimer.start(cmdModeTimeout_ms, AsyncDelay::MILLIS);
      processCmdBuffer(c);
      return;
    }
    break;
    
  default:
    // Unknown states get mapped to data transmission mode
    cmdModeState = 0;
    break;
  };
  
  // Not in command mode. Reset state to beginning, reset cmdModeTimer
  // to check for first guard time, send data to be transmitted.
  cmdModeState = 0;
  guardTimer.start(guardTime_ms, AsyncDelay::MILLIS);
  rfm12b.write(c);
}

// Check for an incomplete guard sequence. Also check for a completed
// guard sequence but no command characters sent within
// cmdModeTimeout_ms.
void handleCmdModeTimeouts(void)
{
  switch (cmdModeState) {
  case 0:
    // do nothing
    break;
    
  case 1:
  case 2:
    if (cmdModeTimer.isExpired()) {
      // Guard sequence not completed in time
      for (uint8_t i = 0; i < cmdModeState; ++i) 
	rfm12b.write(guardChar);
      cmdModeState = 0;
    }
    break;

  case 3:
    if (cmdModeTimer.isExpired())
      cmdModeState = 0;
    else if (guardTimer.isExpired()) {
      mySerial.println(okStr);
      cmdModeState = 4;
    }
    break;

  case 4:
    if (cmdModeTimer.isExpired())
      cmdModeState = 0;
    break;

  default:
    cmdModeState = 0;
    break;
  };
}

bool startsWith(char *&ptr, PGM_P s)
{
  if (strncasecmp_P(ptr, s, strlen_P(s)) == 0) {
    // Jump past the command and skip whitespace
    ptr += strlen_P(s); 
    while (isspace(*ptr))
      ++ptr;
    return true;
  }
  return false;
}

void processCmdBuffer(char c)
{
  if (c == '\n' || c == '\r') {
    // End of command, process
    *cmdPtr = '\0';
    if (strncasecmp_P(cmdBuffer, PSTR("AT"), 2) != 0)
      mySerial.println(errStr);
    else {
      cmdPtr = cmdBuffer + 2;

      if (*cmdPtr == '\0')
	mySerial.println(okStr); // Plain AT command
      else if (startsWith(cmdPtr, PSTR("AC")))
	processATAC(); // Apply changes
      else if (startsWith(cmdPtr, PSTR("CH")))
	processATCH(); // Channel (band)
      else if (startsWith(cmdPtr, PSTR("CN")))
	processATCN(); // Channel number

      else if (startsWith(cmdPtr, PSTR("DN")))
	processATDN(); // Done
      else if (startsWith(cmdPtr, PSTR("LI")))
	processATLI(); // LED indicator
      else if (startsWith(cmdPtr, PSTR("LN")))
	processATLN(); // Local node ID
      else if (startsWith(cmdPtr, PSTR("RB")))
	processATRB(); // Reboot
      else if (startsWith(cmdPtr, PSTR("RN")))
	processATRN(); // Remote node ID
      else if (startsWith(cmdPtr, PSTR("VR"))) {
	mySerial.println(firmwareVersion); // Firmware version
	mySerial.println(okStr);
      }
      else {
	mySerial.println(errStr);
      }
    }
    cmdPtr = cmdBuffer;
    *cmdPtr = '\0';
    return;
  }
  if (cmdPtr == cmdBuffer && isspace(c))
    // Don't add whitespace to start of buffer
    return;

  // Add to the buffer, avoiding overflow
  if (cmdPtr < cmdBuffer + cmdBufferLen - 1)
    *cmdPtr++ = c;
}

void processATAC(void)
{
  writeEepromConfig();
  processATRB();
}


void processATCH(void)
{
  if (*cmdPtr == '\0') {
    mySerial.println(band, DEC);
    mySerial.println(int(band));
    mySerial.println(okStr);
    return;
  }
  char **eptr;
  long s = strtol(cmdPtr, eptr, 0);
  if (*eptr == '\0' &&
      (s == RF12_433MHZ || s == RF12_868MHZ || s == RF12_915MHZ)) {
    band = s;
    mySerial.println(okStr);
  }
  else
    mySerial.println(errStr);
}

void processATCN(void)
{
  if (*cmdPtr == '\0') {
    mySerial.println(channel, DEC);
    mySerial.println(okStr);
    return;
  }
  char **eptr;
  long s = strtol(cmdPtr, eptr, 0);
  if (*eptr == '\0' && s >= 0 && s <= 0xFFF) {
    channel = s;
    mySerial.println(okStr);
  }
  else
    mySerial.println(errStr);
}

void processATDN(void)
{
  mySerial.println(okStr);
  cmdModeState = 0;
}

void processATGR(void)
{
  if (*cmdPtr == '\0') {
    mySerial.println(group, DEC);
    mySerial.println(okStr);
    return;
  }
  char **eptr;
  long s = strtol(cmdPtr, eptr, 0);
  if (*eptr == '\0' && s >= 0 && s <= 250) {
    group = s;
    mySerial.println(okStr);
  }
  else
    mySerial.println(errStr);
}

void processATLI(void)
{
  switch (toupper(*cmdPtr)) {
  case '\0':
    mySerial.println(char(ledMode));
    mySerial.println(okStr);
    return;
    
  case 'O': // For compatibility with XRF
  case '0':
    ledMode = ledModeOff;
    break;

  case '1':
    ledMode = ledModeOn;
    mySerial.println(okStr);
#ifdef LED_PIN 
    digitalWrite(LED_PIN, HIGH);
#endif
    return;

  case 'A':
    ledMode = ledModeActivity;
    break;

  case 'D':
    ledMode = ledModeDebug;
    mySerial.println(okStr);
#ifdef LED_PIN 
    digitalWrite(LED_PIN, HIGH);
#endif
    return;
    
  case 'H':
    ledMode = ledModeHeartbeat;
    ledState = LOW;
    activityDelay.start(heartbeatDelay_ms, AsyncDelay::MILLIS);
    break;

  default:
    mySerial.println(errStr);
    return;
  };

  mySerial.println(okStr);
#ifdef LED_PIN 
  digitalWrite(LED_PIN, LOW);
#endif
}

void processATLN(void)
{
  if (*cmdPtr == '\0') {
    mySerial.println(localNodeId, DEC);
    mySerial.println(okStr);
    return;
  }
  char **eptr;
  long s = strtol(cmdPtr, eptr, 0);
  if (*eptr == '\0' && s > 0 && s <= 31) {
    localNodeId = s;
    mySerial.println(okStr);
  }
  else
    mySerial.println(errStr);
}

void processATRB(void)
{
  mySerial.println(okStr);
  mySerial.flush();
  wdt_enable(WDTO_15MS);
  while (1)
    ; // Loop until reboot
}

void processATRN(void)
{
  if (*cmdPtr == '\0') {
    mySerial.println(remoteNodeId, DEC);
    mySerial.println(okStr);
    return;
  }
  char **eptr;
  long s = strtol(cmdPtr, eptr, 0);
  if (*eptr == '\0' && s > 0 && s <= 30) {
    remoteNodeId = s;
    mySerial.println(okStr);
  }
  else
    mySerial.println(errStr);
}

#endif

void ledActivityIndicator(bool activity)
{
  if (activityDelay.isExpired()) {
    if (activity) {
      digitalWrite(LED_PIN, HIGH);
      activityDelay.start(250, AsyncDelay::MILLIS);
    }
    else
      digitalWrite(LED_PIN, LOW);
  }
}

void setup(void)
{
  MCUSR &= ~(1 << WDRF);
  wdt_disable();
  wdt_enable(WDTO_4S);
  
#ifdef SS
  pinMode(SS, OUTPUT);
#endif
  
#ifdef USE_SW_SERIAL
  swSerial.begin(9600);
#else
  Serial.begin(38400);
#endif

  mySerial.print("Firmware version ");
  mySerial.println(firmwareVersion);
#ifdef CMD_MODE
  mySerial.println("Command mode: enabled");

  readEepromConfig();
#endif
  
  if (rfm12b.begin(RFM12B_CS, RFM12B_IRQ_PIN, RFM12B_IRQ_NUM,
		   1, band, channel, group)) {
  }
  else 
    while (1) {
      delay(500);
      mySerial.println("RFM12B not found");
    }

#ifdef LED_PIN
  digitalWrite(LED_PIN, LOW);
#endif
  activityDelay.expire();

  wdt_reset();
}

void loop(void)
{
  bool activity = false;
  rfm12b.poll();

#ifdef CMD_MODE
  handleCmdModeTimeouts();
#endif
  
  while (mySerial.available() && rfm12b.getTxBuffer().getSizeRemaining()) {
#ifdef CMD_MODE
    processIncomingChar(mySerial.read());
#else
    rfm12b.write(char(mySerial.read()));
#endif
    activity = true;
  }

#ifdef CMD_MODE
  if (cmdModeState != inCmdModeStateNum)
#endif
    while (rfm12b.available()) {
      mySerial.write(char(rfm12b.read()));
      activity = true;
    }
  
#ifdef LED_PIN
#ifdef CMD_MODE
  switch (ledMode) {
  case ledModeActivity:
    ledActivityIndicator(activity);
    break;

  case ledModeHeartbeat:
    if (activityDelay.isExpired()) {
      activityDelay.start(heartbeatDelay_ms, AsyncDelay::MILLIS);
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
    }
    break;
    
  case ledModeDebug:
    {
      static uint8_t lastState = cmdModeState;
      if (lastState != cmdModeState) {
	analogWrite(LED_PIN, cmdModeState * 63);
	lastState = cmdModeState;
      }
    }
    break;

  case ledModeOn:
  case ledModeOff:
    break;
  };

#else
  ledActivityIndicator(activity);
#endif // #ifdef CMD_MODE
#endif // #ifdef LED_PIN
  
  wdt_reset();
}

