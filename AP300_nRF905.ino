/*
 * Project: nRF905 AVR/Arduino Library/Driver
 * Author: Zak Kemble, contact@zakkemble.co.uk
 * Copyright: (C) 2013 by Zak Kemble
 * License: GNU GPL v3 (see License.txt)
 * Web: http://blog.zakkemble.co.uk/nrf905-avrarduino-librarydriver/
 */

/*
 * Time how long it takes to send some data and get a reply
 * Should be around 14-16ms with default settings.
 *
 * 7 -> PWR == PWR_MODE
 * 8 -> CE  == TRX_EN
 * 9 -> TXE == TX_EN
 * 2 -> CD
 * 3 -> DR  - uses INT1 interrupt
 * 10 -> CSN
 * 12 -> SO
 * 11 -> SI
 * 13 -> SCK
 */

#include <nRF905.h>
#include <SPI.h>

byte buf1[] = {0x00, 0x00, 0x0A, 0x7A}; // Address of device to send to (4 bytes) = TXADDR
byte buf2[] = {0x00, 0x00, 0x0A, 0x7A}; // Address of this device (4 bytes) = RXADDR

#define TIMEOUT 1000 // 1 second ping timeout

uint8_t payload[4][NRF905_MAX_PAYLOAD];
uint8_t plCounter;
//#define DBG_PIN 5
//////////////////////////////////////////////////////////////////////////
void nRF905_Config(void)
{
  // Set control register 0 - CHANNEL
  nRF905_setConfigReg0(0x76);
  // config reg 1
  nRF905_setConfigReg1(0x0E);
  // config reg 2
  nRF905_setConfigReg2(0x44);
  // Set payload sizes
  nRF905_setPayloadSizes(0x20);
  // set TX and Rx addresses
  nRF905_setTXAddress(buf1);
  // set Rx addres
  nRF905_setRXAddress(buf2);
  // read config register 9 - just to be conform with data detected by sniffing
  nRF905_getConfigReg(9);
  // set config register 9
  nRF905_setConfigReg9(0xDB);
  // Clear DR by reading receive payload
  nRF905_flushRecBuffer();
  // Set interrupts
//  REG_EXTERNAL_INT_CTL |= BIT_EXTERNAL_INT_CTL;
//  nRF905_interrupt_on();
}
//////////////////////////////////////////////////////////////////////////
void nRF905_Initialise()
{
  pinMode(PWR_MODE, OUTPUT);
  digitalWrite(PWR_MODE, LOW);  // activate power down mode
  pinMode(TRX_EN, OUTPUT);
  digitalWrite(TRX_EN, LOW);  // activate standby mode
  pinMode(TX_EN, OUTPUT);
  pinMode(CSN, OUTPUT);
  pinMode(CD, INPUT);
  pinMode(DR, INPUT);
  digitalWrite(CSN, HIGH);
#ifdef DBG_PIN
  pinMode(DBG_PIN, OUTPUT);
#endif

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV2);

  nRF905_Config();

#if NRF905_INTERRUPTS
	// Set interrupts
	REG_EXTERNAL_INT_CTL |= BIT_EXTERNAL_INT_CTL;
	nRF905_interrupt_on();
#endif

Serial.println(F("nRF905 configured..."));
StatusPins();
// leave config mode
  nRF905_powerUp();
Serial.println(F("nRF905 powered up..."));
StatusPins();
  // Put into receive mode
//  nRF905_receive();
}
//////////////////////////////////////////////////////////////////
void StatusPins(void)
{
return;
/**/
  Serial.print(F("pins: "));
  Serial.print(digitalRead(CD));
  Serial.print(",");
  Serial.print(digitalRead(DR));
  Serial.print(",");
  Serial.print(digitalRead(PWR_MODE));
  Serial.print(",");
  Serial.print(digitalRead(TRX_EN));
  Serial.print(",");
  Serial.println(digitalRead(TX_EN));
}
///////////////////////////////////////////////////////////////////
void setup()
{
	Serial.begin(57600);
	Serial.println(F("Client started"));
  StatusPins();
	// Start up
	nRF905_Initialise();
  StatusPins();
  Serial.println(F("nRF905 initialised"));
  plCounter = 0;
}

//////////////////////////////////////////////////////
nRF905_radio_state_t nRF905_getStatus(void)
{
	if (!digitalRead(PWR_MODE))	return NRF905_RADIO_STATE_POWER_DOWN;
	else if (!digitalRead(TRX_EN))	return NRF905_RADIO_STATE_STANDBY;	
	else if (!digitalRead(TX_EN)) {
		if (!digitalRead(DR))	return NRF905_RADIO_STATE_RX;
		else			return NRF905_RADIO_STATE_RX_END;
	} else {
		if (!digitalRead(DR))	return NRF905_RADIO_STATE_TX;
		else			return NRF905_RADIO_STATE_TX_END;
	}
}

/////////////////////////////////////////////////////////////
const char wr1[] PROGMEM = {0x90, 0x80, 0x03, 0x04, 0x00, 0x00, 0x0C};
const char wr2[] PROGMEM = {0x90, 0x80, 0x03, 0x04, 0x0C, 0x00, 0x0C};
static byte counter = 0;
static byte recOK = 0;
/////////////////////////////////////////////////////////////
void loop()
{
  byte ret;

  // Make data
  strcpy_P((char*)payload, wr1);
  counter++;
  Serial.println(counter);

#ifdef DBG_PIN
  digitalWrite(DBG_PIN, 1);
#endif
  unsigned long startTime = millis();
/*	// Set address of device to send to
	byte addr[] = TXADDR;
	nRF905_setTXAddress(addr);
*/
#if 0
//// SET DATA
  StatusPins();
  Serial.println(F("setting data ..."));
  // Set payload data
  ret = nRF905_setData(payload, sizeof(wr1));
  if (ret) Serial.println(F("Error by setting data!"));

//// SEND DATA
  StatusPins();
  Serial.println(F("sending data ..."));
  // Send payload (send fails if other transmissions are going on, keep trying until success)
  while(1) {
    ret = nRF905_send();
    if (ret==0) break;
    else {
      Serial.print(F("nRF905_send returned: "));
      Serial.println(ret);
      delay(100);
    }
    StatusPins();
  };
#endif
//// RECEIVE DATA
#ifdef DBG_PIN
  digitalWrite(DBG_PIN, 0);
  StatusPins();
  Serial.println(F("receiving data ..."));
#endif
  // Put into receive mode
  nRF905_receive();

  // Make buffer for reply
//  byte buffer[NRF905_MAX_PAYLOAD];
#define buffer payload

//// GETTING DATA
#ifdef DBG_PIN
  StatusPins();
  Serial.println(F("getting data ..."));
#endif
  // Wait for reply with timeout
  unsigned long time0 = millis() + TIMEOUT;
  while(1)
  {
    ret = nRF905_getData(&buffer[plCounter][0], NRF905_MAX_PAYLOAD);
    if (ret==0) {plCounter++; recOK = true; break;}  // Got data
    else {
#ifdef DBG_PIN
      Serial.print(F("getData returned: "));
      Serial.println(ret);
#endif
    }
//    StatusPins();
    // check timeout
    if ( millis() > time0 )  break;
  }

//// EVALUATE DATA
#ifdef DBG_PIN
  StatusPins();
  Serial.println(F("evaluating data..."));
#endif
  if ( ret==0 )
  {  // data received. Do nothing, wait for time-out for display received data
#ifdef DBG_PIN
    unsigned int totalTime = millis() - startTime;
    Serial.print(F("Ping time: "));
    Serial.print(totalTime);
    Serial.println(F("ms"));
    // Printout ping contents
    Serial.print(F("Replay from AP300: "));
#endif
  }
  else if (recOK) {
//    Serial.print(F("Replay from AP300: "));
    for (uint8_t y = 0; y<plCounter; y++) {
      for (byte i=0; i<NRF905_MAX_PAYLOAD; i++) {
        ret = buffer[y][i];
        if (ret<16)        Serial.print(0, HEX);
        Serial.print(ret, HEX);
      }
      Serial.println();
    }
    recOK = false;
    plCounter = 0;
    Serial.println();
  } else // time-out
    Serial.println(F("time-out"));
/*
  if (counter>=100)
    while (1);  // stop here
//  delay(100);
*/
}
