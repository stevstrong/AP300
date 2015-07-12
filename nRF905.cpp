/*
 * Project: nRF905 AVR/Arduino Library/Driver
 * Author: Zak Kemble, contact@zakkemble.co.uk
 * Copyright: (C) 2013 by Zak Kemble
 * License: GNU GPL v3 (see License.txt)
 * Web: http://blog.zakkemble.co.uk/nrf905-avrarduino-librarydriver/
 */

#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#ifdef ARDUINO
#include <Arduino.h>
#include <SPI.h>
#else
#include "nRF905_spi.h"
#endif
#include "nRF905.h"
#include "nRF905_config.h"
#include "nRF905_defs.h"
#include "nRF905_types.h"

#define noinline __attribute__ ((__noinline__))

#define AM_IS_USED_SW (NRF905_AM_SW)
#define NEED_SW_STATUS_SUPPORT (AM_IS_USED_SW || NRF905_DR_SW)

//#define DISABLE_STANDBY_MODE()	(nRF905_leaveStandBy())
//#define ENABLE_STANDBY_MODE()		(nRF905_enterStandBy())

static inline bool cselect(void)
{
	nRF905_interrupt_off();
//	spi_enable();
	SPI_SELECT();
	return true;
}

static inline bool cdeselect(void)
{
	SPI_DESELECT();
//	spi_disable();
	nRF905_interrupt_on();
	return false;
}

// Need to be in standby mode to write registers?
#define STANDBY (ENABLE_STANDBY_MODE())

#define CHIPSELECT(standby) standby; \
							for(bool cs = cselect(); cs; cs = cdeselect())

static inline bool interrupt_off(void)
{
	nRF905_interrupt_off();
	return true;
}

static inline bool interrupt_on(void)
{
	nRF905_interrupt_on();
	return false;
}

#define NRF905_ATOMIC() for(bool cs = interrupt_off(); cs; cs = interrupt_on())

typedef struct
{
	uint8_t reg1; // Change to array
	uint8_t reg2;
	uint8_t payloadSize;
} config_s;

typedef struct{
	nRF905_radio_state_t state;
	bool goToRxMode;
} state_s;

typedef struct{
	uint8_t buffer[NRF905_MAX_PAYLOAD];
	bool ready;
} data_s;

static void setConfigRegister(uint8_t, uint8_t);
//static noinline void defaultConfig(void);
static void setAddress(uint8_t * addr, uint8_t cmd);
#if !NRF905_INTERRUPTS
static bool dataReady(void);
#endif
static inline void stateTx(void);
#if NEED_SW_STATUS_SUPPORT
static uint8_t readStatus(void);
#endif

// We could instead read the registers over SPI, but that would be a bit slower
static config_s config;

#if NRF905_INTERRUPTS
#define DATA_BUFFERS	4	// must be power of 2 !!!
static data_s rxData[DATA_BUFFERS];
static volatile state_s radio;
static volatile uint8_t rxRecPtr;
static volatile uint8_t rxReadPtr;
#else
static state_s radio;
#endif

static void spi_transfer_nr(uint8_t cmd, uint8_t * address, uint8_t len)
{
	SPI_SELECT();
	spi_transfer(cmd);
	for(uint8_t i = 0; i<len; i++)
		address[i] = spi_transfer(address[i]);
	SPI_DESELECT();
}

#if 0
///////////////////////////////////////////////////////////
void nRF905_init()
{
#ifdef ARDUINO
	pinMode(TRX_EN, OUTPUT);
	pinMode(PWR_MODE, OUTPUT);
	pinMode(TX_EN, OUTPUT);

#if NRF905_COLLISION_AVOID
	pinMode(CD, INPUT);
#endif

#if AM_IS_USED_HW
	pinMode(AM, INPUT);
#endif

#if !NRF905_DR_SW
	pinMode(DR, INPUT);
#endif

	pinMode(CSN, OUTPUT);
	digitalWrite(CSN, HIGH);

	SPI.begin();
	SPI.setClockDivider(SPI_CLOCK_DIV2);
#else
	TRX_EN_DDR |= _BV(TRX_EN_BIT);
	PWR_MODE_DDR |= _BV(PWR_MODE_BIT);
	TX_EN_DDR |= _BV(TX_EN_BIT);

#if NRF905_COLLISION_AVOID
	CD_DDR &= ~_BV(CD_BIT);
#endif

#if AM_IS_USED_HW
	AM_DDR &= ~_BV(AM_BIT);
#endif

#if !NRF905_DR_SW
	DR_DDR &= ~_BV(DR_BIT);
#endif

	spi_init();
#endif

	radio.state = NRF905_RADIO_STATE_IDLE;

	// Startup
	ENABLE_STANDBY_MODE();
	RECEIVE_MODE();
	nRF905_powerDown();
	_delay_ms(3);
	defaultConfig();

#if NRF905_INTERRUPTS
	// Set interrupts
	REG_EXTERNAL_INT_CTL |= BIT_EXTERNAL_INT_CTL;
	nRF905_interrupt_on();
#endif

	nRF905_powerUp();
}
#endif

// Set frequency, workout the channel from the frequency
void nRF905_setFrequency(nRF905_band_t band, uint32_t freq)
{
	nRF905_setChannel(band, NRF905_CALC_CHANNEL(freq, band));
}

///////////////////////////////////////////////////////////////////////////
#if 0
// Set channel
void nRF905_setChannel(nRF905_band_t band, uint16_t channel)
{
	config.reg1 = (config.reg1 & NRF905_MASK_CHANNEL) | band | ((channel>>8) & 0x01);

	SPI_SELECT();
	spi_transfer(NRF905_CMD_W_CONFIG | NRF905_REG_CHANNEL);
	spi_transfer(channel);
	spi_transfer(config.reg1);
	SPI_DESELECT();
}

// Set auto retransmit
void nRF905_setAutoRetransmit(nRF905_auto_retran_t val)
{
	setConfigReg1(val, NRF905_MASK_AUTO_RETRAN, NRF905_REG_AUTO_RETRAN);
}
// Set low power receive
void nRF905_setLowRxPower(nRF905_low_rx_t val)
{
	setConfigReg1(val, NRF905_MASK_LOW_RX, NRF905_REG_LOW_RX);
}
// Set output power
void nRF905_setTransmitPower(nRF905_pwr_t val)
{
	setConfigReg1(val, NRF905_MASK_PWR, NRF905_REG_PWR);
}
// Set CRC
void nRF905_setCRC(nRF905_crc_t val)
{
	setConfigReg2(val, NRF905_MASK_CRC, NRF905_REG_CRC);
}
// Set clock output
void nRF905_setClockOut(nRF905_outclk_t val)
{
	setConfigReg2(val, NRF905_MASK_OUTCLK, NRF905_REG_OUTCLK);
}
#endif
//////////////////////////////////////////////////////////////////////
void nRF905_setConfigReg0(uint8_t val)
{
	setConfigRegister(NRF905_CMD_W_CONFIG, val);
}

void nRF905_setConfigReg1(uint8_t val)
{
	config.reg1 = val;
	setConfigRegister(NRF905_CMD_W_CONFIG | 1, val);
}

void nRF905_setConfigReg2(uint8_t val)
{
	config.reg2 = val;
	setConfigRegister(NRF905_CMD_W_CONFIG | 2, val);
}

void nRF905_setConfigReg9(uint8_t val)
{
	setConfigRegister(NRF905_CMD_W_CONFIG | 9, val);
}

static void setConfigRegister(uint8_t cmd, uint8_t val)
{
	SPI_SELECT();
	spi_transfer(cmd);
	spi_transfer(val);
	SPI_DESELECT();
}

uint8_t nRF905_getConfigReg(uint8_t reg)
{
	SPI_SELECT();
	spi_transfer(NRF905_CMD_R_CONFIG | reg);
	uint8_t retVal = spi_transfer(0);
	SPI_DESELECT();
	return retVal;
}

void nRF905_flushRecBuffer(void)
{
	SPI_SELECT();
	spi_transfer(NRF905_CMD_R_RX_PAYLOAD);
	for(uint8_t i=NRF905_MAX_PAYLOAD; i>0;i--)
		spi_transfer(NRF905_CMD_NOP);
	SPI_DESELECT();
	rxRecPtr = 0;
	rxReadPtr = 0;
}

#if 0
// Set configuration
// Radio should be in standby mode and interrupts disabled
static noinline void defaultConfig()
{
	uint16_t channel = NRF905_CALC_CHANNEL(NRF905_FREQ, NRF905_BAND);
	uint8_t reg1 = NRF905_AUTO_RETRAN | NRF905_LOW_RX | NRF905_PWR | NRF905_BAND | ((channel>>8) & 0x01);
	uint8_t reg2 = NRF905_CRC | NRF905_CLK_FREQ | NRF905_OUTCLK;

	config.reg1 = reg1;
	config.reg2 = reg2;
	config.payloadSize = NRF905_PAYLOAD_SIZE;
	
	// Set control registers
	SPI_SELECT();
	spi_transfer_nr(NRF905_CMD_W_CONFIG);
	spi_transfer_nr(channel);
	spi_transfer_nr(reg1);
	spi_transfer_nr((NRF905_ADDR_SIZE<<4) | NRF905_ADDR_SIZE);
	spi_transfer_nr(NRF905_PAYLOAD_SIZE); // RX payload size
	spi_transfer_nr(NRF905_PAYLOAD_SIZE); // TX payload size
	for(uint8_t i=4;i--;)
		spi_transfer_nr(0xE7); // Default receive address
	spi_transfer_nr(reg2);
	SPI_DESELECT();

	// Default transmit address
	SPI_SELECT();
	spi_transfer_nr(NRF905_CMD_W_TX_ADDRESS);
	for(uint8_t i=4;i--;)
		spi_transfer_nr(0xE7);
	SPI_DESELECT();
	
	// Clear transmit payload
	SPI_SELECT();
	spi_transfer_nr(NRF905_CMD_W_TX_PAYLOAD);
	for(uint8_t i=NRF905_MAX_PAYLOAD;i--;)
		spi_transfer_nr(0x00);
	SPI_DESELECT();

	// Clear DR by reading receive payload
	SPI_SELECT();
	spi_transfer_nr(NRF905_CMD_R_RX_PAYLOAD);
	for(uint8_t i=NRF905_MAX_PAYLOAD;i--;)
		spi_transfer_nr(NRF905_CMD_NOP);
	SPI_DESELECT();
}
#endif

// Payload size
void nRF905_setPayloadSizes(uint8_t size)
{
	if(size > NRF905_MAX_PAYLOAD)
		size = NRF905_MAX_PAYLOAD;
	config.payloadSize = size;
	SPI_SELECT();
	spi_transfer(NRF905_CMD_W_CONFIG | NRF905_REG_RX_PAYLOAD_SIZE);
	spi_transfer(size);
	spi_transfer(size);
	SPI_DESELECT();
}

// Power up
void nRF905_powerUp()
{
	radio.state = NRF905_RADIO_STATE_STANDBY;
	ENABLE_STANDBY_MODE();
	POWER_UP();

	// Give it time to turn on
	_delay_ms(3);
}

void nRF905_powerDown()
{
	ENABLE_STANDBY_MODE();
	POWER_DOWN();
	radio.state = NRF905_RADIO_STATE_POWER_DOWN;
}

void nRF905_enterStandBy()
{
	ENABLE_STANDBY_MODE();
	radio.state = NRF905_RADIO_STATE_STANDBY;
}

/*
// 
void nRF905_setAddressSize(uint8_t size)
{
	CHIPSELECT(STANDBY)
	{
		spi_transfer_nr(NRF905_CMD_W_CONFIG | NRF905_REG_ADDR_WIDTH);
		spi_transfer_nr((size<<4) | size);
	}
}
*/
// Set address
static void setAddress(uint8_t cmd, uint8_t * address)
{
	// Address bytes are sent in reverse order, which is fine as long as both ends do the same thing.
	spi_transfer_nr(cmd, address, NRF905_ADDR_SIZE);
}

// Set address of device to send to
void nRF905_setTXAddress(uint8_t * address)
{
	setAddress(NRF905_CMD_W_TX_ADDRESS, address);
}

// Set address for this device
void nRF905_setRXAddress(uint8_t * address)
{
	setAddress(NRF905_CMD_W_CONFIG | NRF905_REG_RX_ADDRESS, address);
}

// Set the payload data
uint8_t nRF905_setData(uint8_t * data, uint8_t len)
{
/*	// Busy transmitting something else
	if(radio.state == NRF905_RADIO_STATE_TX)
		return 1;//false;
*/
	nRF905_enterStandBy();
	uint8_t maxPayload = config.payloadSize;
	if(len > maxPayload)
		len = maxPayload;

	// Load data
	spi_transfer_nr(NRF905_CMD_W_TX_PAYLOAD, data, len);
	
	return 0;//true;
}

// See if device is receiving something
// Hardware: Address match pin high
// Software: Address match status bit set
bool nRF905_receiveBusy()
{
#if (!AM_IS_USED_HW && !AM_IS_USED_SW)
	return false;
#elif AM_IS_USED_SW
	return (readStatus() & _BV(NRF905_STATUS_AM));
#else
	return digitalRead(AM);
#endif
}

// See if data ready, true if received new data/finished transmitting
// Hardware: Data ready pin high
// Software: Data ready status bit set
#if !NRF905_INTERRUPTS
static bool dataReady()
{
#if NRF905_DR_SW
	return (readStatus() & _BV(NRF905_STATUS_DR));
#else
	return digitalRead(DR);
#endif
}
#endif

///////////////////////////////////////////////////////////
// Transmit payload
///////////////////////////////////////////////////////////
uint8_t nRF905_send(void)
{
	// Already transmitting
	if(radio.state == NRF905_RADIO_STATE_TX)
		return 1;//false;
#if NRF905_COLLISION_AVOID
	// Don't transmit if busy
	else if(nRF905_airwayBusy())
		return 2;//false;
#endif

	// Put into transmit mode
	TRANSMIT_MODE();

	radio.state = NRF905_RADIO_STATE_TX;

	// Pulse standby pin to start transmission
	DISABLE_STANDBY_MODE();

	_delay_ms(10);

#if NRF905_AUTO_RETRAN == NRF905_AUTO_RETRAN_DISABLE
	// Radio will go back into standby mode after transmission, unless nRF905_receive()
	// is called in which case it will go straight to receive mode after transmission.
	// If auto-retransmission is disabled and if the radio isn't set to go into standby or receive mode then it will
	// transmit a carrier signal with no data.
	ENABLE_STANDBY_MODE();
#endif

	return 0;//true;
}

// Return radio state
nRF905_radio_state_t nRF905_getState(void)
{
	return radio.state;
}

// Put into receive mode
void nRF905_receive(void)
{
//	NRF905_ATOMIC()
//	{
	if(radio.state == NRF905_RADIO_STATE_TX) // Currently transmitting, so wait until finished then go into receive mode
		while (nRF905_getStatus()==NRF905_RADIO_STATE_TX);	//radio.goToRxMode = true;
//		else
//		{
	RECEIVE_MODE();
	DISABLE_STANDBY_MODE();
	radio.state = NRF905_RADIO_STATE_RX;
	delay(1);
//		}
//	}
}

// Get received data if available
uint8_t nRF905_getData(uint8_t * data, uint8_t len)
{
static uint8_t i;
static uint8_t * ptr;
	if(len > config.payloadSize)
		len = config.payloadSize;

#if NRF905_INTERRUPTS
/*	// No data received
	if(!rxData.ready)
		return 1;//false;
*/		
	if (rxReadPtr==rxRecPtr)
		return 1;//false;

	// retrun actual read ptr
//	NRF905_ATOMIC()
//	{
		// Copy and clear data buffer
		//memcpy(data, (uint8_t*)rxData.buffer, len);
		ptr = rxData[rxReadPtr].buffer;
		for (i=0; i<len; i++)
			data[i] = *ptr++;
		//memset((uint8_t*)rxData.buffer, 0, sizeof(rxData.buffer));
		rxData[rxReadPtr].ready = false;
		rxReadPtr = (rxReadPtr+1)&(DATA_BUFFERS-1);
//	}
	return 0;//true;
#else
	// No data ready
	if(!dataReady())
		return 2;//false;

	uint8_t remaining;
	switch(radio.state)
	{
		case NRF905_RADIO_STATE_TX:
			// Finished transmitting payload
			//stateTx();
			break;
		case NRF905_RADIO_STATE_RX:
			// New payload received
			SPI_SELECT();
			spi_transfer(NRF905_CMD_R_RX_PAYLOAD);
			// Get received payload
			for(uint8_t i=0;i<len;i++)
				data[i] = spi_transfer(NRF905_CMD_NOP);
			// Must make sure all of the payload has been read, otherwise DR never goes low
			remaining = config.payloadSize - len;
			while(remaining--)
				spi_transfer(NRF905_CMD_NOP);
			SPI_DESELECT();
			// We're still in receive mode
			return 0;//true;
		default:
			break;
	}
	return 3;//false;
#endif
}
/*
static inline void stateTx()
{
	if(radio.goToRxMode)
	{
		// We want to go into receive mode
		radio.goToRxMode = false;
		RECEIVE_MODE();
		DISABLE_STANDBY_MODE();
		radio.state = NRF905_RADIO_STATE_RX;
	}
	else
	{
		// If we didn't want to go into receive mode then we're now in standby mode
		radio.state = NRF905_RADIO_STATE_IDLE;
	}
}
*/
#if NEED_SW_STATUS_SUPPORT
// Read status register
static uint8_t readStatus()
{
	uint8_t status;
	CHIPSELECT()
		status = spi_transfer(NRF905_CMD_NOP);
	return status;
}
#endif

#if NRF905_INTERRUPTS
// Data ready pin interrupt
ISR(INT_VECTOR)
{
	switch(radio.state)
	{
		case NRF905_RADIO_STATE_TX:
			// Finished transmitting payload
			//stateTx();
			break;
		case NRF905_RADIO_STATE_RX:
			// New payload received
			spi_transfer_nr(NRF905_CMD_R_RX_PAYLOAD, (uint8_t*)rxData[rxRecPtr].buffer, config.payloadSize);
			rxData[rxRecPtr].ready = true;
			rxRecPtr = (rxRecPtr+1)&(DATA_BUFFERS-1);
			// We're still in receive mode
			break;
		default:
			break;
	}
}
#endif
