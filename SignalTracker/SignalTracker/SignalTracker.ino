/*

Copyright (c) [2018] [Startup Factory Ruse]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

#pragma region Definitions

/**
* @brief Device settings block.
*/
//#error "Create your own keys and device IDs from the TTN platform. Then comment this line."
#define SF_RUSE_DEV_DEFAULT

/**
* @brief Pin of the battery.
*/
#define PIN_VBAT A9

/**
* @brief Cayenne message size.
*/
#define CAYENNE_SIZE 51

/**
* @brief Schedule TX every this many seconds (might become longer due to duty cycle limitations).
* 900 seconds/ 15minutes.
*/
#define TX_INTERVAL 10U

/**
* @brief Debug LED flag.
*/
#define DEBUG_LED

#pragma endregion

#pragma region Headers

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <CayenneLPP.h>

#pragma endregion

#pragma region Constants

#ifdef SF_RUSE_DEV_DEFAULT

/**
* @brief LoRaWAN NwkSKey, network session key This is the default Semtech key, which is used by the prototype TTN network initially.
*/
static const PROGMEM u1_t NWKSKEY[16] = { 0x82, 0xAC, 0xA1, 0x61, 0x33, 0xC4, 0x0D, 0x38, 0x50, 0xD9, 0xF7, 0x68, 0xB4, 0xC7, 0xC6, 0x49 };

/**
* @brief LoRaWAN AppSKey, application session key This is the default Semtech key, which is used by the prototype TTN network initially.
*/
static const u1_t PROGMEM APPSKEY[16] = { 0xB2, 0x5A, 0x62, 0x6B, 0x82, 0x4D, 0x9F, 0x91, 0xCD, 0x64, 0x22, 0x9F, 0x6C, 0xF4, 0x6B, 0xC1 };

/**
* @brief LoRaWAN end-device address (DevAddr)
* @see http://thethingsnetwork.org/wiki/AddressSpace
*/
static const u4_t DEVADDR = 0x260111E8;

#endif

/**
* @brief Pin mapping for the Feather32U4.
* @see lmic_pinmap
*/
const lmic_pinmap lmic_pins = {
	.nss = 8,
	.rxtx = LMIC_UNUSED_PIN,
	.rst = 4,
	.dio = { 7, 6, LMIC_UNUSED_PIN },
};

#pragma endregion

#pragma region Prototypes

/**
* @brief On OS event handler.
*  @param ev, event data.
*  @return Void.
*/
void onEvent(ev_t ev);

/**
* @brief Do send job handler.
* @param j, job data.
* @return Void.
*/
void do_send(osjob_t* j);

/**
* @brief OS get art EUI.
* @param buf, data buffer.
* @return Void.
*/
void os_getArtEui(u1_t* buf);

/**
* @brief OS get device EUI.
* @param buf, data buffer.
* @return Void.
*/
void os_getDevEui(u1_t* buf);

/**
* @brief OS get device key.
* @param buf, data buffer.
* @return Void.
*/
void os_getDevKey(u1_t* buf);

/**
 * @brief Measure battery voltage.
 * @return float Battery voltage.
 */
float get_battery_voltage();

#ifdef DEBUG_LED

/**
 * @brief Blink once the debug LED.
 * @return Void.
 */
void blink_led();

#endif // DEBUG_LED

#pragma endregion

#pragma region Variables

/**
 * @brief OS Send job handle.
 */
static osjob_t SendJob_g;

/**
 * @brief Cayenne Low Power Protocol container.
 */
CayenneLPP LPP_g(CAYENNE_SIZE);

#pragma endregion

/**
 * @brief The setup function runs once when you press reset or power the board.
 * @return Void.
 */
void setup()
{

#ifdef DEBUG_LED
	pinMode(LED_BUILTIN, OUTPUT);
#endif

	Serial.begin(115200);
	while (!Serial)
	{
		delay(1);
	}
	Serial.println("Starting");

#ifdef VCC_ENABLE
	// For Pinoccio Scout boards
	pinMode(VCC_ENABLE, OUTPUT);
	digitalWrite(VCC_ENABLE, HIGH);
	delay(1000);
#endif

	// LMIC initialize.
	os_init();

	// Reset the MAC state. Session and pending data transfers will be discarded.
	LMIC_reset();

	// Read about this. line fo code.
	// https://www.thethingsnetwork.org/forum/t/got-adafruit-feather-32u4-lora-radio-to-work-and-here-is-how/6863/44
	LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

	// Set static session parameters. Instead of dynamically establishing a session
	// by joining the network, precomputed session parameters are be provided.
#ifdef PROGMEM
	// On AVR, these values are stored in flash and only copied to RAM
	// once. Copy them to a temporary buffer here, LMIC_setSession will
	// copy them into a buffer of its own again.
	uint8_t appskey[sizeof(APPSKEY)];
	uint8_t nwkskey[sizeof(NWKSKEY)];
	memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
	memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
	LMIC_setSession(0x1, DEVADDR, nwkskey, appskey);
#else
	// If not running an AVR with PROGMEM, just use the arrays directly
	LMIC_setSession(0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif

#if defined(CFG_eu868)
	// Set up the channels used by the Things Network, which corresponds
	// to the defaults of most gateways. Without this, only three base
	// channels from the LoRaWAN specification are used, which certainly
	// works, so it is good for debugging, but can overload those
	// frequencies, so be sure to configure the full frequency range of
	// your network here (unless your network autoconfigures them).
	// Setting up channels should happen after LMIC_setSession, as that
	// configures the minimal channel set.
	// NA-US channels 0-71 are configured automatically
	LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);      // g-band
	//LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
	//LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	//LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	//LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	//LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	//LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	//LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	//LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
	
	//
	LMIC_disableChannel(1);
	LMIC_disableChannel(2);

	// TTN defines an additional channel at 869.525Mhz using SF9 for class B
	// devices' ping slots. LMIC does not have an easy way to define set this
	// frequency and support for class B is spotty and untested, so this
	// frequency is not configured here.
#elif defined(CFG_us915)
	// NA-US channels 0-71 are configured automatically
	// but only one group of 8 should (a subband) should be active
	// TTN recommends the second sub band, 1 in a zero based count.
	// https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
	LMIC_selectSubBand(1);
#endif

	// Disable link check validation
	LMIC_setLinkCheckMode(0);

	// TTN uses SF9 for its RX2 window.
	LMIC.dn2Dr = DR_SF9;

	// Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
	LMIC_setDrTxpow(DR_SF7, 14);

	// Start job
	do_send(&SendJob_g);
}

/**
 * @brief The loop function runs over and over again until power down or reset.
 * @return Void.
 */
void loop()
{
	os_runloop_once();
}

#pragma region Functions

/*
These callbacks are only used in over-the-air activation, so they are
left empty here (we cannot leave them out completely unless
DISABLE_JOIN is set in config.h, otherwise the linker will complain).
*/

/** @brief OS get art EUI.
*  @param buf, data buffer.
*  @return Void.
*/
void os_getArtEui(u1_t* buf) { }

/**
* @brief OS get device EUI.
* @param buf, data buffer.
* @return Void.
*/
void os_getDevEui(u1_t* buf) { }

/**
* @brief OS get device key.
* @param buf, data buffer.
* @return Void.
*/
void os_getDevKey(u1_t* buf) { }

/**
* @brief On OS event handler.
* @param ev, event data.
* @return Void.
*/
void onEvent(ev_t ev)
{
	Serial.print(os_getTime());
	Serial.print(": ");
	switch (ev) {
	case EV_SCAN_TIMEOUT:
		Serial.println(F("EV_SCAN_TIMEOUT"));
		break;
	case EV_BEACON_FOUND:
		Serial.println(F("EV_BEACON_FOUND"));
		break;
	case EV_BEACON_MISSED:
		Serial.println(F("EV_BEACON_MISSED"));
		break;
	case EV_BEACON_TRACKED:
		Serial.println(F("EV_BEACON_TRACKED"));
		break;
	case EV_JOINING:
		Serial.println(F("EV_JOINING"));
		break;
	case EV_JOINED:
		Serial.println(F("EV_JOINED"));
		break;
	case EV_RFU1:
		Serial.println(F("EV_RFU1"));
		break;
	case EV_JOIN_FAILED:
		Serial.println(F("EV_JOIN_FAILED"));
		break;
	case EV_REJOIN_FAILED:
		Serial.println(F("EV_REJOIN_FAILED"));
		break;
	case EV_TXCOMPLETE:
		Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
		if (LMIC.txrxFlags & TXRX_ACK)
			Serial.println(F("Received ack"));
		if (LMIC.dataLen) {
			Serial.println(F("Received "));
			Serial.println(LMIC.dataLen);
			Serial.println(F(" bytes of payload"));
		}

		// Schedule next transmission
		os_setTimedCallback(&SendJob_g, os_getTime() + sec2osticks(TX_INTERVAL), do_send);

		break;
	case EV_LOST_TSYNC:
		Serial.println(F("EV_LOST_TSYNC"));
		break;
	case EV_RESET:
		Serial.println(F("EV_RESET"));
		break;
	case EV_RXCOMPLETE:
		// data received in ping slot
		Serial.println(F("EV_RXCOMPLETE"));
		break;
	case EV_LINK_DEAD:
		Serial.println(F("EV_LINK_DEAD"));
		break;
	case EV_LINK_ALIVE:
		Serial.println(F("EV_LINK_ALIVE"));
		break;
	default:
		Serial.println(F("Unknown event"));
		break;
	}
}

/**
* @brief Do send job handler.
* @param j, job data.
* @return Void.
*/
void do_send(osjob_t* j)
{
	// Check if there is not a current TX/RX job running
	if (LMIC.opmode & OP_TXRXPEND)
	{
		Serial.println(F("OP_TXRXPEND, not sending"));
	}
	else
	{
		// Create LPP package.
		LPP_g.reset();
		LPP_g.addAnalogInput(0, get_battery_voltage());
		LPP_g.addAnalogInput(1, (float)LMIC.txCnt);

		// Send
		LMIC_setTxData2(1, LPP_g.getBuffer(), LPP_g.getSize(), 0);

		// Debug message.
		Serial.println(F("Packet queued"));

#ifdef DEBUG_LED
		blink_led();
#endif
	}
	// Next TX is scheduled after TX_COMPLETE event.
}

/**
* @brief Measure battery voltage.
* @return float Battery voltage.
*/
float get_battery_voltage()
{
	float MeasuredBatteryVoltageL = analogRead(PIN_VBAT);

	// We divided by 2, so multiply back.
	MeasuredBatteryVoltageL *= 2;

	// Multiply by 3.3V, our reference voltage.
	MeasuredBatteryVoltageL *= 3.3;

	// Convert to voltage.
	MeasuredBatteryVoltageL /= 1024;

	return MeasuredBatteryVoltageL;
}

#ifdef DEBUG_LED

/**
* @brief Blink once the debug LED.
* @return Void.
*/
void blink_led()
{
	digitalWrite(LED_BUILTIN, HIGH);
	delay(100);
	digitalWrite(LED_BUILTIN, LOW);
}

#endif // DEBUG_LED

#pragma endregion
