//#ifdef ESP8266
//#include <ESP8266WiFi.h>
//#endif
//#ifdef ESP32
//#include "WiFi.h"
//#endif

#include <SoftwareSerial.h>

// On ESP8266:
// Local SoftwareSerial loopback, connect D5 (14) to D6 (12), or with repeater, connect crosswise.
// or hardware loopback, connect D5 (14) to D8 (tx, 15), D6 (12) to D7 (rx, 13).
// Hint: The logger is run at 9600bps such that enableIntTx(true) can remain unchanged. Blocking
// interrupts severely impacts the ability of the SoftwareSerial devices to operate concurrently
// and/or in duplex mode.
// By default (no HWLOOPBACK, no HALFDUPLEX),
// runs at 80MHz with 28800bps, and at 160MHz CPU frequency with 38400bps with no errors.

// Pick only one of HWLOOPBACK OR HWSENDNSINK
//#define HWLOOPBACK 1
//#define HWSENDNSINK 1
//#define HALFDUPLEX 1

#ifdef ESP32
constexpr int IUTBITRATE = 38400;
#else
constexpr int IUTBITRATE = 28800;
#endif

#if defined(ESP8266) || defined(ESP32)
constexpr SoftwareSerialConfig swSerialConfig = SWSERIAL_8N1;
#else
constexpr unsigned swSerialConfig = 3;
#endif

constexpr int BLOCKSIZE = 16; // use fractions of 256

unsigned long start;
String effTxTxt("eff. tx: ");
String effRxTxt("eff. rx: ");
int txCount;
int rxCount;
int expected;
int rxErrors;
constexpr int ReportInterval = IUTBITRATE / 20;

#if defined(HWLOOPBACK) || defined(HWSENDNSINK)
#if defined(ESP8266)
SoftwareSerial logger;
HardwareSerial& hwLoopback(Serial);
#elif defined(ESP32)
Stream& logger(Serial);
HardwareSerial& hwLoopback(Serial1);
#endif
#else
Stream& logger(Serial);
#endif

#ifdef HWSENDNSINK
Stream& serialIUT(Serial);
#elif defined(ESP8266) || defined(ESP32)
SoftwareSerial serialIUT;
#else
SoftwareSerial serialIUT(14, 12);
#endif

void setup() {

	//WiFi.mode(WIFI_OFF);
	//WiFi.forceSleepBegin();
	//delay(1);

#if defined(HWLOOPBACK) || defined(HWSENDNSINK)
#if defined(ESP8266)
	Serial.begin(IUTBITRATE);
	Serial.setRxBufferSize(4 * BLOCKSIZE);
	Serial.swap();
	logger.begin(9600, RX, TX);
#elif defined(ESP32)
	Serial.begin(9600);
	Serial1.begin(IUTBITRATE, SERIAL_8N1, 13, 15, false, 200);
	Serial1.setRxBufferSize(4 * BLOCKSIZE);
#else
	Serial.begin(9600);
#endif
#else
	Serial.begin(9600);
#endif

#if !defined(HWSENDNSINK)
#if defined(ESP8266) || defined(ESP32)
	serialIUT.begin(IUTBITRATE, 14, 12, swSerialConfig, false, 2 * BLOCKSIZE);
#else
	serialIUT.begin(IUTBITRATE);
#endif
#endif

	start = micros();
	txCount = 0;
	rxCount = 0;
	rxErrors = 0;
}

unsigned char c = 0;

void loop() {
	expected = -1;

	unsigned char block[BLOCKSIZE];
	unsigned char inBuf[BLOCKSIZE];
	for (int i = 0; i < BLOCKSIZE; ++i) {
		block[i] = c;
		c = (c + 1) % 256;
		++txCount;
		//serialIUT.write(c);
#if defined(HWLOOPBACK) && !defined(HALFDUPLEX)
		int avail = hwLoopback.available();
		while ((0 == (i % 8)) && avail > 0) {
			int inCnt = hwLoopback.readBytes(inBuf, min(avail, min(BLOCKSIZE, hwLoopback.availableForWrite())));
			hwLoopback.write(inBuf, inCnt);
			avail -= inCnt;
		}
#endif
	}
	serialIUT.write(block, BLOCKSIZE);
#ifdef HWSENDNSINK
#if defined(ESP8266)
	if (Serial.hasOverrun()) { logger.println("Serial::overrun"); }
#endif
#else
	if (serialIUT.overflow()) { logger.println("SoftwareSerial::overflow"); }
#endif

	uint32_t deadline;
	int inCnt;

#ifdef HWLOOPBACK
	// starting deadline for the first bytes to become readable
	deadline = micros() + static_cast<uint32_t>(1000000 / IUTBITRATE * 10 * BLOCKSIZE);
	inCnt = 0;
	while (static_cast<int32_t>(deadline - micros()) > 0) {
		int avail = hwLoopback.available();
		if (0 >= avail) {
			continue;
		}
		inCnt += hwLoopback.readBytes(&inBuf[inCnt], min(avail, min(BLOCKSIZE - inCnt, hwLoopback.availableForWrite())));
		if (inCnt >= BLOCKSIZE) { break; }
		// wait for more outstanding bytes to trickle in
		deadline = micros() + 200000U;
	}
	hwLoopback.write(inBuf, inCnt);
#endif

	// starting deadline for the first bytes to come in
	deadline = micros() + static_cast<uint32_t>(2 * 1000000 / IUTBITRATE * 10 * BLOCKSIZE);
	inCnt = 0;
	while (static_cast<int32_t>(deadline - micros()) > 0) {
		int avail;
		if (0 == (avail = serialIUT.available())) {
			continue;
		}
		avail = serialIUT.readBytes(inBuf, BLOCKSIZE);
		for (int i = 0; i < avail; ++i) {
			unsigned char r = inBuf[i];
			if (expected == -1) { expected = r; }
			else {
				expected = (expected + 1) % 256;
			}
			if (r != (expected & ((1 << (5 + swSerialConfig % 4)) - 1))) {
				++rxErrors;
			}
			++rxCount;
			++inCnt;
		}
		if (inCnt >= BLOCKSIZE) { break; }
		// wait for more outstanding bytes to trickle in
		deadline = micros() + 200000U;
	}

	if (txCount >= ReportInterval) {
		logger.println(String("tx/rx: ") + txCount + "/" + rxCount);
		const auto end = micros();
		const unsigned long interval = end - start;
		const long txCps = txCount * (1000000.0 / interval);
		const long rxCps = rxCount * (1000000.0 / interval);
		const long errorCps = rxErrors * (1000000.0 / interval);
		logger.println(effTxTxt + 10 * txCps + "bps, "
			+ effRxTxt + 10 * rxCps + "bps, "
			+ errorCps + "cps errors (" + 100.0 * rxErrors / rxCount + "%)");
		start = end;
		txCount = 0;
		rxCount = 0;
		rxErrors = 0;
		expected = -1;
	}
}
