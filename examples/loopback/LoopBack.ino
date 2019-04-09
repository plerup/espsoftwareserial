//#ifdef ESP8266
//#include <ESP8266WiFi.h>
//#endif
//#ifdef ESP32
//#include "WiFi.h"
//#endif

#include <SoftwareSerial.h>

// On ESP8266:
// Local SoftwareSerial loopback, connect D5 (14) to D6 (12), or with repeater, connect crosswise.
// or hardware loopback, connect D5 to D8 (tx), D6 to D7 (rx).
// Hint: The logger is run at 9600bps such that enableIntTx(true) can remain unchanged. Blocking
// interrupts severely impacts the ability of the SoftwareSerial devices to operate concurrently
// and/or in duplex mode.
// By default (no HWLOOPBACK, no HALFDUPLEX),
// runs at 80MHz with 22800bps, and at 160MHz CPU frequency with 57600bps with no errors.

// Pick only one of HWLOOPBACK OR HWSENDNSINK
//#define HWLOOPBACK 1
//#define HWSENDNSINK 1
//#define HALFDUPLEX 1

#ifndef RX
#define RX 13
#endif
#ifndef TX
#define TX 15
#endif

#ifdef ESP32
constexpr int IUTBITRATE = 28800;
#else
constexpr int IUTBITRATE = 2400;
#endif

constexpr SoftwareSerialConfig swSerialConfig = SWSERIAL_8N1;

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
SoftwareSerial ssLogger(RX, TX);
Stream& logger(ssLogger);
#else
Stream& logger(Serial);
#endif

#ifdef HWSENDNSINK
Stream& serialIUT(Serial);
#else
SoftwareSerial serialIUT(14, 12, false, 2 * BLOCKSIZE);
#endif

void setup() {

	//WiFi.mode(WIFI_OFF);
	//WiFi.forceSleepBegin();
	//delay(1);

#if defined(HWLOOPBACK) || defined(HWSENDNSINK)
	Serial.begin(IUTBITRATE);
	Serial.setRxBufferSize(4 * BLOCKSIZE);
	Serial.swap();
	ssLogger.begin(9600);
#else
	Serial.begin(9600);
#endif

#if !defined(HWSENDNSINK)
	serialIUT.begin(IUTBITRATE, swSerialConfig);
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
		while (0 == (i % 8) && Serial.available()) {
			int inCnt = Serial.readBytes(inBuf, min(BLOCKSIZE, Serial.availableForWrite()));
			Serial.write(inBuf, inCnt);
		}
#endif
	}
	serialIUT.write(block, BLOCKSIZE);
#ifdef HWSENDNSINK
	if (Serial.hasOverrun()) { logger.println("Serial::overrun"); }
#else
	if (serialIUT.overflow()) { logger.println("SoftwareSerial::overflow"); }
#endif

	uint32_t deadline;
	int inCnt;

#ifdef HWLOOPBACK
	// starting deadline for the first bytes to become readable
	deadline = ESP.getCycleCount() + static_cast<uint32_t>(1000000 / IUTBITRATE * ESP.getCpuFreqMHz() * 10 * BLOCKSIZE);
	inCnt = 0;
	while (static_cast<int32_t>(deadline - ESP.getCycleCount()) > 0) {
		if (!Serial.available()) {
			delay(100);
			continue;
		}
		inCnt += Serial.readBytes(&inBuf[inCnt], min(BLOCKSIZE - inCnt, Serial.availableForWrite()));
		if (inCnt >= BLOCKSIZE) { break; }
		// wait for more outstanding bytes to trickle in
		deadline = ESP.getCycleCount() + static_cast<uint32_t>(200 * 1000 * ESP.getCpuFreqMHz());
	}
	Serial.write(inBuf, inCnt);
#endif

	// starting deadline for the first bytes to come in
	deadline = ESP.getCycleCount() + static_cast<uint32_t>(2 * 1000000 / IUTBITRATE * ESP.getCpuFreqMHz() * 10 * BLOCKSIZE);
	inCnt = 0;
	while (static_cast<int32_t>(deadline - ESP.getCycleCount()) > 0) {
		int avail;
		if (0 == (avail = serialIUT.available())) {
			delay(100);
			continue;
		}
		for (int i = 0; i < avail; ++i) {
			int r = serialIUT.read();
			if (r == -1) { logger.println("read() == -1"); }
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
		deadline = ESP.getCycleCount() + static_cast<uint32_t>(200 * 1000 * ESP.getCpuFreqMHz());
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
