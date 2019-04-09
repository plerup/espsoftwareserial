//#ifdef ESP8266
//#include <ESP8266WiFi.h>
//#endif
//#ifdef ESP32
//#include "WiFi.h"
//#endif

#include <SoftwareSerial.h>

// SoftwareSerial loopback for remote source (loopback.ino),
// or hardware loopback, connect source D5 to local D8 (TX, 15), source D6 to local D7 (RX, 13).
// Hint: The logger is run at 9600bps such that enableIntTx(true) can remain unchanged. Blocking
// interrupts severely impacts the ability of the SoftwareSerial devices to operate concurrently
// and/or in duplex mode.
#define HWLOOPBACK 1
#define HALFDUPLEX 1

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
String bitRateTxt("Effective data rate: ");
int rxCount;
int seqErrors;
int expected;
constexpr int ReportInterval = IUTBITRATE / 20;

#ifdef HWLOOPBACK
Stream& repeater(Serial);
SoftwareSerial ssLogger(RX, TX);
Stream& logger(ssLogger);
#else
SoftwareSerial repeater(14, 12, false, 2 * BLOCKSIZE);
Stream& logger(Serial);
#endif

void setup() {
	//WiFi.mode(WIFI_OFF);
	//WiFi.forceSleepBegin();
	//delay(1);

#ifdef HWLOOPBACK
	Serial.begin(IUTBITRATE);
	Serial.setRxBufferSize(2 * BLOCKSIZE);
	Serial.swap();
	ssLogger.begin(9600);
#else
	repeater.begin(IUTBITRATE, swSerialConfig);
#ifdef HALFDUPLEX
	repeater.enableIntTx(false);
#endif
	Serial.begin(9600);
#endif

	start = micros();
	rxCount = 0;
	seqErrors = 0;
}

void loop() {
	expected = -1;

#ifdef HALFDUPLEX
	unsigned char block[BLOCKSIZE];
	int inCnt = 0;
	uint32_t deadline;
	// starting deadline for the first bytes to come in
	deadline = ESP.getCycleCount() + static_cast<uint32_t>(4 * 1000000 / IUTBITRATE * ESP.getCpuFreqMHz() * 10 * BLOCKSIZE);
	while (static_cast<int32_t>(deadline - ESP.getCycleCount()) > 0) {
		if (!repeater.available()) {
			delay(100);
			continue;
		}
#else
	while (repeater.available()) {
#endif
		int r = repeater.read();
		if (r == -1) { logger.println("read() == -1"); }
		if (expected == -1) { expected = r; }
		else {
			expected = (expected + 1) % 256;
		}
		if (r != (expected & ((1 << (5 + swSerialConfig % 4)) - 1))) {
			++seqErrors;
		}
		++rxCount;
#if HALFDUPLEX
		block[inCnt++] = expected;
		if (inCnt >= BLOCKSIZE) { break; }
		// wait for more outstanding bytes to trickle in
		deadline = ESP.getCycleCount() + static_cast<uint32_t>(200 * 1000 * ESP.getCpuFreqMHz());
#else
		repeater.write(expected);
#endif
	}

#ifdef HALFDUPLEX
	if (inCnt != 0 && inCnt != BLOCKSIZE) {
		logger.print("Got "); logger.print(inCnt); logger.println(" bytes during buffer interval");
	}
	repeater.write(block, inCnt);
#endif

	if (rxCount >= ReportInterval) {
		auto end = micros();
		unsigned long interval = end - start;
		long cps = rxCount * (1000000.0 / interval);
		long seqErrorsps = seqErrors * (1000000.0 / interval);
		logger.println(bitRateTxt + 10 * cps + "bps, " + seqErrorsps + "cps seq. errors");
		start = end;
		rxCount = 0;
		seqErrors = 0;
		expected = -1;
	}
}