#include <SoftwareSerial.h>

// On ESP8266:
// Local SoftwareSerial loopback, connect D5 to D6, or with repeater, connect crosswise.
// For hardware loopback, connect D5 to D8 (tx), D6 to D7 (rx).
// Hint: The logger is run at 9600bps such that enableIntTx(true) can remain unchanged. Blocking
// interrupts severely impacts the ability of the SoftwareSerial devices to operate concurrently
// and/or in duplex mode.
// By default (no HWLOOPBACK, no HALFDUPLEX),
// runs at 80MHz with 28800bps, and at 160MHz CPU frequency with 38400bps with no errors.
// On ESP32:
// Hardware Serial2 defaults to D4 (rx), D3 (tx).
// For hardware loopback, connect D5 to D3 (tx), D6 to D4 (rx).

#if defined(ESP32) && !defined(ARDUINO_D1_MINI32)
#define D5 (14)
#define D6 (12)
#define D7 (13)
#define D8 (15)
#endif

// Pick only one of HWLOOPBACK OR HWSENDNSINK
//#define HWLOOPBACK 1
//#define HWSENDNSINK 1
#define HALFDUPLEX 1

#ifdef ESP32
constexpr int IUTBITRATE = 57600;
#else
constexpr int IUTBITRATE = 57600;
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
constexpr int ReportInterval = IUTBITRATE / 16;

#if defined(ESP8266)
#if defined(HWLOOPBACK)
HardwareSerial& hwLoopback(Serial);
SoftwareSerial serialIUT;
SoftwareSerial logger;
#elif defined(HWSENDNSINK)
HardwareSerial& serialIUT(Serial);
SoftwareSerial logger;
#else
SoftwareSerial serialIUT;
HardwareSerial& logger(Serial);
#endif
#elif defined(ESP32)
#if defined(HWLOOPBACK)
HardwareSerial& hwLoopback(Serial2);
SoftwareSerial serialIUT;
#elif defined(HWSENDNSINK)
HardwareSerial& serialIUT(Serial2);
#else
SoftwareSerial serialIUT;
#endif
HardwareSerial& logger(Serial);
#else
SoftwareSerial serialIUT(14, 12);
HardwareSerial& logger(Serial);
#endif

void setup() {
#if defined(ESP8266)
#if defined(HWLOOPBACK) || defined(HWSENDNSINK)
	Serial.begin(IUTBITRATE);
	Serial.swap();
	Serial.setRxBufferSize(4 * BLOCKSIZE);
	logger.begin(9600, RX, TX);
#else
	Serial.begin(9600);
#endif
#elif defined(ESP32)
#if defined(HWLOOPBACK)
	Serial2.begin(IUTBITRATE);
	Serial2.setRxBufferSize(4 * BLOCKSIZE);
	logger.begin(9600);
#elif defined(HWSENDNSINK)
	serialIUT.begin(IUTBITRATE, SERIAL_8N1, D5, D6);
	serialIUT.setRxBufferSize(4 * BLOCKSIZE);
	logger.begin(9600);
#else
Serial.begin(9600);
#endif
#else
	Serial.begin(9600);
#endif

#if !defined(HWSENDNSINK)
#if defined(ESP8266) || defined(ESP32)
	serialIUT.begin(IUTBITRATE, D5, D6, swSerialConfig, false, 2 * BLOCKSIZE);
#ifdef HALFDUPLEX
	serialIUT.enableIntTx(false);
#endif
#else
	serialIUT.begin(IUTBITRATE);
#endif
#endif

	start = micros();
	txCount = 0;
	rxCount = 0;
	rxErrors = 0;

	logger.println("Loopback example for EspSoftwareSerial");
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
#ifndef HALFDUPLEX
		serialIUT.write(c);
#ifdef HWLOOPBACK
		int avail = hwLoopback.available();
		while ((0 == (i % 8)) && avail > 0) {
			int inCnt = hwLoopback.readBytes(inBuf, min(avail, min(BLOCKSIZE, hwLoopback.availableForWrite())));
			hwLoopback.write(inBuf, inCnt);
			avail -= inCnt;
		}
#endif
	}
#else
	}
	serialIUT.write(block, BLOCKSIZE);
#endif
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
	deadline = micros() + static_cast<uint32_t>(8 * 1000000 / IUTBITRATE * 10 * BLOCKSIZE);
	inCnt = 0;
	while (static_cast<int32_t>(deadline - micros()) > 0) {
		int avail = hwLoopback.available();
		if (0 >= avail) {
			delay(1);
			continue;
		}
		inCnt += hwLoopback.readBytes(&inBuf[inCnt], min(avail, min(BLOCKSIZE - inCnt, hwLoopback.availableForWrite())));
		if (inCnt >= BLOCKSIZE) { break; }
		// wait for more outstanding bytes to trickle in
		deadline = micros() + static_cast<uint32_t>(1000000 / IUTBITRATE * 10 * BLOCKSIZE);
	}
	hwLoopback.write(inBuf, inCnt);
#endif

	// starting deadline for the first bytes to come in
	deadline = micros() + static_cast<uint32_t>(128 * 1000000 / IUTBITRATE * 10 * BLOCKSIZE);
	inCnt = 0;
	while (static_cast<int32_t>(deadline - micros()) > 0) {
		int avail;
		if (0 >= (avail = serialIUT.available())) {
			delay(1);
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
		deadline = micros() + static_cast<uint32_t>(128 * 1000000 / IUTBITRATE * 10 * BLOCKSIZE);
	}

	if (inCnt != BLOCKSIZE) {
		logger.print("Got "); logger.print(inCnt); logger.println(" bytes during block loopback interval");
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
