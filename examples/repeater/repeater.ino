#include <SoftwareSerial.h>

// On ESP8266:
// SoftwareSerial loopback for remote source (loopback.ino), or hardware loopback.
// Connect source D5 to local D8 (tx), source D6 to local D7 (rx).
// Hint: The logger is run at 9600bps such that enableIntTx(true) can remain unchanged. Blocking
// interrupts severely impacts the ability of the SoftwareSerial devices to operate concurrently
// and/or in duplex mode.
// On ESP32:
// Hardware Serial2 defaults to D4 (rx), D3 (tx).
// Connect source D5 to local D3 (tx), source D6 to local D4 (rx).

#if defined(ESP32) && !defined(ARDUINO_D1_MINI32)
#define D5 (14)
#define D6 (12)
#define D7 (13)
#define D8 (15)
#endif

//#define HWLOOPBACK 1
#define HALFDUPLEX 1

#ifdef ESP32
constexpr int IUTBITRATE = 57600;
#else
constexpr int IUTBITRATE = 57600;
#endif

constexpr SoftwareSerialConfig swSerialConfig = SWSERIAL_8N1;

constexpr int BLOCKSIZE = 16; // use fractions of 256


unsigned long start;
String bitRateTxt("Effective data rate: ");
int rxCount;
int seqErrors;
int expected;
constexpr int ReportInterval = IUTBITRATE / 16;

#ifdef HWLOOPBACK
#if defined(ESP8266)
HardwareSerial& repeater(Serial);
SoftwareSerial logger;
#elif defined(ESP32)
HardwareSerial& repeater(Serial2);
HardwareSerial& logger(Serial);
#endif
#else
SoftwareSerial repeater;
HardwareSerial& logger(Serial);
#endif

void setup() {
#ifdef HWLOOPBACK
#if defined(ESP8266)
	Serial.begin(IUTBITRATE);
	Serial.setRxBufferSize(2 * BLOCKSIZE);
	Serial.swap();
	logger.begin(9600, RX, TX);
#elif defined(ESP32)
	Serial2.begin(IUTBITRATE);
	Serial2.setRxBufferSize(2 * BLOCKSIZE);
	logger.begin(9600);
#endif
#else
#if defined(ESP8266)
	repeater.begin(IUTBITRATE, D7, D8, swSerialConfig, false, 2 * BLOCKSIZE);
#elif defined(ESP32)
	repeater.begin(IUTBITRATE, D4, D3, swSerialConfig, false, 2 * BLOCKSIZE);
#endif
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
	deadline = micros() + static_cast<uint32_t>(1000000 / IUTBITRATE * 10 * BLOCKSIZE);
	while (static_cast<int32_t>(deadline - micros()) > 0) {
		if (0 >= repeater.available()) {
			delay(1);
			continue;
		}
#else
	while (0 < repeater.available()) {
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
		deadline = micros() + static_cast<uint32_t>(1000000 / IUTBITRATE * 10 * BLOCKSIZE);
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