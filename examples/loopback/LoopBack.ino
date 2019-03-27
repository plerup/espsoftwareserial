//#ifdef ESP8266
//#include <ESP8266WiFi.h>
//#endif
//#ifdef ESP32
//#include "WiFi.h"
//#endif

#include <SoftwareSerial.h>

// local SoftwareSerial loopback, connect D5 (14) to D6 (12), or with repeater, connect crosswise.
// or hardware loopback, connect D5 to D8 (tx), D6 to D7 (rx).
//#define HWLOOPBACK 1
//#define HALFDUPLEX 1

#ifdef ESP32
constexpr int SWSERBITRATE = 28800;
#else
constexpr int SWSERBITRATE = 28800;
#endif

constexpr SoftwareSerialConfig swSerialConfig = SWSERIAL_8N1;

constexpr int BLOCKSIZE = 16; // use fractions of 256

SoftwareSerial swSerial(14, 12);

unsigned long start;
String effTxTxt("eff. tx: ");
String effRxTxt("eff. rx: ");
int txCount;
int rxCount;
int expected;
int rxErrors;
constexpr int ReportInterval = 10000;

#ifdef HWLOOPBACK
SoftwareSerial ssLogger(RX, TX);
Stream& logger(ssLogger);
#else
Stream& logger(Serial);
#endif

void setup() {

	//WiFi.mode(WIFI_OFF);
	//WiFi.forceSleepBegin();
	//delay(1);

#ifdef HWLOOPBACK
	Serial.begin(SWSERBITRATE);
	Serial.setRxBufferSize(2 * BLOCKSIZE);
	Serial.swap();
	ssLogger.begin(115200);
	ssLogger.enableIntTx(false);
#else
	Serial.begin(115200);
#endif

	swSerial.begin(SWSERBITRATE, swSerialConfig);
#ifdef HALFDUPLEX
	swSerial.enableIntTx(false);
#endif
	start = micros();
	txCount = 0;
	rxCount = 0;
	expected = -1;
	rxErrors = 0;
}

unsigned char c = 0;

void loop() {
	unsigned char block[BLOCKSIZE];
	for (int i = 0; i < BLOCKSIZE; ++i) {
		block[i] = c;
		//swSerial.write(c);
		c = ++c % 256;
		++txCount;
#ifdef HWLOOPBACK
		while (0 == (i % 8) && Serial.available()) { Serial.write(Serial.read()); }
#endif
	}
	swSerial.write(block, BLOCKSIZE);
	if (swSerial.overflow()) { logger.println("overflow"); }

#ifdef HWLOOPBACK
	while (Serial.available()) { Serial.write(Serial.read()); }
#endif
#ifdef HALFDUPLEX
	delay(1000 * 10 * BLOCKSIZE / 2 / SWSERBITRATE);
#endif
	int avail;
	while (0 != (avail = swSerial.available())) {
		for (int i = 0; i < avail; ++i) {
			int r = swSerial.read();
			if (r == -1) { logger.println("read() == -1"); }
			if (expected == -1) { expected = r; }
			else {
				expected = ++expected % 256;
			}
			if (r != (expected & ((1 << (5 + swSerialConfig % 4)) - 1))) {
				++rxErrors;
			}
			++rxCount;
		}
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
	int rem;
	if (0 != (rem = swSerial.available())) {
		logger.print("swSerial.available() == ");
		logger.println(rem);
	}
}
