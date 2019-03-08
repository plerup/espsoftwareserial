//#ifdef ESP8266
//#include <ESP8266WiFi.h>
//#endif
//#ifdef ESP32
//#include "WiFi.h"
//#endif

#include <SoftwareSerial.h>

// SoftwareSerial loopback for remote source (loopback.ino),
// or hardware loopback, connect source D5 to local D8 (tx), source D6 to local D7 (rx).
#define HWLOOPBACK 1
//#define HALFDUPLEX 1

constexpr int SWSERBITRATE = 28800;

constexpr int BLOCKSIZE = 16; // use fractions of 256


unsigned long start;
String bitRateTxt("Effective data rate: ");
int rxCount;
int seqErrors;
int expected;
constexpr int ReportInterval = 10000;

#ifdef HWLOOPBACK
Stream& repeater(Serial);
SoftwareSerial ssLogger(RX, TX);
Stream& logger(ssLogger);
#else
SoftwareSerial repeater(D7, D8);
Stream& logger(Serial);
#endif

void setup()
{
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
	repeater.begin(SWSERBITRATE);
#ifdef HALFDUPLEX
	repeater.enableIntTx(false);
#endif
	Serial.begin(115200);
#endif

	start = micros();
	rxCount = 0;
	seqErrors = 0;
	expected = -1;
}

void loop()
{
#ifdef HALFDUPLEX
	unsigned char block[BLOCKSIZE];
	int inCnt = 0;
#endif
	expected = -1;
	while (repeater.available())
	{
		int r = repeater.read();
		if (r == -1) { logger.println("read() == -1"); }
		if (expected == -1) expected = r;
		else
		{
			expected = ++expected % 256;
		}
#if HALFDUPLEX
		block[inCnt++] = expected;
#else
		repeater.write(expected);
#endif
		if (r != expected) {
			++seqErrors;
		}
		++rxCount;
		if (rxCount >= ReportInterval) break;
	}

#ifdef HALFDUPLEX
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