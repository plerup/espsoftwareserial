#include "SoftwareSerial.h"

#ifndef D5
#if defined(ESP8266)
#define D5 (14)
#define D6 (12)
#elif defined(ESP32)
#define D5 (18)
#define D6 (19)
#endif
#endif

SoftwareSerial swSer;
#ifdef ESP8266
auto logSer = Serial1;
auto hwSer = Serial;
#else
auto logSer = Serial;
auto hwSer = Serial1;
#endif

void setup() {
	delay(2000);
	logSer.begin(115200);
#ifdef ESP8266
	hwSer.begin(115200, SERIAL_8N1);
	hwSer.swap();
#else
	hwSer.begin(115200, SERIAL_8N1, -1, D5);
#endif
	logSer.println(PSTR("\nOne Wire Half Duplex Bitpattern and Datarate Test"));
	swSer.begin(115200, SWSERIAL_8N1, D6, -1);
	swSer.enableIntTx(true);
	logSer.println(PSTR("Tx on hwSer"));
}

uint8_t val = 0xff;

void loop() {
	hwSer.write((uint8_t)0x00);
	hwSer.write(val);
	hwSer.write(val);
	auto start = ESP.getCycleCount();
	int rxCnt = 0;
	while (ESP.getCycleCount() - start < ESP.getCpuFreqMHz() * 1000000 / 10) {
		if (swSer.available()) {
			auto rxVal = swSer.read();
			if ((!rxCnt && rxVal) || (rxCnt && rxVal != val)) {
				logSer.printf(PSTR("Rx bit error: tx = 0x%02x, rx = 0x%02x\n"), val, rxVal);
			}
			++rxCnt;
		}
	}
	if (rxCnt != 3) {
		logSer.printf(PSTR("Rx cnt error, tx = 0x%02x\n"), val);
	}
	++val;
	if (!val) {
		logSer.println("Starting over");
	}
}
