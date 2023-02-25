// On ESP8266:
// Runs up to 128000bps at 80MHz, 250000bps at 160MHz, with nearly zero errors.
// This example is currently not ported to ESP32, which is based on FreeRTOS.

#include <SoftwareSerial.h>

#ifndef D5
#define D8 (15)
#define D5 (14)
#define D7 (13)
#define D6 (12)
#define RX (3)
#define TX (1)
#endif

#define BAUD_RATE 128000

SoftwareSerial testSerial;

bool rxPending = false;

void IRAM_ATTR receiveHandler() {
	rxPending = true;
	esp_schedule();
}

void setup() {
	Serial.begin(115200);
	Serial.setDebugOutput(false);
	Serial.swap();
	testSerial.begin(BAUD_RATE, SWSERIAL_8N1, RX, TX);
	// Only half duplex this way, but reliable TX timings for high bps
	testSerial.enableIntTx(false);
	testSerial.onReceive(receiveHandler);

	testSerial.println(PSTR("\nSoftware serial onReceive() event test started"));

	for (char ch = ' '; ch <= 'z'; ch++) {
		testSerial.write(ch);
	}
	testSerial.println();
}

void loop() {
	if (rxPending && !testSerial.available()) {
		// event fired on start bit, wait until first stop bit
		delayMicroseconds(1 + (1 + 8 + 1) * 1000000 / BAUD_RATE);
	}
	auto avail = testSerial.available();
	rxPending = avail > 0;
	if (!rxPending) {
		// On development board, idle power draw at USB:
		// with yield() 77mA, 385mW (160MHz: 82mA, 410mW)
		// with esp_suspend() 20mA, 100mW (at 160MHz, too)
		//yield();
		esp_suspend();
	}
	else {
		// try to force to half-duplex
		decltype(avail) new_avail;
		while (avail != (new_avail = testSerial.available())) {
			avail = new_avail;
		}
		do {
			testSerial.write(testSerial.read());
			avail = testSerial.available();
			rxPending = avail > 0;
		} while (rxPending);
		testSerial.println();
	}
}
