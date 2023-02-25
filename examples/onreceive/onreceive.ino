// On ESP8266:
// At 80MHz runs up 57600ps, and at 160MHz CPU frequency up to 115200bps with only negligible errors.
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

#define BAUD_RATE 115200

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
	testSerial.onReceive(receiveHandler);

	testSerial.println(PSTR("\nSoftware serial onReceive() event test started"));

	for (char ch = ' '; ch <= 'z'; ch++) {
		testSerial.write(ch);
	}
	testSerial.println();
}

void loop() {
	if (rxPending && !testSerial.available()) {
		delayMicroseconds(1 + (1 + 8 + 1) * 1000000 / BAUD_RATE);
	}
	rxPending = testSerial.available() > 0;
	if (!rxPending) {
		// On development board, idle power draw at USB:
		// with yield() 77mA, 385mW
		// with esp_suspend() 20mA, 100mW
		//yield();
		esp_suspend();
	}
	else {
		do {
			testSerial.write(testSerial.read());
			rxPending = testSerial.available() > 0;
		} while (rxPending);
		testSerial.println();
	}
}
