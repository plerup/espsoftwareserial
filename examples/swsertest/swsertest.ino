// On ESP8266:
// Runs at both 80MHz and 160MHz CPU frequency at 56000bps without visible errors.
// Connect pin 12 to 14.

#include <SoftwareSerial.h>

#ifdef ESP32
#define BAUD_RATE 56000
#else
#define BAUD_RATE 56000
#endif

SoftwareSerial swSer(14, 12, false, 93, 10);

void setup() {
  Serial.begin(115200);
  swSer.begin(BAUD_RATE);

  // ESP8266 internal cache RAM needs warm up - allow write and ISR to load
  swSer.write(static_cast<uint8_t>(0));
  Serial.println("\nSoftware serial test started");

  for (char ch = ' '; ch <= 'z'; ch++) {
    swSer.write(ch);
  }
  swSer.println("");
}

void loop() {
  while (swSer.available() > 0) {
    Serial.write(swSer.read());
    yield();
  }
  while (Serial.available() > 0) {
    swSer.write(Serial.read());
    yield();
  }

}
