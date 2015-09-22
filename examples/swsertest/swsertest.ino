
#include <SoftwareSerial.h>

SoftwareSerial swSer(14, 12, 128);

void setup() {
  Serial.begin(115200);
  swSer.begin(115200);

  Serial.println("\nSoftware serial test started");

  for (char ch = ' '; ch <= 'z'; ch++) {
    swSer.write(ch);
  }
  swSer.println("");

}

void loop() {
  if (swSer.available()) {
    Serial.write(swSer.read());
  }
  if (Serial.available()) {
    swSer.write(Serial.read());
  }

}
