#include <SoftwareSerial.h>

SoftwareSerial repeater(D5, D6);
unsigned long start;
String bitRateTxt("\tBitrate: ");
int rxCount;
constexpr int ReportInterval = 100000;

void setup()
{
	Serial.begin(115200);
	repeater.begin(115200);
	repeater.enableIntTx(true);
	start = micros();
	rxCount = 0;
}

void loop()
{
	while (repeater.available())
	{
		int r = repeater.read();
		++rxCount;
		repeater.write(r);
		if (rxCount >= ReportInterval) break;
	}

	if (rxCount >= ReportInterval) {
		auto end = micros();
		auto cps = 1000000 / ((end - start) / rxCount);
		Serial.println(bitRateTxt + 10 * cps + "bps");
		start = end;
		rxCount = 0;
	}
}
