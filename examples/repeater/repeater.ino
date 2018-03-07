#include <SoftwareSerial.h>

SoftwareSerial repeater(D5, D6);
unsigned long start;
String bitRateTxt("\tBitrate: ");
int rxCount;
int seqErrors;
int expected;
constexpr int ReportInterval = 100000;

void setup()
{
	Serial.begin(115200);
	repeater.begin(115200);
	repeater.enableIntTx(true);
	start = micros();
	rxCount = 0;
	seqErrors = 0;
	expected = -1;
}

void loop()
{
	while (repeater.available())
	{
		int r = repeater.read();
		if (expected == -1) expected = r;
		else
		{
			expected = ++expected % 256;
		}
		if (r != expected) ++seqErrors;
		++rxCount;
		repeater.write(r);
		if (rxCount >= ReportInterval) break;
	}

	if (rxCount >= ReportInterval) {
		auto end = micros();
		auto cps = 1000000 / ((end - start) / rxCount);
		Serial.println(bitRateTxt + 10 * cps + "bps, " + seqErrors / (end - start) + "seq. errors/s");
		start = end;
		rxCount = 0;
		expected = -1;
		seqErrors = 0;
	}
}
