#include <SoftwareSerial.h>

SoftwareSerial loopBack(D5, D6);
unsigned long start;
String bitRateTxt("\tBitrate: ");
int txCount;
int rxCount;
int expect;
int rxErrors;
constexpr int ReportInterval = 100000;

void setup()
{
	Serial.begin(115200);
	loopBack.begin(115200);
	loopBack.enableIntTx(true);
	start = micros();
	txCount = 0;
	rxCount = 0;
	expect = -1;
	rxErrors = 0;
}

unsigned char c = 0;

void loop()
{
	if (expect == -1) expect = c;
	do {
		loopBack.write(c++);
		++txCount;
	} while (c % 16);
	while (loopBack.available())
	{
		int r = loopBack.read();
		++rxCount;
		if (r != expect++) {
			//Serial.print(String() + c + " - ");
			//Serial.println(static_cast<char>(r));
			++rxErrors;
		}
	}
	rxErrors += txCount - rxCount; // missing bytes most likely are also duplicated in mismatch rxErrors count

	if (txCount >= ReportInterval) {
		auto end = micros();
		auto cps = 1000000 / ((end - start) / rxCount);
		auto errorCps = (1000000 * rxErrors / (end - start));
		Serial.println(bitRateTxt + 10 * cps + "bps, "
			+ errorCps + "cps errors (" + 100.0 * errorCps / cps + "%)");
		start = end;
		rxCount = 0;
		txCount = 0;
		expect = -1;
		rxErrors = 0;
		c = 0;
	}
}
