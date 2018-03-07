#include <SoftwareSerial.h>

SoftwareSerial loopBack(D5, D6);
unsigned long start;
String bitRateTxt("\tBitrate: ");
int txCount;
int rxErrors;
constexpr int ReportInterval = 100000;

void setup()
{
	Serial.begin(115200);
	loopBack.begin(115200);
	loopBack.enableIntTx(true);
	start = micros();
	txCount = 0;
	rxErrors = 0;
}

char c = 0;

void loop()
{
	char expect = c;
	auto rxCount = txCount;
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
	rxErrors += txCount - rxCount;

	if (txCount >= ReportInterval) {
		auto end = micros();
		auto cps = 1000000 / ((end - start) / txCount);
		auto errorCps = (1000000 * rxErrors / (end - start));
		Serial.println(bitRateTxt + 10 * cps + "bps, "
			+ errorCps + "cps errors (" + 100.0 * errorCps / cps + "%)");
		start = end;
		txCount = 0;
		rxErrors = 0;
		c = 0;
	}
}
