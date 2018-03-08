#include <SoftwareSerial.h>

SoftwareSerial loopBack(D5, D6);
unsigned long start;
String bitRateTxt("Effective data rate: ");
int txCount;
int rxCount;
unsigned char expect;
int rxErrors;
constexpr int ReportInterval = 10000;

void setup()
{
	Serial.begin(115200);
	loopBack.begin(57600);
	start = micros();
	txCount = 0;
	rxCount = 0;
	expect = -1;
	rxErrors = 0;
}

unsigned char c = 0;

void loop()
{
	expect = c;
	rxCount = txCount;
	do {
		loopBack.write(c);
		c = ++c % 256;
		++txCount;
	} while (c % 16); // use fractions of 256
	while (loopBack.available())
	{
		unsigned char r = loopBack.read();
		if (r == expect) ++rxCount;
		//else Serial.println(String("tx: ") + expect + " rx: " + r);
		expect = ++expect % 256;
	}
	rxErrors += txCount - rxCount;

	if (txCount >= ReportInterval) {
		delay(1);
		const auto end = micros();
		const unsigned long interval = end - start;
		Serial.println(String("tx: ") + txCount + " rx: " + rxCount + " us: " + interval);
		const long txCps = txCount * (1000000.0 / interval);
		const long cps = rxCount * (1000000.0 / interval);
		const long errorCps = rxErrors * (1000000.0 / interval);
		Serial.println(bitRateTxt + 10 * cps + "bps, "
			+ errorCps + "cps errors (" + 100.0 * errorCps / txCps + "%)");
		start = end;
		txCount = 0;
		rxErrors = 0;
	}
}
