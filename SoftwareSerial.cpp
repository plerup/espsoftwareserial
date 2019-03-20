/*

SoftwareSerial.cpp - Implementation of the Arduino software serial for ESP8266/ESP32.
Copyright (c) 2015-2016 Peter Lerup. All rights reserved.
Copyright (c) 2018-2019 Dirk O. Kaar. All rights reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

*/

#include <Arduino.h>

#include <SoftwareSerial.h>

// signal quality in ALT_DIGITAL_WRITE is better or equal in all
// tests so far (ESP8266 HW UART, SDS011 PM sensor, SoftwareSerial back-to-back).
#define ALT_DIGITAL_WRITE 1

#if defined(ESP8266)
constexpr size_t MAX_PIN = 15;
#elif defined(ESP32)
constexpr size_t MAX_PIN = 35;
#endif

// As the Arduino attachInterrupt has no parameter, lists of objects
// and callbacks corresponding to each possible GPIO pins have to be defined
SoftwareSerial *ObjList[MAX_PIN + 1];

void ICACHE_RAM_ATTR sws_isr_0() { ObjList[0]->rxRead(); };
#ifdef ESP32
// Pin 1 can not be used
#else
void ICACHE_RAM_ATTR sws_isr_1() { ObjList[1]->rxRead(); };
#endif
void ICACHE_RAM_ATTR sws_isr_2() { ObjList[2]->rxRead(); };
#ifdef ESP32
// Pin 3 can not be used
#else
void ICACHE_RAM_ATTR sws_isr_3() { ObjList[3]->rxRead(); };
#endif
void ICACHE_RAM_ATTR sws_isr_4() { ObjList[4]->rxRead(); };
void ICACHE_RAM_ATTR sws_isr_5() { ObjList[5]->rxRead(); };
// Pin 6 to 11 can not be used
void ICACHE_RAM_ATTR sws_isr_12() { ObjList[12]->rxRead(); };
void ICACHE_RAM_ATTR sws_isr_13() { ObjList[13]->rxRead(); };
void ICACHE_RAM_ATTR sws_isr_14() { ObjList[14]->rxRead(); };
void ICACHE_RAM_ATTR sws_isr_15() { ObjList[15]->rxRead(); };
#ifdef ESP32
void ICACHE_RAM_ATTR sws_isr_16() { ObjList[16]->rxRead(); };
void ICACHE_RAM_ATTR sws_isr_17() { ObjList[17]->rxRead(); };
void ICACHE_RAM_ATTR sws_isr_18() { ObjList[18]->rxRead(); };
void ICACHE_RAM_ATTR sws_isr_19() { ObjList[19]->rxRead(); };
// Pin 20 can not be used
void ICACHE_RAM_ATTR sws_isr_21() { ObjList[21]->rxRead(); };
void ICACHE_RAM_ATTR sws_isr_22() { ObjList[22]->rxRead(); };
void ICACHE_RAM_ATTR sws_isr_23() { ObjList[23]->rxRead(); };
// Pin 24 can not be used
void ICACHE_RAM_ATTR sws_isr_25() { ObjList[25]->rxRead(); };
void ICACHE_RAM_ATTR sws_isr_26() { ObjList[26]->rxRead(); };
void ICACHE_RAM_ATTR sws_isr_27() { ObjList[27]->rxRead(); };
// Pin 28 to 31 can not be used
void ICACHE_RAM_ATTR sws_isr_32() { ObjList[32]->rxRead(); };
void ICACHE_RAM_ATTR sws_isr_33() { ObjList[33]->rxRead(); };
void ICACHE_RAM_ATTR sws_isr_34() { ObjList[34]->rxRead(); };
void ICACHE_RAM_ATTR sws_isr_35() { ObjList[35]->rxRead(); };
#endif

static void(*ISRList[MAX_PIN + 1])() = {
	sws_isr_0,
#ifdef ESP32
	0,
#else
	sws_isr_1,
#endif
	sws_isr_2,
#ifdef ESP32
	0,
#else
	sws_isr_3,
#endif
	sws_isr_4,
	sws_isr_5,
	0,
	0,
	0,
	0,
	0,
	0,
	sws_isr_12,
	sws_isr_13,
	sws_isr_14,
	sws_isr_15,
#ifdef ESP32
	sws_isr_16,
	sws_isr_17,
	sws_isr_18,
	sws_isr_19,
	0,
	sws_isr_21,
	sws_isr_22,
	sws_isr_23,
	0,
	sws_isr_25,
	sws_isr_26,
	sws_isr_27,
	0,
	0,
	0,
	0,
	sws_isr_32,
	sws_isr_33,
	sws_isr_34,
	sws_isr_35,
#endif
};

SoftwareSerial::SoftwareSerial(int receivePin, int transmitPin, bool inverse_logic, int bufSize, int isrBufSize) {
	m_oneWire = (receivePin == transmitPin);
	m_invert = inverse_logic;
	if (isValidGPIOpin(receivePin)) {
		m_rxPin = receivePin;
		m_bufSize = bufSize;
		m_buffer = (uint8_t*)malloc(m_bufSize);
		m_isrBufSize = isrBufSize ? isrBufSize : 10 * bufSize;
		m_isrBuffer = (long unsigned*)malloc(m_isrBufSize * sizeof(long));
	}
	if (isValidGPIOpin(transmitPin) || (!m_oneWire && (transmitPin == 16))) {
		m_txValid = true;
		m_txPin = transmitPin;
	}
}

SoftwareSerial::~SoftwareSerial() {
	enableRx(false);
	if (m_rxValid) {
		ObjList[m_rxPin] = 0;
	}
	if (m_buffer) {
		free(m_buffer);
	}
}

bool SoftwareSerial::isValidGPIOpin(int pin) {
	return (pin >= 0 && pin <= 5) || (pin >= 12 && pin <= MAX_PIN);
}

void SoftwareSerial::begin(long baud) {
	m_bitCycles = ESP.getCpuFreqMHz() * 1000000 / baud;
	m_intTxEnabled = true;
	if (m_buffer != 0 && m_isrBuffer != 0) {
		m_rxValid = true;
		m_inPos = m_outPos = 0;
		m_isrInPos = m_isrOutPos = 0;
		pinMode(m_rxPin, INPUT_PULLUP);
		if (this != ObjList[m_rxPin]) { delete ObjList[m_rxPin]; }
		ObjList[m_rxPin] = this;
	}
	if (m_txValid && !m_oneWire) {
#ifdef ALT_DIGITAL_WRITE
		digitalWrite(m_txPin, LOW);
		pinMode(m_txPin, m_invert ? OUTPUT : INPUT_PULLUP);
#else
		pinMode(m_txPin, OUTPUT);
		digitalWrite(m_txPin, !m_invert);
#endif
	}

	if (!m_rxEnabled) { enableRx(true); }
}

long SoftwareSerial::baudRate() {
	return ESP.getCpuFreqMHz() * 1000000 / m_bitCycles;
}

void SoftwareSerial::setTransmitEnablePin(int transmitEnablePin) {
	if (isValidGPIOpin(transmitEnablePin)) {
		m_txEnableValid = true;
		m_txEnablePin = transmitEnablePin;
#ifdef ALT_DIGITAL_WRITE
		digitalWrite(m_txEnablePin, LOW);
		pinMode(m_txEnablePin, OUTPUT);
#else
		pinMode(m_txEnablePin, OUTPUT);
		digitalWrite(m_txEnablePin, LOW);
#endif
	} else {
		m_txEnableValid = false;
	}
}

void SoftwareSerial::enableIntTx(bool on) {
	m_intTxEnabled = on;
}

void SoftwareSerial::enableTx(bool on) {
	if (m_oneWire && m_txValid) {
		if (on) {
			enableRx(false);
#ifdef ALT_DIGITAL_WRITE
			digitalWrite(m_txPin, LOW);
			pinMode(m_txPin, m_invert ? OUTPUT : INPUT_PULLUP);
			digitalWrite(m_rxPin, LOW);
			pinMode(m_rxPin, m_invert ? OUTPUT : INPUT_PULLUP);
#else
			pinMode(m_txPin, OUTPUT);
			digitalWrite(m_txPin, !m_invert);
			pinMode(m_rxPin, OUTPUT);
			digitalWrite(m_rxPin, !m_invert);
#endif
		} else {
#ifdef ALT_DIGITAL_WRITE
			digitalWrite(m_txPin, LOW);
			pinMode(m_txPin, m_invert ? OUTPUT : INPUT_PULLUP);
#else
			pinMode(m_txPin, OUTPUT);
			digitalWrite(m_txPin, !m_invert);
#endif
			pinMode(m_rxPin, INPUT_PULLUP);
			enableRx(true);
		}
	}
}

void SoftwareSerial::enableRx(bool on) {
	if (m_rxValid) {
		if (on) {
			m_rxCurBit = 8;
			attachInterrupt(digitalPinToInterrupt(m_rxPin), ISRList[m_rxPin], CHANGE);
		} else {
			detachInterrupt(digitalPinToInterrupt(m_rxPin));
		}
		m_rxEnabled = on;
	}
}

int SoftwareSerial::read() {
	if (!m_rxValid) { return -1; }
	if (m_inPos == m_outPos) {
		rxBits();
		if (m_inPos == m_outPos) { return -1; }
	}
	uint8_t ch = m_buffer[m_outPos];
	m_outPos = (m_outPos + 1) % m_bufSize;
	return ch;
}

int SoftwareSerial::available() {
	if (!m_rxValid) { return 0; }
	rxBits();
	int avail = m_inPos - m_outPos;
	if (!avail) {
		optimistic_yield(m_bitCycles / ESP.getCpuFreqMHz() * 20);
		rxBits();
		avail = m_inPos - m_outPos;
	}
	if (avail < 0) { avail += m_bufSize; }
	return avail;
}

void ICACHE_RAM_ATTR SoftwareSerial::preciseDelay(long unsigned deadline) {
	long micro_s = static_cast<long>(deadline - ESP.getCycleCount()) / ESP.getCpuFreqMHz();
	if (micro_s > 8)
	{
		optimistic_yield(micro_s - 8);
		micro_s = static_cast<long>(deadline - ESP.getCycleCount()) / ESP.getCpuFreqMHz();
	}
	if (micro_s > 1) {
		delayMicroseconds(micro_s - 1);
	}
	while (static_cast<long>(deadline - ESP.getCycleCount()) > 1) {}
}

void ICACHE_RAM_ATTR SoftwareSerial::writePeriod(long unsigned dutyCycle, long unsigned offCycle) {
	if (dutyCycle)
	{
		m_periodDeadline += dutyCycle;
#ifdef ALT_DIGITAL_WRITE
		pinMode(m_txPin, INPUT_PULLUP);
#else
		digitalWrite(m_txPin, HIGH);
#endif
		preciseDelay(m_periodDeadline);
	}
	if (offCycle)
	{
		m_periodDeadline += offCycle;
#ifdef ALT_DIGITAL_WRITE
		pinMode(m_txPin, OUTPUT);
#else
		digitalWrite(m_txPin, LOW);
#endif
		preciseDelay(m_periodDeadline);
	}
}

size_t ICACHE_RAM_ATTR SoftwareSerial::write(uint8_t b) {
	return write(&b, 1);
}

size_t ICACHE_RAM_ATTR SoftwareSerial::write(const uint8_t *buffer, size_t size) {
	if (m_rxValid) { rxBits(); }
	if (!m_txValid) { return 0; }

	if (m_txEnableValid) {
#ifdef ALT_DIGITAL_WRITE
		pinMode(m_txEnablePin, INPUT_PULLUP);
#else
		digitalWrite(m_txEnablePin, HIGH);
#endif
	}
	// Stop bit level : LOW if inverted logic, otherwise HIGH
#ifdef ALT_DIGITAL_WRITE
	pinMode(m_txPin, m_invert ? OUTPUT : INPUT_PULLUP);
#else
	digitalWrite(m_txPin, !m_invert);
#endif
	uint32_t dutyCycle = 0;
	uint32_t offCycle = 0;
	bool pb;
	// Disable interrupts in order to get a clean transmit timing
	if (!m_intTxEnabled) noInterrupts();
	m_periodDeadline = ESP.getCycleCount();
	for (int cnt = 0; cnt < size; ++cnt, ++buffer) {
		// Start bit : HIGH if inverted logic, otherwise LOW
		if (m_invert) dutyCycle += m_bitCycles; else offCycle += m_bitCycles;
		pb = m_invert;
		uint8_t o = m_invert ? ~*buffer : *buffer;
		bool b;
		for (int i = 0; i < 9; ++i) {
			// data bit
			// or stop bit : LOW if inverted logic, otherwise HIGH
			b = (i < 8) ? (o & 1) : !m_invert;
			o >>= 1;
			if (!pb && b)
			{
				writePeriod(dutyCycle, offCycle);
				dutyCycle = offCycle = 0;
			}
			if (b) dutyCycle += m_bitCycles; else offCycle += m_bitCycles;
			pb = b;
		}
		if (cnt == size - 1) {
			writePeriod(dutyCycle, offCycle);
			break;
		}
	}
	if (!m_intTxEnabled) interrupts();
	if (m_txEnableValid) {
#ifdef ALT_DIGITAL_WRITE
		pinMode(m_txEnablePin, OUTPUT);
#else
		digitalWrite(m_txEnablePin, LOW);
#endif
	}
	return size;
}

void SoftwareSerial::flush() {
	m_inPos = m_outPos = 0;
	m_isrInPos = m_isrOutPos = 0;
}

bool SoftwareSerial::overflow() {
	bool res = m_overflow;
	m_overflow = false;
	return res;
}

int SoftwareSerial::peek() {
	if (!m_rxValid || (rxBits(), m_inPos == m_outPos)) { return -1; }
	return m_buffer[m_outPos];
}

void ICACHE_RAM_ATTR SoftwareSerial::rxBits() {
	int avail = m_isrInPos - m_isrOutPos;
	if (avail < 0) { avail += m_isrBufSize; }
	if (m_isrOverflow) {
		m_overflow = true;
		m_isrOverflow = false;
	}

	// stop bit can go undetected if leading data bits are at same level
	// and there was also no next start bit yet, so one byte may be pending.
	// low-cost check first
	if (avail == 0 && m_rxCurBit < 8 && m_isrInPos == m_isrOutPos && m_rxCurBit >= 0) {
		long unsigned curCycle = ESP.getCycleCount();
		long unsigned delta = curCycle - m_lastCycle;
		long unsigned expectedDelta = (11 - m_rxCurBit) * m_bitCycles;
		if (delta > expectedDelta) {
			// Store inverted stop bit edge and cycle in the buffer unless we have an overflow
			// cycle's LSB is repurposed for the level bit
			int next = (m_isrInPos + 1) % m_isrBufSize;
			if (next != m_isrOutPos) {
				long unsigned expectedCycle = m_lastCycle + expectedDelta;
				m_isrBuffer[m_isrInPos] = (expectedCycle | 1) ^ !m_invert;
				m_isrInPos = next;
				++avail;
			} else {
				m_isrOverflow = true;
			}
		}
	}

	while (avail--) {
		// error introduced by edge value in LSB is neglegible
		long unsigned isrCycle = m_isrBuffer[m_isrOutPos];
		// extract inverted edge value
		bool level = (isrCycle & 1) == m_invert;
		m_isrOutPos = (m_isrOutPos + 1) % m_isrBufSize;
		long cycles = (isrCycle - m_lastCycle) - m_bitCycles / 2;
		m_lastCycle = isrCycle;
		do {
			// data bits
			if (m_rxCurBit >= -1 && m_rxCurBit < 7) {
				if (cycles >= m_bitCycles) {
					// preceding masked bits
					int hiddenBits = cycles / m_bitCycles;
					if (hiddenBits > 7 - m_rxCurBit) { hiddenBits = 7 - m_rxCurBit; }
					bool lastBit = m_rxCurByte & 0x80;
					m_rxCurByte >>= hiddenBits;
					// masked bits have same level as last unmasked bit
					if (lastBit) { m_rxCurByte |= 0xff << (8 - hiddenBits); }
					m_rxCurBit += hiddenBits;
					cycles -= hiddenBits * m_bitCycles;
				}
				if (m_rxCurBit < 7) {
					++m_rxCurBit;
					cycles -= m_bitCycles;
					m_rxCurByte >>= 1;
					if (level) { m_rxCurByte |= 0x80; }
				}
				continue;
			}
			if (m_rxCurBit == 7) {
				m_rxCurBit = 8;
				cycles -= m_bitCycles;
				// Store the received value in the buffer unless we have an overflow
				int next = (m_inPos + 1) % m_bufSize;
				if (next != m_outPos) {
					m_buffer[m_inPos] = m_rxCurByte;
					// reset to 0 is important for masked bit logic
					m_rxCurByte = 0;
					m_inPos = next;
				} else {
					m_overflow = true;
				}
				continue;
			}
			if (m_rxCurBit == 8) {
				// start bit level is low
				if (!level) {
					m_rxCurBit = -1;
				}
			}
			break;
		} while (cycles >= 0);
	}
}

void ICACHE_RAM_ATTR SoftwareSerial::rxRead() {
	long unsigned curCycle = ESP.getCycleCount();
	bool level = digitalRead(m_rxPin);

	// Store inverted edge value & cycle in the buffer unless we have an overflow
	// cycle's LSB is repurposed for the level bit
	int next = (m_isrInPos + 1) % m_isrBufSize;
	if (next != m_isrOutPos) {
		m_isrBuffer[m_isrInPos] = (curCycle | 1) ^ level;
		m_isrInPos = next;
	} else {
		m_isrOverflow = true;
	}
}

void SoftwareSerial::onReceive(std::function<void(int available)> handler) {
	receiveHandler = handler;
}

void SoftwareSerial::perform_work() {
	if (receiveHandler) {
		if (!m_rxValid) { return; }
		rxBits();
		int avail = m_inPos - m_outPos;
		if (avail < 0) { avail += m_bufSize; }
		if (avail) { receiveHandler(avail); }
	}
}
