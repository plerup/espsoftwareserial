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

#ifndef ESP32
#ifndef SOFTWARESERIAL_MAX_INSTS
#define SOFTWARESERIAL_MAX_INSTS 8
#endif

// As the ESP8266 Arduino attachInterrupt has no parameter, lists of objects
// and callbacks corresponding to each possible list index have to be defined
static SoftwareSerial* ObjList[SOFTWARESERIAL_MAX_INSTS];

template<int I> void ICACHE_RAM_ATTR sws_isr() {
	SoftwareSerial::rxRead(ObjList[I]);
}

template <int N, int I = N - 1> class ISRTable : public ISRTable<N, I - 1> {
public:
	static const int dummy;
};

template <int N> class ISRTable<N, -1> {
public:
	static const int dummy;
	static void (*array[N])();
};

template <int N, int I>	const int ISRTable<N, I>::dummy =
	reinterpret_cast<int>(ISRTable<N, -1>::array[I] = sws_isr<I>) + 0 * ISRTable<N, I - 1>::dummy;

template <int N> void (*ISRTable<N, -1>::array[N])();

template class ISRTable<SOFTWARESERIAL_MAX_INSTS>;

static void (*(*ISRList))() = ISRTable<SOFTWARESERIAL_MAX_INSTS>::array;
#endif

SoftwareSerial::SoftwareSerial(
	int receivePin, int transmitPin, bool inverse_logic, int bufSize, int isrBufSize) {
	m_isrBuffer = 0;
	m_isrOverflow = false;
	m_isrLastCycle = 0;
	m_oneWire = (receivePin == transmitPin);
	m_invert = inverse_logic;
	if (isValidGPIOpin(receivePin)) {
		m_rxPin = receivePin;
		m_bufSize = bufSize;
		m_buffer = (uint8_t*)malloc(m_bufSize);
		m_isrBufSize = isrBufSize ? isrBufSize : 10 * bufSize;
		m_isrBuffer = static_cast<std::atomic<uint32_t>*>(malloc(m_isrBufSize * sizeof(uint32_t)));
	}
	if (isValidGPIOpin(transmitPin)
#ifdef ESP8266
		|| (!m_oneWire && (transmitPin == 16))) {
#else
		) {
#endif
		m_txValid = true;
		m_txPin = transmitPin;
	}
}

SoftwareSerial::~SoftwareSerial() {
	end();
	if (m_buffer) {
		free(m_buffer);
	}
	if (m_isrBuffer) {
		free(m_isrBuffer);
	}
}

bool SoftwareSerial::isValidGPIOpin(int pin) {
#ifdef ESP8266
	return (pin >= 0 && pin <= 5) || (pin >= 12 && pin <= 15);
#endif
#ifdef ESP32
	return pin == 0 || pin == 2 || (pin >= 4 && pin <= 5) || (pin >= 12 && pin <= 19) ||
		(pin >= 21 && pin <= 23) || (pin >= 25 && pin <= 27) || (pin >= 32 && pin <= 35);
#endif
}

#ifndef ESP32
bool SoftwareSerial::begin(int32_t baud, SoftwareSerialConfig config) {
	if (m_swsInstsIdx < 0)
		for (size_t i = 0; i < (sizeof ObjList / sizeof ObjList[0]); ++i)
		{
			if (!ObjList[i]) {
				m_swsInstsIdx = i;
				ObjList[m_swsInstsIdx] = this;
				break;
			}
		}
	if (m_swsInstsIdx < 0) return false;
#else
	void SoftwareSerial::begin(int32_t baud, SoftwareSerialConfig config) {
#endif
	m_dataBits = 5 + (config % 4);
	m_bitCycles = ESP.getCpuFreqMHz() * 1000000 / baud;
	m_intTxEnabled = true;
	if (m_buffer != 0 && m_isrBuffer != 0) {
		m_rxValid = true;
		m_inPos = m_outPos = 0;
		m_isrInPos.store(0);
		m_isrOutPos.store(0);
		pinMode(m_rxPin, INPUT);
	}
	if (m_txValid && !m_oneWire) {
		pinMode(m_txPin, OUTPUT);
		digitalWrite(m_txPin, !m_invert);
	}

	if (!m_rxEnabled) { enableRx(true); }
#ifndef ESP32
	return true;
#endif
}

void SoftwareSerial::end()
{
	enableRx(false);
#ifndef ESP32
	if (m_swsInstsIdx >= 0)	{
		ObjList[m_swsInstsIdx] = 0;
		m_swsInstsIdx = -1;
	}
#endif
}

int32_t SoftwareSerial::baudRate() {
	return ESP.getCpuFreqMHz() * 1000000 / m_bitCycles;
}

void SoftwareSerial::setTransmitEnablePin(int transmitEnablePin) {
	if (isValidGPIOpin(transmitEnablePin)) {
		m_txEnableValid = true;
		m_txEnablePin = transmitEnablePin;
		pinMode(m_txEnablePin, OUTPUT);
		digitalWrite(m_txEnablePin, LOW);
	} else {
		m_txEnableValid = false;
	}
}

void SoftwareSerial::enableIntTx(bool on) {
	m_intTxEnabled = on;
}

void SoftwareSerial::enableTx(bool on) {
	if (m_txValid && m_oneWire) {
		if (on) {
			enableRx(false);
			pinMode(m_txPin, OUTPUT);
			digitalWrite(m_txPin, !m_invert);
		} else {
			pinMode(m_rxPin, INPUT);
			enableRx(true);
		}
	}
}

void SoftwareSerial::enableRx(bool on) {
	if (m_rxValid) {
		if (on) {
			m_rxCurBit = m_dataBits;
#ifndef ESP32
			attachInterrupt(digitalPinToInterrupt(m_rxPin), ISRList[m_swsInstsIdx], CHANGE);
#else
			attachInterruptArg(digitalPinToInterrupt(m_rxPin), reinterpret_cast<void (*)(void*)>(rxRead), this, CHANGE);
#endif
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
	if (avail < 0) { avail += m_bufSize; }
	if (!avail) {
		optimistic_yield(2 * (m_dataBits + 2) * m_bitCycles / ESP.getCpuFreqMHz());
		rxBits();
		avail = m_inPos - m_outPos;
		if (avail < 0) { avail += m_bufSize; }
	}
	return avail;
}

void ICACHE_RAM_ATTR SoftwareSerial::preciseDelay(uint32_t deadline, bool asyn) {
	// Reenable interrupts while delaying to avoid other tasks piling up
	if (asyn && !m_intTxEnabled) { interrupts(); }
	int32_t micro_s = static_cast<int32_t>(deadline - ESP.getCycleCount()) / ESP.getCpuFreqMHz();
	if (micro_s > 0) {
		if (asyn) optimistic_yield(micro_s); else delayMicroseconds(micro_s);
	}
	while (static_cast<int32_t>(deadline - ESP.getCycleCount()) > 0) { if (asyn) optimistic_yield(1); }
	if (asyn) {
		// Disable interrupts again
		if (!m_intTxEnabled) {
			noInterrupts();
		}
		m_periodDeadline = ESP.getCycleCount();
	}
}

void ICACHE_RAM_ATTR SoftwareSerial::writePeriod(uint32_t dutyCycle, uint32_t offCycle, bool withStopBit) {
	if (dutyCycle) {
		digitalWrite(m_txPin, HIGH);
		m_periodDeadline += dutyCycle;
		preciseDelay(m_periodDeadline, withStopBit && !m_invert);
	}
	if (offCycle) {
		digitalWrite(m_txPin, LOW);
		m_periodDeadline += offCycle;
		preciseDelay(m_periodDeadline, withStopBit && m_invert);
	}
}

size_t SoftwareSerial::write(uint8_t b) {
	return write(&b, 1);
}

size_t ICACHE_RAM_ATTR SoftwareSerial::write(const uint8_t *buffer, size_t size) {
	if (m_rxValid) { rxBits(); }
	if (!m_txValid) { return 0; }

	if (m_txEnableValid) {
		digitalWrite(m_txEnablePin, HIGH);
	}
	// Stop bit : LOW if inverted logic, otherwise HIGH
	bool b = !m_invert;
	// Force line level on entry
	uint32_t dutyCycle = b;
	uint32_t offCycle = m_invert;
	// Disable interrupts in order to get a clean transmit timing
	if (!m_intTxEnabled) { noInterrupts(); }
	m_periodDeadline = ESP.getCycleCount();
	const uint32_t dataMask = ((1UL << m_dataBits) - 1);
	for (size_t cnt = 0; cnt < size; ++cnt, ++buffer) {
		bool withStopBit = true;
		// push LSB start-data-stop bit pattern into uint32_t
		// Stop bit : LOW if inverted logic, otherwise HIGH
		uint32_t word = (!m_invert) << m_dataBits;
		word |= (m_invert ? ~*buffer : *buffer) & dataMask;
		// Start bit : HIGH if inverted logic, otherwise LOW
		word <<= 1;
		word |= m_invert;
		for (int i = 0; i <= m_dataBits + 1; ++i) {
			bool pb = b;
			b = (word >> i) & 1;
			if (!pb && b) {
				writePeriod(dutyCycle, offCycle, withStopBit);
				withStopBit = false;
				dutyCycle = offCycle = 0;
			}
			if (b) {
				dutyCycle += m_bitCycles;
			} else {
				offCycle += m_bitCycles;
			}
		}
	}
	writePeriod(dutyCycle, offCycle, true);
	if (!m_intTxEnabled) { interrupts(); }
	if (m_txEnableValid) {
		digitalWrite(m_txEnablePin, LOW);
	}
	return size;
}

void SoftwareSerial::flush() {
	m_inPos = m_outPos = 0;
	m_isrInPos.store(0);
	m_isrOutPos.store(0);
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

void SoftwareSerial::rxBits() {
	int avail = m_isrInPos.load() - m_isrOutPos.load();
	if (avail < 0) { avail += m_isrBufSize; }
	if (m_isrOverflow.load()) {
		m_overflow = true;
		m_isrOverflow.store(false);
	}

	// stop bit can go undetected if leading data bits are at same level
	// and there was also no next start bit yet, so one byte may be pending.
	// low-cost check first
	if (avail == 0 && m_rxCurBit < m_dataBits && m_isrInPos.load() == m_isrOutPos.load() && m_rxCurBit >= 0) {
		uint32_t expectedCycle = m_isrLastCycle.load() + (m_dataBits + 1 - m_rxCurBit) * m_bitCycles;
		if (static_cast<int32_t>(ESP.getCycleCount() - expectedCycle) > m_bitCycles) {
			// Store inverted stop bit edge and cycle in the buffer unless we have an overflow
			// cycle's LSB is repurposed for the level bit
			int next = (m_isrInPos.load() + 1) % m_isrBufSize;
			if (next != m_isrOutPos.load()) {
				m_isrBuffer[m_isrInPos.load()].store((expectedCycle | 1) ^ !m_invert);
				m_isrInPos.store(next);
				++avail;
			} else {
				m_isrOverflow.store(true);
			}
		}
	}

	while (avail--) {
		// error introduced by edge value in LSB is negligible
		uint32_t isrCycle = m_isrBuffer[m_isrOutPos.load()].load();
		// extract inverted edge value
		bool level = (isrCycle & 1) == m_invert;
		m_isrOutPos.store((m_isrOutPos.load() + 1) % m_isrBufSize);
		int32_t cycles = static_cast<int32_t>(isrCycle - m_isrLastCycle.load() - (m_bitCycles / 2));
		if (cycles < 0) { continue; }
		m_isrLastCycle.store(isrCycle);
		do {
			// data bits
			if (m_rxCurBit >= -1 && m_rxCurBit < (m_dataBits - 1)) {
				if (cycles >= m_bitCycles) {
					// preceding masked bits
					int hiddenBits = cycles / m_bitCycles;
					if (hiddenBits >= m_dataBits - m_rxCurBit) { hiddenBits = (m_dataBits - 1) - m_rxCurBit; }
					bool lastBit = m_rxCurByte & 0x80;
					m_rxCurByte >>= hiddenBits;
					// masked bits have same level as last unmasked bit
					if (lastBit) { m_rxCurByte |= 0xff << (8 - hiddenBits); }
					m_rxCurBit += hiddenBits;
					cycles -= hiddenBits * m_bitCycles;
				}
				if (m_rxCurBit < (m_dataBits - 1)) {
					++m_rxCurBit;
					cycles -= m_bitCycles;
					m_rxCurByte >>= 1;
					if (level) { m_rxCurByte |= 0x80; }
				}
				continue;
			}
			if (m_rxCurBit == (m_dataBits - 1)) {
				++m_rxCurBit;
				cycles -= m_bitCycles;
				// Store the received value in the buffer unless we have an overflow
				int next = (m_inPos + 1) % m_bufSize;
				if (next != m_outPos) {
					m_buffer[m_inPos] = m_rxCurByte >> (8 - m_dataBits);
					// reset to 0 is important for masked bit logic
					m_rxCurByte = 0;
					m_inPos = next;
				} else {
					m_overflow = true;
				}
				continue;
			}
			if (m_rxCurBit >= m_dataBits) {
				// start bit level is low
				if (!level) {
					m_rxCurBit = -1;
				}
			}
			break;
		} while (cycles >= 0);
	}
}

void ICACHE_RAM_ATTR SoftwareSerial::rxRead(SoftwareSerial* self) {
	uint32_t curCycle = ESP.getCycleCount();
	bool level = digitalRead(self->m_rxPin);

	// Store inverted edge value & cycle in the buffer unless we have an overflow
	// cycle's LSB is repurposed for the level bit
	int next = (self->m_isrInPos.load() + 1) % self->m_isrBufSize;
	if (next != self->m_isrOutPos.load()) {
		self->m_isrBuffer[self->m_isrInPos.load()].store((curCycle | 1) ^ level);
		self->m_isrInPos.store(next);
	} else {
		self->m_isrOverflow.store(true);
	}
}

void SoftwareSerial::onReceive(std::function<void(int available)> handler) {
	receiveHandler = handler;
}

void SoftwareSerial::perform_work() {
	if (!m_rxValid) { return; }
	rxBits();
	if (receiveHandler) {
		int avail = m_inPos - m_outPos;
		if (avail < 0) { avail += m_bufSize; }
		if (avail) { receiveHandler(avail); }
	}
}
