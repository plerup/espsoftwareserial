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

#include <SoftwareSerial.h>
#ifdef ESP8266
#include <FunctionalInterrupt.h>
#endif
#include <Arduino.h>

#ifdef ESP32
#define xt_rsil(a) (a)
#define xt_wsr_ps(a)
#endif

constexpr uint8_t BYTE_ALL_BITS_SET = ~static_cast<uint8_t>(0);

SoftwareSerial::SoftwareSerial() {
	m_isrOverflow = false;
}

SoftwareSerial::~SoftwareSerial() {
	end();
}

bool SoftwareSerial::isValidGPIOpin(int8_t pin) {
#if defined(ESP8266)
	return (pin >= 0 && pin <= 5) || (pin >= 12 && pin <= 15);
#elif defined(ESP32)
	return pin == 0 || pin == 2 || (pin >= 4 && pin <= 5) || (pin >= 12 && pin <= 19) ||
		(pin >= 21 && pin <= 23) || (pin >= 25 && pin <= 27) || (pin >= 32 && pin <= 35);
#else
	return true;
#endif
}

void SoftwareSerial::begin(int32_t baud, int8_t rxPin, int8_t txPin,
	SoftwareSerialConfig config, bool invert, int bufCapacity, int isrBufCapacity) {
	m_oneWire = (rxPin == txPin);
	m_invert = invert;
	if (isValidGPIOpin(rxPin)) {
		m_rxPin = rxPin;
		std::unique_ptr<circular_queue<uint8_t> > buffer(new circular_queue<uint8_t>((bufCapacity > 0) ? bufCapacity : 64));
		m_buffer = move(buffer);
		std::unique_ptr<circular_queue<uint32_t> > isrBuffer(new circular_queue<uint32_t>((isrBufCapacity > 0) ? isrBufCapacity : (sizeof(uint8_t) * 8 + 2) * bufCapacity));
		m_isrBuffer = move(isrBuffer);
		if (m_buffer != 0 && m_isrBuffer != 0) {
			m_rxValid = true;
			pinMode(m_rxPin, INPUT_PULLUP);
		}
	}
	if (isValidGPIOpin(txPin)
#ifdef ESP8266
		|| ((txPin == 16) && !m_oneWire)) {
#else
		) {
#endif
		m_txValid = true;
		m_txPin = txPin;
		if (!m_oneWire) {
			pinMode(m_txPin, OUTPUT);
			digitalWrite(m_txPin, !m_invert);
		}
	}

	m_dataBits = 5 + config;
	m_bit_us = (1000000 + baud / 2) / baud;
	m_bitCycles = (ESP.getCpuFreqMHz() * 1000000 + baud / 2) / baud;
	m_intTxEnabled = true;
	if (!m_rxEnabled) { enableRx(true); }
}

void SoftwareSerial::end()
{
	enableRx(false);
	m_txValid = false;
	if (m_buffer) {
		m_buffer.reset();
	}
	if (m_isrBuffer) {
		m_isrBuffer.reset();
	}
}

int32_t SoftwareSerial::baudRate() {
	return ESP.getCpuFreqMHz() * 1000000 / m_bitCycles;
}

void SoftwareSerial::setTransmitEnablePin(int8_t txEnablePin) {
	if (isValidGPIOpin(txEnablePin)) {
		m_txEnableValid = true;
		m_txEnablePin = txEnablePin;
		pinMode(m_txEnablePin, OUTPUT);
		digitalWrite(m_txEnablePin, LOW);
	}
	else {
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
		}
		else {
			pinMode(m_rxPin, INPUT_PULLUP);
			enableRx(true);
		}
	}
}

void SoftwareSerial::enableRx(bool on) {
	if (m_rxValid) {
		if (on) {
			m_rxCurBit = m_dataBits;
			// Init to stop bit level and current cycle
			m_isrLastCycle = (ESP.getCycleCount() | 1) ^ m_invert;
#ifdef ESP8266
			attachInterrupt(digitalPinToInterrupt(m_rxPin), std::bind(rxRead, this), CHANGE);
#else
			attachInterruptArg(digitalPinToInterrupt(m_rxPin), reinterpret_cast<void (*)(void*)>(rxRead), this, CHANGE);
#endif
		}
		else {
			detachInterrupt(digitalPinToInterrupt(m_rxPin));
		}
		m_rxEnabled = on;
	}
}

int SoftwareSerial::read() {
	if (!m_rxValid) { return -1; }
	if (!m_buffer->available()) {
		rxBits();
		if (!m_buffer->available()) { return -1; }
	}
	return m_buffer->pop();
}

size_t SoftwareSerial::readBytes(uint8_t* buffer, size_t size) {
	if (!m_rxValid) { return -1; }
	if (0 != (size = m_buffer->pop_n(buffer, size))) return size;
	rxBits();
	size = m_buffer->pop_n(buffer, size);
	return (size == 0) ? -1 : size;
}

int SoftwareSerial::available() {
	if (!m_rxValid) { return 0; }
	rxBits();
	int avail = m_buffer->available();
	if (!avail) {
		optimistic_yield(10000);
	}
	return avail;
}

void ICACHE_RAM_ATTR SoftwareSerial::preciseDelay(uint32_t deadline, bool asyn, uint32_t savedPS) {
	if (asyn && !m_intTxEnabled) { xt_wsr_ps(savedPS); }
	int32_t micro_s = static_cast<int32_t>(deadline - ESP.getCycleCount()) / ESP.getCpuFreqMHz();
	if (asyn) {
		if (micro_s > 0) delay(micro_s / 1000);
	} else
	{
		if (micro_s > m_bit_us) delayMicroseconds(micro_s - m_bit_us);
	}
	while (static_cast<int32_t>(deadline - ESP.getCycleCount()) > 0) { if (asyn) optimistic_yield(10000); }
	if (asyn) m_periodDeadline = ESP.getCycleCount();
	if (asyn && !m_intTxEnabled) { savedPS = xt_rsil(15); }
}

void ICACHE_RAM_ATTR SoftwareSerial::writePeriod(
	uint32_t dutyCycle, uint32_t offCycle, bool withStopBit, uint32_t savedPS) {
	if (dutyCycle) {
		digitalWrite(m_txPin, HIGH);
		m_periodDeadline += dutyCycle;
		bool asyn = withStopBit && !m_invert; 
		// Reenable interrupts while delaying to avoid other tasks piling up
		preciseDelay(m_periodDeadline, asyn, savedPS);
		// Disable interrupts again
	}
	if (offCycle) {
		digitalWrite(m_txPin, LOW);
		m_periodDeadline += offCycle;
		bool asyn = withStopBit && m_invert; 
		// Reenable interrupts while delaying to avoid other tasks piling up
		preciseDelay(m_periodDeadline, asyn, savedPS);
		// Disable interrupts again
	}
}

size_t SoftwareSerial::write(uint8_t b) {
	return write(&b, 1);
}

size_t ICACHE_RAM_ATTR SoftwareSerial::write(const uint8_t * buffer, size_t size) {
	if (m_rxValid) { rxBits(); }
	if (!m_txValid) { return -1; }

	if (m_txEnableValid) {
		digitalWrite(m_txEnablePin, HIGH);
	}
	// Stop bit : LOW if inverted logic, otherwise HIGH
	bool b = !m_invert;
	// Force line level on entry
	uint32_t dutyCycle = b;
	uint32_t offCycle = m_invert;
	uint32_t savedPS = 0;
	if (!m_intTxEnabled) {
		// Disable interrupts in order to get a clean transmit timing
		savedPS = xt_rsil(15);
	}
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
		for (int i = 0; i < m_dataBits + 2; ++i) {
			bool pb = b;
			b = (word >> i) & 1;
			if (!pb && b) {
				writePeriod(dutyCycle, offCycle, withStopBit, savedPS);
				withStopBit = false;
				dutyCycle = offCycle = 0;
			}
			if (b) {
				dutyCycle += m_bitCycles;
			}
			else {
				offCycle += m_bitCycles;
			}
		}
	}
	writePeriod(dutyCycle, offCycle, true, savedPS);
	if (!m_intTxEnabled) {
		// restore the interrupt state
		xt_wsr_ps(savedPS);
	}
	if (m_txEnableValid) {
		digitalWrite(m_txEnablePin, LOW);
	}
	return size;
}

void SoftwareSerial::flush() {
	m_buffer->flush();
}

bool SoftwareSerial::overflow() {
	bool res = m_overflow;
	m_overflow = false;
	return res;
}

int SoftwareSerial::peek() {
	if (!m_rxValid) { return -1; }
	if (!m_buffer->available()) {
		rxBits();
		if (!m_buffer->available()) return -1;
	}
	return m_buffer->peek();
}

void SoftwareSerial::rxBits() {
	int isrAvail = m_isrBuffer->available();
#ifdef ESP8266
	if (m_isrOverflow.load()) {
		m_overflow = true;
		m_isrOverflow.store(false);
	}
#else
	if (m_isrOverflow.exchange(false)) {
		m_overflow = true;
	}
#endif

	// stop bit can go undetected if leading data bits are at same level
	// and there was also no next start bit yet, so one byte may be pending.
	// low-cost check first
	if (!isrAvail && m_rxCurBit >= -1 && m_rxCurBit < m_dataBits) {
		uint32_t expectedCycle = m_isrLastCycle + (m_dataBits - m_rxCurBit) * m_bitCycles;
		if (static_cast<int32_t>(ESP.getCycleCount() - expectedCycle) > 0) {
			// Produce faux stop bit level, prevents start bit maldetection
			// cycle's LSB is repurposed for the level bit
			rxBits((expectedCycle | 1) ^ m_invert);
		}
	}

	m_isrBuffer->for_each([this](const uint32_t& isrCycle) { rxBits(isrCycle); });
}

void SoftwareSerial::rxBits(const uint32_t& isrCycle) {
	bool level = (m_isrLastCycle & 1) ^ m_invert;

	// error introduced by edge value in LSB of isrCycle is negligible
	int32_t cycles = isrCycle - m_isrLastCycle;
	m_isrLastCycle = isrCycle;

	uint8_t bits = cycles / m_bitCycles;
	if (cycles % m_bitCycles > (m_bitCycles >> 1)) ++bits;
	while (bits > 0) {
		// start bit detection
		if (m_rxCurBit >= m_dataBits) {
			// leading edge of start bit
			if (level) break;
			m_rxCurBit = -1;
			--bits;
			continue;
		}
		// data bits
		if (m_rxCurBit >= -1 && m_rxCurBit < (m_dataBits - 1)) {
			int8_t dataBits = min(bits, static_cast<uint8_t>(m_dataBits - m_rxCurBit - 1));
			m_rxCurBit += dataBits;
			bits -= dataBits;
			m_rxCurByte >>= dataBits;
			if (level) { m_rxCurByte |= (BYTE_ALL_BITS_SET << (8 - dataBits)); }
			continue;
		}
		// stop bit
		if (m_rxCurBit == (m_dataBits - 1)) {
			// Store the received value in the buffer unless we have an overflow
			// if not high stop bit level, discard word
			if (level)
			{
				m_buffer->push(m_rxCurByte >> (sizeof(uint8_t) * 8 - m_dataBits));
			}
			++m_rxCurBit;
			// reset to 0 is important for masked bit logic
			m_rxCurByte = 0;
			break;
		}
		break;
	}
}

void ICACHE_RAM_ATTR SoftwareSerial::rxRead(SoftwareSerial * self) {
	uint32_t curCycle = ESP.getCycleCount();
	bool level = digitalRead(self->m_rxPin);

	// Store level and cycle in the buffer unless we have an overflow
	// cycle's LSB is repurposed for the level bit
	if (!self->m_isrBuffer->push((curCycle | 1) ^ !level)) self->m_isrOverflow.store(true);
}

void SoftwareSerial::onReceive(std::function<void(int available)> handler) {
	receiveHandler = handler;
}

void SoftwareSerial::perform_work() {
	if (!m_rxValid) { return; }
	rxBits();
	if (receiveHandler) {
		int avail = m_buffer->available();
		if (avail) { receiveHandler(avail); }
	}
}
