/*
SoftwareSerial.h

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

#ifndef SoftwareSerial_h
#define SoftwareSerial_h

#include <inttypes.h>
#include <Stream.h>
#include <functional>
#include <atomic>

// If only one tx or rx wanted then use this as parameter for the unused pin
constexpr int SW_SERIAL_UNUSED_PIN = -1;

enum SoftwareSerialConfig {
	SWSERIAL_5N1 = 0,
	SWSERIAL_6N1,
	SWSERIAL_7N1,
	SWSERIAL_8N1,
};

// This class is compatible with the corresponding AVR one,
// the constructor however has an optional rx buffer size.
// Baudrates up to 115200 can be used.

class SoftwareSerial : public Stream {
public:
	SoftwareSerial(int receivePin, int transmitPin, bool inverse_logic = false, int bufSize = 64, int isrBufSize = 0);
	virtual ~SoftwareSerial();
#ifndef ESP32
	// Returns false if more than SOFTWARESERIAL_MAX_INSTS instances are started
	bool begin(int32_t baud) {
		return begin(baud, SWSERIAL_8N1);
	}
	bool begin(int32_t baud, SoftwareSerialConfig config);
#else
	void begin(int32_t baud) {
		begin(baud, SWSERIAL_8N1);
	}
	void begin(int32_t baud, SoftwareSerialConfig config);
#endif
	int32_t baudRate();
	// Transmit control pin
	void setTransmitEnablePin(int transmitEnablePin);
	// Enable or disable interrupts during tx
	void enableIntTx(bool on);

	bool overflow();

	int available() override;
	int peek() override;
	int read() override;
	void flush() override;
	size_t write(uint8_t byte) override;
	size_t write(const uint8_t *buffer, size_t size) override;
	operator bool() const { return m_rxValid || m_txValid; }

	// Disable or enable interrupts on the rx pin
	void enableRx(bool on);
	// One wire control
	void enableTx(bool on);

	static void rxRead(SoftwareSerial* self);

	// AVR compatibility methods
	bool listen() { enableRx(true); return true; }
	void end();
	bool isListening() { return m_rxEnabled; }
	bool stopListening() { enableRx(false); return true; }

	void onReceive(std::function<void(int available)> handler);
	void perform_work();

	using Print::write;

private:
	// If asyn, its legal to exceed the deadline, for instance,
	// by enabling interrupts.
	void preciseDelay(uint32_t deadline, bool asyn);
	// If withStopBit is set, either cycle contains a stop bit.
	// If dutyCycle == 0, the level is not forced to HIGH.
	// If offCycle == 0, the level remains unchanged from dutyCycle.
	void writePeriod(uint32_t dutyCycle, uint32_t offCycle, bool withStopBit);
	bool isValidGPIOpin(int pin);
	/* check m_rxValid that calling is safe */
	void rxBits();

	// Member variables
	bool m_oneWire;
	int m_rxPin = SW_SERIAL_UNUSED_PIN;
	int m_txPin = SW_SERIAL_UNUSED_PIN;
#ifndef ESP32
	ssize_t m_swsInstsIdx = -1;
#endif
	int m_txEnablePin = SW_SERIAL_UNUSED_PIN;
	bool m_rxValid = false;
	bool m_rxEnabled = false;
	bool m_txValid = false;
	bool m_txEnableValid = false;
	bool m_invert;
	bool m_overflow = false;
	int8_t m_dataBits;
	int32_t m_bitCycles;
	uint32_t m_periodDeadline;
	bool m_intTxEnabled;
	int m_inPos, m_outPos;
	int m_bufSize = 0;
	uint8_t *m_buffer = 0;
	// the ISR stores the relative bit times in the buffer. The inversion corrected level is used as sign bit (2's complement):
	// 1 = positive including 0, 0 = negative.
	std::atomic<int> m_isrInPos, m_isrOutPos;
	int m_isrBufSize = 0;
	std::atomic<uint32_t>* m_isrBuffer;
	std::atomic<bool> m_isrOverflow;
	std::atomic<uint32_t> m_isrLastCycle;
	int m_rxCurBit; // 0 - 7: data bits. -1: start bit. 8: stop bit.
	uint8_t m_rxCurByte = 0;

	std::function<void(int available)> receiveHandler = 0;
};

#endif
