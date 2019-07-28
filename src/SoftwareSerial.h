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

#ifndef __SoftwareSerial_h
#define __SoftwareSerial_h

#include "circular_queue/circular_queue.h"
#include <Stream.h>
#include <functional>

/// If only one tx or rx wanted then use this as parameter for the unused pin.
constexpr int SW_SERIAL_UNUSED_PIN = -1;

enum SoftwareSerialConfig {
	SWSERIAL_5N1 = 0,
	SWSERIAL_6N1,
	SWSERIAL_7N1,
	SWSERIAL_8N1,
};

/// This class is compatible with the corresponding AVR one,
/// the constructor however has optional rx buffer capacity arguments
/// for byte buffer and ISR bit buffer.
/// Baudrates up to 115200 can be used.
class SoftwareSerial : public Stream {
public:
	SoftwareSerial();
	SoftwareSerial(const SoftwareSerial&) = delete;
	SoftwareSerial& operator= (const SoftwareSerial&) = delete;
	virtual ~SoftwareSerial();
	void begin(int32_t baud, int8_t rxPin, int8_t txPin = -1,
		SoftwareSerialConfig config = SWSERIAL_8N1,
		bool invert = false, int bufCapacity = 64, int isrBufCapacity = 0);
	void begin(int32_t baud, SoftwareSerialConfig config) = delete;
	int32_t baudRate();
	/// Transmit control pin.
	void setTransmitEnablePin(int8_t txEnablePin);
	/// Enable or disable interrupts during tx.
	void enableIntTx(bool on);

	bool overflow();

	int available() override;
	int availableForWrite() {
		if (!m_txValid) return 0;
		return 1;
	}
	int peek() override;
	int read() override;
	#ifdef ESP32
	/// The readBytes functions are non-waiting, there is no timeout.
	size_t readBytes(uint8_t* buffer, size_t size) ;
	/// The readBytes functions are non-waiting, there is no timeout.
	size_t readBytes(char* buffer, size_t size) {
		return readBytes(reinterpret_cast<uint8_t*>(buffer), size);
	}
	#else
	/// The readBytes functions are non-waiting, there is no timeout.
	size_t readBytes(uint8_t* buffer, size_t size) override;
	/// The readBytes functions are non-waiting, there is no timeout.
	size_t readBytes(char* buffer, size_t size) override {
		return readBytes(reinterpret_cast<uint8_t*>(buffer), size);
	}
	#endif 
	void flush() override;
	size_t write(uint8_t byte) override;
	size_t write(const uint8_t* buffer, size_t size) override;
	size_t write(const char* buffer, size_t size) {
		return write(reinterpret_cast<const uint8_t*>(buffer), size);
	}
	operator bool() const { return m_rxValid || m_txValid; }

	/// Disable or enable interrupts on the rx pin.
	void enableRx(bool on);
	/// One wire control.
	void enableTx(bool on);

	// AVR compatibility methods.
	bool listen() { enableRx(true); return true; }
	void end();
	bool isListening() { return m_rxEnabled; }
	bool stopListening() { enableRx(false); return true; }

	/// Set an event handler for received data.
	void onReceive(std::function<void(int available)> handler);

	/// Run the internal processing and event engine. Can be iteratively called
	/// from loop, or otherwise scheduled.
	void perform_work();

	using Print::write;

private:
	// If asyn, it's legal to exceed the deadline, for instance,
	// by enabling interrupts.
	void preciseDelay(uint32_t deadline, bool asyn, uint32_t savedPS);
	// If withStopBit is set, either cycle contains a stop bit.
	// If dutyCycle == 0, the level is not forced to HIGH.
	// If offCycle == 0, the level remains unchanged from dutyCycle.
	void writePeriod(
		uint32_t dutyCycle, uint32_t offCycle, bool withStopBit, uint32_t savedPS);
	bool isValidGPIOpin(int8_t pin);
	/* check m_rxValid that calling is safe */
	void rxBits();
	void rxBits(const uint32_t& isrCycle);

	static void rxBitISR(SoftwareSerial* self);

	// Member variables
	bool m_oneWire;
	int8_t m_rxPin = SW_SERIAL_UNUSED_PIN;
	int8_t m_txPin = SW_SERIAL_UNUSED_PIN;
	int8_t m_txEnablePin = SW_SERIAL_UNUSED_PIN;
	bool m_rxValid = false;
	bool m_rxEnabled = false;
	bool m_txValid = false;
	bool m_txEnableValid = false;
	bool m_invert;
	bool m_overflow = false;
	int8_t m_dataBits;
	int32_t m_bit_us;
	int32_t m_bitCycles;
	uint32_t m_periodDeadline;
	bool m_intTxEnabled;
	std::unique_ptr<circular_queue<uint8_t> > m_buffer;
	// the ISR stores the relative bit times in the buffer. The inversion corrected level is used as sign bit (2's complement):
	// 1 = positive including 0, 0 = negative.
	std::unique_ptr<circular_queue<uint32_t> > m_isrBuffer;
	std::atomic<bool> m_isrOverflow;
	uint32_t m_isrLastCycle;
	int8_t m_rxCurBit; // 0 - 7: data bits. -1: start bit. 8: stop bit.
	uint8_t m_rxCurByte = 0;

	std::function<void(int available)> receiveHandler;
};

#endif // __SoftwareSerial_h
