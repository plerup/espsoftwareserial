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

// Options for `config` argument
#define SWSER_NB_BIT_MASK      0B00011000
#define SWSER_NB_BIT_5         0B00000000
#define SWSER_NB_BIT_6         0B00001000
#define SWSER_NB_BIT_7         0B00010000
#define SWSER_NB_BIT_8         0B00011000

#define SWSER_PARITY_MASK      0B00000111
#define SWSER_PARITY_NONE      0B00000000
#define SWSER_PARITY_EVEN      0B00000010
#define SWSER_PARITY_ODD       0B00000011
#define SWSER_PARITY_MARK      0B00000100
#define SWSER_PARITY_SPACE     0B00000101

#define SWSER_NB_STOP_BIT_MASK 0B01100000
#define SWSER_NB_STOP_BIT_0    0B00000000
#define SWSER_NB_STOP_BIT_1    0B00100000
#define SWSER_NB_STOP_BIT_15   0B01000000
#define SWSER_NB_STOP_BIT_2    0B01100000

enum SoftwareSerialConfig : uint8_t {
	SWSERIAL_5N1 = ( SWSER_NB_BIT_5 | SWSER_PARITY_NONE  | SWSER_NB_STOP_BIT_1 ),
	SWSERIAL_6N1 = ( SWSER_NB_BIT_6 | SWSER_PARITY_NONE  | SWSER_NB_STOP_BIT_1 ),
	SWSERIAL_7N1 = ( SWSER_NB_BIT_7 | SWSER_PARITY_NONE  | SWSER_NB_STOP_BIT_1 ),
	SWSERIAL_8N1 = ( SWSER_NB_BIT_8 | SWSER_PARITY_NONE  | SWSER_NB_STOP_BIT_1 ),
	SWSERIAL_5E1 = ( SWSER_NB_BIT_5 | SWSER_PARITY_EVEN  | SWSER_NB_STOP_BIT_1 ),
	SWSERIAL_6E1 = ( SWSER_NB_BIT_6 | SWSER_PARITY_EVEN  | SWSER_NB_STOP_BIT_1 ),
	SWSERIAL_7E1 = ( SWSER_NB_BIT_7 | SWSER_PARITY_EVEN  | SWSER_NB_STOP_BIT_1 ),
	SWSERIAL_8E1 = ( SWSER_NB_BIT_8 | SWSER_PARITY_EVEN  | SWSER_NB_STOP_BIT_1 ),
	SWSERIAL_5O1 = ( SWSER_NB_BIT_5 | SWSER_PARITY_ODD   | SWSER_NB_STOP_BIT_1 ),
	SWSERIAL_6O1 = ( SWSER_NB_BIT_6 | SWSER_PARITY_ODD   | SWSER_NB_STOP_BIT_1 ),
	SWSERIAL_7O1 = ( SWSER_NB_BIT_7 | SWSER_PARITY_ODD   | SWSER_NB_STOP_BIT_1 ),
	SWSERIAL_8O1 = ( SWSER_NB_BIT_8 | SWSER_PARITY_ODD   | SWSER_NB_STOP_BIT_1 ),
	SWSERIAL_5M1 = ( SWSER_NB_BIT_5 | SWSER_PARITY_MARK  | SWSER_NB_STOP_BIT_1 ),
	SWSERIAL_6M1 = ( SWSER_NB_BIT_6 | SWSER_PARITY_MARK  | SWSER_NB_STOP_BIT_1 ),
	SWSERIAL_7M1 = ( SWSER_NB_BIT_7 | SWSER_PARITY_MARK  | SWSER_NB_STOP_BIT_1 ),
	SWSERIAL_8M1 = ( SWSER_NB_BIT_8 | SWSER_PARITY_MARK  | SWSER_NB_STOP_BIT_1 ),
	SWSERIAL_5S1 = ( SWSER_NB_BIT_5 | SWSER_PARITY_SPACE | SWSER_NB_STOP_BIT_1 ),
	SWSERIAL_6S1 = ( SWSER_NB_BIT_6 | SWSER_PARITY_SPACE | SWSER_NB_STOP_BIT_1 ),
	SWSERIAL_7S1 = ( SWSER_NB_BIT_7 | SWSER_PARITY_SPACE | SWSER_NB_STOP_BIT_1 ),
	SWSERIAL_8S1 = ( SWSER_NB_BIT_8 | SWSER_PARITY_SPACE | SWSER_NB_STOP_BIT_1 ),
	SWSERIAL_5N2 = ( SWSER_NB_BIT_5 | SWSER_PARITY_NONE  | SWSER_NB_STOP_BIT_2 ),
	SWSERIAL_6N2 = ( SWSER_NB_BIT_6 | SWSER_PARITY_NONE  | SWSER_NB_STOP_BIT_2 ),
	SWSERIAL_7N2 = ( SWSER_NB_BIT_7 | SWSER_PARITY_NONE  | SWSER_NB_STOP_BIT_2 ),
	SWSERIAL_8N2 = ( SWSER_NB_BIT_8 | SWSER_PARITY_NONE  | SWSER_NB_STOP_BIT_2 ),
	SWSERIAL_5E2 = ( SWSER_NB_BIT_5 | SWSER_PARITY_EVEN  | SWSER_NB_STOP_BIT_2 ),
	SWSERIAL_6E2 = ( SWSER_NB_BIT_6 | SWSER_PARITY_EVEN  | SWSER_NB_STOP_BIT_2 ),
	SWSERIAL_7E2 = ( SWSER_NB_BIT_7 | SWSER_PARITY_EVEN  | SWSER_NB_STOP_BIT_2 ),
	SWSERIAL_8E2 = ( SWSER_NB_BIT_8 | SWSER_PARITY_EVEN  | SWSER_NB_STOP_BIT_2 ),
	SWSERIAL_5O2 = ( SWSER_NB_BIT_5 | SWSER_PARITY_ODD   | SWSER_NB_STOP_BIT_2 ),
	SWSERIAL_6O2 = ( SWSER_NB_BIT_6 | SWSER_PARITY_ODD   | SWSER_NB_STOP_BIT_2 ),
	SWSERIAL_7O2 = ( SWSER_NB_BIT_7 | SWSER_PARITY_ODD   | SWSER_NB_STOP_BIT_2 ),
	SWSERIAL_8O2 = ( SWSER_NB_BIT_8 | SWSER_PARITY_ODD   | SWSER_NB_STOP_BIT_2 ),
	SWSERIAL_5M2 = ( SWSER_NB_BIT_5 | SWSER_PARITY_MARK  | SWSER_NB_STOP_BIT_2 ),
	SWSERIAL_6M2 = ( SWSER_NB_BIT_6 | SWSER_PARITY_MARK  | SWSER_NB_STOP_BIT_2 ),
	SWSERIAL_7M2 = ( SWSER_NB_BIT_7 | SWSER_PARITY_MARK  | SWSER_NB_STOP_BIT_2 ),
	SWSERIAL_8M2 = ( SWSER_NB_BIT_8 | SWSER_PARITY_MARK  | SWSER_NB_STOP_BIT_2 ),
	SWSERIAL_5S2 = ( SWSER_NB_BIT_5 | SWSER_PARITY_SPACE | SWSER_NB_STOP_BIT_2 ),
	SWSERIAL_6S2 = ( SWSER_NB_BIT_6 | SWSER_PARITY_SPACE | SWSER_NB_STOP_BIT_2 ),
	SWSERIAL_7S2 = ( SWSER_NB_BIT_7 | SWSER_PARITY_SPACE | SWSER_NB_STOP_BIT_2 ),
	SWSERIAL_8S2 = ( SWSER_NB_BIT_8 | SWSER_PARITY_SPACE | SWSER_NB_STOP_BIT_2 )
};

enum ParityMode : uint8_t { NONE = 0, ODD, EVEN, SPACE, MARK };

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
	int peekParityError();
	void flush() override;
	size_t write(uint8_t byte) override;
	size_t write(uint8_t byte, ParityMode parity);
	size_t write(const uint8_t *buffer, size_t size) override;
	size_t write(const uint8_t *buffer, size_t size, ParityMode parity);
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
	bool calcParity(const uint8_t b, ParityMode parity);
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
	int8_t m_parityBits;
    int8_t m_stopBits;
    ParityMode m_parity;
	int32_t m_bitCycles;
	uint32_t m_periodDeadline;
	bool m_intTxEnabled;
	int m_inPos, m_outPos;
	int m_bufSize = 0;
	uint8_t *m_buffer = 0;
	uint8_t *m_pbuffer = 0;		// buffer to store parity for received chars
	// the ISR stores the relative bit times in the buffer. The inversion corrected level is used as sign bit (2's complement):
	// 1 = positive including 0, 0 = negative.
	std::atomic<int> m_isrInPos, m_isrOutPos;
	int m_isrBufSize = 0;
	std::atomic<uint32_t>* m_isrBuffer;
	std::atomic<bool> m_isrOverflow;
	std::atomic<uint32_t> m_isrLastCycle;
	int m_rxCurBit; // -1: start bit. 0 - 7: data bits. 8: parity bit (optional). 8(-9) or 9(-10): stop bit(s).
	uint8_t m_rxCurByte = 0;
	uint8_t m_rxCurParityBit;
	std::function<void(int available)> receiveHandler = 0;
};

#endif
