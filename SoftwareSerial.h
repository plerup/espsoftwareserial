/*
  SoftwareSerial.h

  SoftwareSerial.cpp - Implementation of the Arduino software serial for ESP8266.
  Copyright (c) 2015 Peter Lerup. All rights reserved.

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

#ifndef EspSoftwareSerial_h
#define EspSoftwareSerial_h

#include <inttypes.h>
#include <Stream.h>


// This class is compatible with the corresponding AVR one,
// the constructor however has an optional rx buffer size.
// Speed up to 115200 can be used.


class SoftwareSerial : public Stream
{
  public:
    SoftwareSerial(int receivePin, int transmitPin, bool inverse_logic = false, unsigned int buffSize = 64);
    ~SoftwareSerial();

    void begin(long speed);

    int peek() {
      if (!m_rxValid || (m_inPos == m_outPos)) return -1;
      return m_buffer[m_outPos];
    }

    virtual size_t write(uint8_t byte);
    virtual int read();
    virtual int available();
    virtual void flush() {
      m_inPos = m_outPos = 0;
    }
    operator bool() {
      return m_rxValid || m_txValid;
    }

    using Print::write;

  private:
    void attachRxInterrupt();
    void detachRxInterrupt();
    void rxRead();
    void commitFrame();
    void updateFrame(uint8_t bitState, unsigned long const endTime);

    // Member variables
    size_t m_buffSize;
    uint8_t *m_buffer;
    int m_rxPin, m_rxInterrupt, m_txPin;
    bool m_rxValid, m_txValid;
    bool m_invert;
    uint32_t m_bitTime;
    unsigned int m_inPos, m_outPos;

    uint32_t m_frameStart;
    uint8_t m_bitState;
    uint16_t m_currentByte;
    uint32_t m_lastBitTime;
    bool m_frameCommitted;

    static SoftwareSerial * M_instances[EXTERNAL_NUM_INTERRUPTS];

    template<uint8_t interrupt>
    static void M_isr();
};

// If only one tx or rx wanted then use this as parameter for the unused pin
#define SW_SERIAL_UNUSED_PIN -1


#endif
