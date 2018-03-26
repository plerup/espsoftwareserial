/*

SoftwareSerial.cpp - Implementation of the Arduino software serial for ESP8266/ESP32.
Copyright (c) 2015-2016 Peter Lerup. All rights reserved.

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

#define ALT_DIGITAL_WRITE 1

#define MAX_PIN 15

// As the Arduino attachInterrupt has no parameter, lists of objects
// and callbacks corresponding to each possible GPIO pins have to be defined
SoftwareSerial *ObjList[MAX_PIN + 1];

void ICACHE_RAM_ATTR sws_isr_0() { ObjList[0]->rxRead(); };
void ICACHE_RAM_ATTR sws_isr_1() { ObjList[1]->rxRead(); };
void ICACHE_RAM_ATTR sws_isr_2() { ObjList[2]->rxRead(); };
void ICACHE_RAM_ATTR sws_isr_3() { ObjList[3]->rxRead(); };
void ICACHE_RAM_ATTR sws_isr_4() { ObjList[4]->rxRead(); };
void ICACHE_RAM_ATTR sws_isr_5() { ObjList[5]->rxRead(); };
// Pin 6 to 11 can not be used
void ICACHE_RAM_ATTR sws_isr_12() { ObjList[12]->rxRead(); };
void ICACHE_RAM_ATTR sws_isr_13() { ObjList[13]->rxRead(); };
void ICACHE_RAM_ATTR sws_isr_14() { ObjList[14]->rxRead(); };
void ICACHE_RAM_ATTR sws_isr_15() { ObjList[15]->rxRead(); };

static void(*ISRList[MAX_PIN + 1])() = {
      sws_isr_0,
      sws_isr_1,
      sws_isr_2,
      sws_isr_3,
      sws_isr_4,
      sws_isr_5,
      NULL,
      NULL,
      NULL,
      NULL,
      NULL,
      NULL,
      sws_isr_12,
      sws_isr_13,
      sws_isr_14,
      sws_isr_15
};

SoftwareSerial::SoftwareSerial(int receivePin, int transmitPin, bool inverse_logic, unsigned int buffSize) {
    m_oneWire = (receivePin == transmitPin);
    m_invert = inverse_logic;
    if (isValidGPIOpin(receivePin)) {
        m_rxPin = receivePin;
        m_buffSize = buffSize;
        m_buffer = (uint8_t*)malloc(m_buffSize);
    }
    if (isValidGPIOpin(transmitPin) || (!m_oneWire && (transmitPin == 16))) {
        m_txValid = true;
        m_txPin = transmitPin;
    }
}

SoftwareSerial::~SoftwareSerial() {
    enableRx(false);
    if (m_rxValid)
        ObjList[m_rxPin] = NULL;
    if (m_buffer)
        free(m_buffer);
}

bool SoftwareSerial::isValidGPIOpin(int pin) {
    return (pin >= 0 && pin <= 5) || (pin >= 12 && pin <= MAX_PIN);
}

void SoftwareSerial::begin(long unsigned baud) {
    // Use getCycleCount() loop to get as exact timing as possible
    m_bitCycles = ESP.getCpuFreqMHz() * 1000000 / baud;
    // Enable interrupts during tx at any baud to allow full duplex
    m_intTxEnabled = true;
    if (m_buffer != NULL) {
        m_rxValid = true;
        m_inPos = m_outPos = 0;
        pinMode(m_rxPin, INPUT_PULLUP);
        if (this != ObjList[m_rxPin]) delete ObjList[m_rxPin];
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

    if (!m_rxEnabled) enableRx(true);
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
    }
    else {
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
        }
        else {
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
        delay(1); // it's important to have a delay after switching
    }
}

void SoftwareSerial::enableRx(bool on) {
    if (m_rxValid) {
        if (on) {
            m_rxCurBit = 8;
            attachInterrupt(digitalPinToInterrupt(m_rxPin), ISRList[m_rxPin], CHANGE);
        }
        else
            detachInterrupt(digitalPinToInterrupt(m_rxPin));
        m_rxEnabled = on;
    }
}

int SoftwareSerial::read() {
    if (!m_rxValid || (rxPendingByte(), m_inPos == m_outPos)) return -1;
    uint8_t ch = m_buffer[m_outPos];
    m_outPos = (m_outPos + 1) % m_buffSize;
    return ch;
}

#define WAIT { long int c = deadline-ESP.getCycleCount(); \
    while (c > 0) { \
        if (m_intTxEnabled && c > 2 * m_bitCycles / 3) optimistic_yield(2 * m_bitCycles / 3 / ESP.getCpuFreqMHz()); \
        c = deadline-ESP.getCycleCount(); } \
    deadline += m_bitCycles; }



int SoftwareSerial::available() {
    if (!m_rxValid) return 0;
    int avail = m_inPos - m_outPos;
    if (avail < 0) avail += m_buffSize;
    if (!avail) {
        if (!rxPendingByte()) optimistic_yield((20 * m_bitCycles) / ESP.getCpuFreqMHz());
        avail = m_inPos - m_outPos;
        if (avail < 0) avail += m_buffSize;
    }
    return avail;
}

size_t ICACHE_RAM_ATTR SoftwareSerial::write(uint8_t b) {
    if (!m_txValid) return 0;

    if (m_invert) b = ~b;
    if (!m_intTxEnabled)
        // Disable interrupts in order to get a clean transmit
        noInterrupts();
    if (m_txEnableValid) {
#ifdef ALT_DIGITAL_WRITE
        pinMode(m_txEnablePin, INPUT_PULLUP);
#else
        digitalWrite(m_txEnablePin, HIGH);
#endif
    }
    long unsigned deadline = ESP.getCycleCount() + m_bitCycles;
#ifdef ALT_DIGITAL_WRITE
    pinMode(m_txPin, m_invert ? OUTPUT : INPUT_PULLUP);
#else
    digitalWrite(m_txPin, !m_invert);
#endif
    // Start bit;
#ifdef ALT_DIGITAL_WRITE
    pinMode(m_txPin, m_invert ? INPUT_PULLUP : OUTPUT);
#else
    digitalWrite(m_txPin, m_invert);
#endif
    WAIT;
    for (int i = 0; i < 8; i++) {
#ifdef ALT_DIGITAL_WRITE
        pinMode(m_txPin, (b & 1) ? INPUT_PULLUP : OUTPUT);
#else
        digitalWrite(m_txPin, (b & 1));
#endif
        WAIT;
        b >>= 1;
    }
    // Stop bit
#ifdef ALT_DIGITAL_WRITE
    pinMode(m_txPin, m_invert ? OUTPUT : INPUT_PULLUP);
#else
    digitalWrite(m_txPin, !m_invert);
#endif
    WAIT;
    if (m_txEnableValid) {
#ifdef ALT_DIGITAL_WRITE
        pinMode(m_txEnablePin, OUTPUT);
#else
        digitalWrite(m_txEnablePin, LOW);
#endif
    }
    if (!m_intTxEnabled)
        interrupts();
    return 1;
}

void SoftwareSerial::flush() {
    m_inPos = m_outPos = 0;
}

bool SoftwareSerial::overflow() {
    bool res = m_overflow;
    m_overflow = false;
    return res;
}

int SoftwareSerial::peek() {
    if (!m_rxValid || (rxPendingByte(), m_inPos == m_outPos)) return -1;
    return m_buffer[m_outPos];
}

bool SoftwareSerial::rxPendingByte() {
    // stop bit interrupt can be missing if leading data bits are same level
    // also had no stop to start bit edge interrupt yet, so one byte may be pending
    long unsigned funCycle = ESP.getCycleCount();
    noInterrupts();
    if (m_rxCurBit < 0 || m_rxCurBit > 7
        || funCycle <= (m_rxCurBitCycle + (8 - m_rxCurBit) * m_bitCycles)) {
        interrupts();
        return false;
    }

    // data bits
    m_rxCurByte >>= 7 - m_rxCurBit;
    if (!m_invert) m_rxCurByte |= 0xff << (m_rxCurBit + 1);

    // stop bit
    m_rxCurBit = 8;
    // Store the received value in the buffer unless we have an overflow
    int next = (m_inPos + 1) % m_buffSize;
    if (next != m_outPos) {
        m_buffer[m_inPos] = m_rxCurByte;
        m_inPos = next;
    }
    else {
        m_overflow = true;
        interrupts();
        return false;
    }
    interrupts();
    return true;
}

void ICACHE_RAM_ATTR SoftwareSerial::rxRead() {
    bool level = digitalRead(m_rxPin);
    level ^= m_invert;
    long unsigned isrCycle = ESP.getCycleCount();
    do {
        // data bits
        if (m_rxCurBit >= -1 && m_rxCurBit < 7) {
            if (isrCycle > m_rxCurBitCycle + m_bitCycles) {
                // preceding masked bits
                int hiddenBits = (isrCycle - m_rxCurBitCycle) / m_bitCycles;
                if (hiddenBits > 7 - m_rxCurBit) hiddenBits = 7 - m_rxCurBit;
                m_rxCurByte >>= hiddenBits;
                if (!level) m_rxCurByte |= 0xff << (8 - hiddenBits);
                m_rxCurBit += hiddenBits;
                m_rxCurBitCycle += hiddenBits * m_bitCycles;
            }
            if (m_rxCurBit < 7) {
                ++m_rxCurBit;
                m_rxCurBitCycle += m_bitCycles;
                m_rxCurByte >>= 1;
                if (level) m_rxCurByte |= 0x80;
            }
            continue;
        }
        // stop bit
        if (m_rxCurBit == 7) {
            ++m_rxCurBit;
            m_rxCurBitCycle += m_bitCycles;
            // Store the received value in the buffer unless we have an overflow
            int next = (m_inPos + 1) % m_buffSize;
            if (next != m_outPos) {
                m_buffer[m_inPos] = m_rxCurByte;
                m_inPos = next;
            }
            else {
                m_overflow = true;
            }
            continue;
        }
        // start bit
        if (m_rxCurBit == 8) {
            if (!level) {
                m_rxCurBit = -1; // start bit must be falling edge
                m_rxCurBitCycle = isrCycle + m_bitCycles - m_bitCycles / 3;
            }
        }
        break; // break by default
    } while (isrCycle >= m_rxCurBitCycle);
}
