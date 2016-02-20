/*
/*

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

#include <Arduino.h>

#include <SoftwareSerial.h>

SoftwareSerial * SoftwareSerial::M_instances[EXTERNAL_NUM_INTERRUPTS] = { NULL };

SoftwareSerial::SoftwareSerial(int receivePin, int transmitPin, bool inverse_logic, unsigned int buffSize):
  // the rxPin has to be on an interrupt, and isr != NULL for rx to work.
  m_rxPin(receivePin),
  m_rxInterrupt(digitalPinToInterrupt(receivePin)),
  m_rxValid(false),
  // txPin can be any GPIO pin (I guess...)
  m_txValid(transmitPin < NUM_DIGITAL_PINS), m_txPin(transmitPin),
  // other stuff
  m_invert(inverse_logic), m_buffSize(buffSize),
  m_buffer(NULL), m_inPos(0), m_outPos(0),
  m_bitTime(0)
{
}

SoftwareSerial::~SoftwareSerial() {
  end();
}

void SoftwareSerial::begin(long speed) {
  // Use getCycleCount() loop to get as exact timing as possible
  m_bitTime = ESP.getCpuFreqMHz() * 1000000 / speed;
  m_frameCommitted = true;

  m_rxValid = digitalPinToInterrupt(m_rxPin) != NOT_AN_INTERRUPT;
  if (m_buffer == NULL) {
    m_buffer = (uint8_t*)malloc(m_buffSize);
    if (m_buffer == NULL)
      m_rxValid = false;
  }

  if (m_rxValid)
    pinMode(m_rxPin, INPUT);

  if (m_txValid) {
    pinMode(m_txPin, OUTPUT);
    digitalWrite(m_txPin, !m_invert);
  }
  
  attachRxInterrupt();
}

void SoftwareSerial::end() {
  m_bitTime = 0;
  detachRxInterrupt();
  free(m_buffer);
}

void SoftwareSerial::attachRxInterrupt() {
  if (m_rxValid) {
    // do not attempt to attach the interrupt if
    // the interrupt is currently used bo another instance
    if (M_instances[m_rxInterrupt] != NULL)
      return;

    M_instances[m_rxInterrupt] = this;

    // sorry for the switch-case, there's no other way of doing this...
    void (*isr)() = NULL;
    switch (m_rxInterrupt) {
      case 0: isr = M_isr<0>; break;
      case 1: isr = M_isr<1>; break;
      case 2: isr = M_isr<2>; break;
      case 3: isr = M_isr<3>; break;
      case 4: isr = M_isr<4>; break;
      case 5: isr = M_isr<5>; break;
      case 6: isr = M_isr<6>; break;
      case 7: isr = M_isr<7>; break;
      case 8: isr = M_isr<8>; break;
      case 9: isr = M_isr<9>; break;
      case 10: isr = M_isr<10>; break;
      case 11: isr = M_isr<11>; break;
      case 12: isr = M_isr<12>; break;
      case 13: isr = M_isr<13>; break;
      case 14: isr = M_isr<14>; break;
      case 15: isr = M_isr<15>; break;
      case 16: isr = M_isr<16>; break;
    }

    // using CHANGE for rx without blocking the CPU
    attachInterrupt(m_rxInterrupt, isr, CHANGE);
  }
}

void SoftwareSerial::detachRxInterrupt() {
  // do not attempt to detach interrupt not handled by this
  if (M_instances[m_rxInterrupt] == this)
  {
    M_instances[m_rxInterrupt] = NULL;
    detachInterrupt(m_rxInterrupt);
  }
}

int SoftwareSerial::read() {
  if (!m_rxValid || (m_inPos == m_outPos))
    return -1;

  uint8_t ch = m_buffer[m_outPos];
  ++m_outPos;
  m_outPos %= m_buffSize;
  return ch;
}

int SoftwareSerial::available() {
  if (!m_rxValid)
    return 0;

  // check if we have to commit the frame or not
  unsigned long now = ESP.getCycleCount();
  if (now - m_frameStart > m_bitTime * (1 + 8 + 1))
    commitFrame(m_frameStart + m_bitTime * (1 + 8 + 1));

  int avail = m_inPos - m_outPos;
  if (avail < 0)
    avail += m_buffSize;
  return avail;
}

#define WAIT { while (ESP.getCycleCount() - start < wait); wait += m_bitTime; }

size_t ICACHE_RAM_ATTR SoftwareSerial::write(uint8_t b) {
  if (!m_txValid || m_bitTime == 0)
    return 0;

  if (m_invert)
    b = ~b;

  uint32_t wait = m_bitTime;
  
  // Disable interrupts in order to get a clean transmit
  //uint32_t savedPS = xt_rsil(0);

  digitalWrite(m_txPin, HIGH);
  unsigned long start = ESP.getCycleCount();
  // Start bit;
  digitalWrite(m_txPin, LOW);
  WAIT;
  // data bits
  for (int i = 1;i < 9;++i) {
    digitalWrite(m_txPin, b & 1);
	WAIT;
	b >>= 1;
  }
  // Stop bit
  digitalWrite(m_txPin, HIGH);
  WAIT;

  // restore interrupt
  //xt_wsr_ps(savedPS);

  return 1;
}

void ICACHE_RAM_ATTR SoftwareSerial::updateFrame(uint8_t bitState, uint32_t endTime)
{
  uint32_t now = ESP.getCycleCount();
  for (uint32_t i = m_frameStart + m_lastBit * m_bitTime; i < endTime - m_bitTime / 2; i += m_bitTime)
  {
    m_currentByte >>= 1;
    if (m_bitState == HIGH)
      m_currentByte |= 0x8000;
	++m_lastBit;
  }

  m_bitState = bitState;
}

void ICACHE_RAM_ATTR SoftwareSerial::commitFrame(uint32_t now) {
  if (m_frameCommitted)
    return;

  // set the frame as committed as soon as possible
  m_frameCommitted = true;

  updateFrame(m_bitState, now);

  // Store the received value in the buffer unless we have an overflow
  int next = m_inPos + 1;
  if (next >= m_buffSize)
	next = 0;
  if (next != m_inPos) {
    if (m_invert)
      m_currentByte = ~m_currentByte;
	m_currentByte >>= 7;
    m_buffer[m_inPos] = m_currentByte & 0xff;
    m_inPos = next;
  }
}

void ICACHE_RAM_ATTR SoftwareSerial::rxRead() {
  // record this as soon as we enter the interrupt.
  uint32_t now = ESP.getCycleCount();
  uint8_t state = digitalRead(m_rxPin);

  if (now - m_frameStart > m_bitTime * (1 + 8 + 1)) {
    commitFrame(m_frameStart + m_bitTime * (1 + 8 + 1));
    m_frameStart = now;
    m_lastBit = 0;
	
	// filter out bogus inputs
	if (state == (m_invert ? HIGH : LOW))
		m_frameCommitted = false;
  }

  updateFrame(state, now);
}

template <uint8_t interrupt>
void ICACHE_RAM_ATTR SoftwareSerial::M_isr()
{
  register SoftwareSerial *thiz = M_instances[interrupt];
  if (thiz != NULL)
    thiz->rxRead();
}