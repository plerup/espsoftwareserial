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

// The Arduino standard GPIO routines are not enough,
// must use some from the Espressif SDK as well
extern "C" {
#include "gpio.h"
}

#include <SoftwareSerial.h>

SoftwareSerial::SoftwareSerial(int receivePin, int transmitPin, unsigned int buffSize) {
   m_rxValid = m_txValid = false;
   m_buffer = NULL;
   if (isValidGPIOpin(receivePin)) {
      m_rxPin = receivePin;
      m_buffSize = buffSize;
      m_buffer = (uint8_t*)malloc(m_buffSize);
      if (m_buffer != NULL) {
         m_rxValid = true;
         m_inPos = m_outPos = 0;
         pinMode(m_rxPin, INPUT);
         // Use SDK interrupt management as Arduino attachInterrupt doesn't take any parameter
         ETS_GPIO_INTR_ATTACH(handle_interrupt, this);
         GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, BIT(m_rxPin));
         gpio_pin_intr_state_set(GPIO_ID_PIN(m_rxPin), GPIO_PIN_INTR_NEGEDGE);
      }
   }
   if (isValidGPIOpin(transmitPin)) {
      m_txValid = true;
      m_txPin = transmitPin;
      pinMode(m_txPin, OUTPUT);
   }
   // Default speed
   begin(9600);
}

SoftwareSerial::~SoftwareSerial() {
   // No available SDK API to detach an interrupt handler,
   // just disable the pin interrupt for now
   gpio_pin_intr_state_set(GPIO_ID_PIN(m_rxPin), GPIO_PIN_INTR_DISABLE);
   if (m_buffer)
      free(m_buffer);
}

bool SoftwareSerial::isValidGPIOpin(int pin) {
   // Some GPIO pins are reserved by the system
   return (pin >= 0 && pin <= 5) || (pin >= 12 && pin <= 15);
}

void SoftwareSerial::begin(long speed) {
   m_bitTime = round(1000000.0/speed);
   if (m_bitTime < 5 || m_bitTime > 500) {
      // Invalid speed
      m_rxValid = m_txValid = false;
   }
}

int SoftwareSerial::read() {
   if (!m_rxValid || (m_inPos == m_outPos)) return -1;
   uint8_t ch = m_buffer[m_outPos];
   m_outPos = (m_outPos+1) % m_buffSize;
   return ch;
}

int SoftwareSerial::available() {
   return m_rxValid && ((m_inPos-m_outPos) > 0);
}

// Use micros loop to get as exect timing as possible
#define WAIT { while (micros()-start < wait); wait += m_bitTime; }

size_t SoftwareSerial::write(uint8_t b) {
   if (!m_txValid) return 0;
   // Disable interrupt in order to get a clean transmit
   cli();
   uint16_t wait = m_bitTime;
   digitalWrite(m_txPin, HIGH);
   unsigned long start = micros();
    // Start bit;
   digitalWrite(m_txPin, LOW);
   WAIT;
   for (int i = 0; i < 8; i++) {
     digitalWrite(m_txPin, (b & 1) ? HIGH : LOW);
     WAIT;
     b >>= 1;
   }
   // Stop bit
   digitalWrite(m_txPin, HIGH);
   WAIT;
   sei();
   return 1;
}

void SoftwareSerial::flush() {
   m_inPos = m_outPos = 0;
}

int SoftwareSerial::peek() {
   if (!m_rxValid || (m_inPos == m_outPos)) return -1;
   return m_buffer[m_outPos];
}

void SoftwareSerial::rxRead() {
   uint16_t wait = m_bitTime;
   unsigned long start = micros();
   // Skip half start bit unless this is less than normal interrupt delay time
   if (m_bitTime > 10) {
     wait = m_bitTime/2;
     WAIT;
   }
   uint8_t rec = 0;
   for (int i = 0; i < 8; i++) {
     WAIT;
     rec >>= 1;
     if (digitalRead(m_rxPin))
       rec |= 0x80;
   }
   // Stop bit
   WAIT;
   // Store the received value in the buffer unless we have an overflow
   int next = (m_inPos+1) % m_buffSize;
   if (next != m_inPos) {
      m_buffer[m_inPos] = rec;
      m_inPos = next;
   }
}

void SoftwareSerial::handle_interrupt(SoftwareSerial *swSerObj) {
   if (!swSerObj) return;

   // Check if this interrupt was was coming from the the rx pin of this object
   int pin = swSerObj->m_rxPin;
   uint32_t gpioStatus = GPIO_REG_READ(GPIO_STATUS_ADDRESS);
   if (!(gpioStatus & BIT(pin))) return;
   // Clear the interrupt
   GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, gpioStatus);
   // Seems like the interrupt is delivered on all flanks in spite
   // of GPIO_PIN_INTR_NEGEDGE. Hence ignore unless we have a start bit
   if (digitalRead(pin)) return;

   // Disable GPIO interrupts when sampling the incoming byte
   ETS_GPIO_INTR_DISABLE();
   swSerObj->rxRead();
   ETS_GPIO_INTR_ENABLE();
}

