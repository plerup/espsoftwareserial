// In this example, we use two ESP32 as the detector and detectee.
// The detectee keeps sending 0x55 at specified baud rate.
// The detector keeps detecting the baud rate, and when the baud rate is determined, 
// it sends "OK" to the detectee.
// When the detectee received "OK" (detector sent with the correct baud rate), 
// it changes to another baud rate and continue to send 0x55. 
// And wait for detector to detect the new baud rate.

#include <Arduino.h>

// If BAUD_DETECTOR is defined, it means this device is to detect baud rate.
// If BAUD_DETECTEE is defined, it means this is the device with certain
// baud rate, and this device should send 0x55 for the other device to detect.

#define BAUD_DETECTOR
#ifndef BAUD_DETECTOR
    #define BAUD_DETECTEE
#endif

#define RX_PIN (16)
#define TX_PIN (17)
#ifdef BAUD_DETECTOR
    #include "SoftwareSerial.h"
    EspSoftwareSerial::UART detectorSerial(RX_PIN, TX_PIN);
#endif
#ifdef BAUD_DETECTEE
    // #define HW_SERIAL
    #ifdef HW_SERIAL
        #include "HardwareSerial.h"
        #define detecteeSerial Serial2
    #else
        #include "SoftwareSerial.h"
        EspSoftwareSerial::UART detecteeSerial(RX_PIN, TX_PIN);
    #endif
#endif

const uint32_t commonBaud[] = {
    9600, 14400, 19200, 38400, 57600, 115200
};

void setup()
{
    Serial.begin(115200);

#ifdef BAUD_DETECTOR
    detectorSerial.begin(1); // Set baud=1 for detectBaud()
#endif
#ifdef BAUD_DETECTEE
    
#endif
}

void loop()
{
#ifdef BAUD_DETECTOR
    while (1) {
        uint32_t detectedBaud = detectorSerial.detectBaud();
        if (detectedBaud == 0) {
            continue;
        }
        else {
            uint32_t min_diff = 0xFFFFFFFF;
            uint32_t baud = 0;
            for (int i = 0; i < sizeof(commonBaud)/sizeof(commonBaud[0]); ++i) {
                int32_t diff = abs((int32_t)detectedBaud - (int32_t)commonBaud[i]);
                if (diff < min_diff) {
                    baud = commonBaud[i];
                    min_diff = diff;
                }
            }
            Serial.printf("Baud Rate Detected: %lu\n", detectedBaud);        

            {
                detectorSerial.end();
                Serial.printf("Set Baud Rate: %lu\n", baud);
                detectorSerial.begin(baud);
                detectorSerial.write("OK", 2);
                detectorSerial.end();
            }
            detectorSerial.begin(1); // Set baud=1 for detectBaud()
        }
    }
#endif
#ifdef BAUD_DETECTEE
    int commonBaudIdx = 0;
    while (1) {
        Serial.printf("Begin with baud = %lu\n", commonBaud[commonBaudIdx]);
#ifdef HW_SERIAL
        detecteeSerial.begin(commonBaud[commonBaudIdx], SERIAL_8N1, RX_PIN, TX_PIN);
#else
        detecteeSerial.begin(commonBaud[commonBaudIdx]);
#endif
        {
            uint8_t rx_buf[8];
            bool responseDetected = false;
            while (responseDetected == false) {
                detecteeSerial.write(0x55);
                if (detecteeSerial.available() >= 2) {
                    detecteeSerial.readBytes(rx_buf, 2);
                    if (rx_buf[0] == 'O' && rx_buf[1] == 'K') {
                        responseDetected = true;
                        Serial.printf("responseDetected = true\n");
                    }
                }
            }
        }
        detecteeSerial.end();

        commonBaudIdx++;
        if (commonBaudIdx >= sizeof(commonBaud)/sizeof(commonBaud[0])) {
            commonBaudIdx = 0;
        }
    }
#endif
}