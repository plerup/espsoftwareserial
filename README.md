# EspSoftwareSerial9

Implementation of the the software serial library for the ESP8266, but working with 9 data bits

Same functionality as the corresponding AVR library but several instances can be active at the same time.
Speed up to 115200 baud is supported. The constructor also has an optional input buffer size.

Please note that due to the fact that the ESP always have other activities ongoing, there will be some inexactness in interrupt
timings. This may lead to bit errors when having heavy data traffic in high baud rates.

This work is based on the original SorfwareSerial project of Peter Lerup.

Work in progress

