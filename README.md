# EspSoftwareSerial

Implementation of the Arduino software serial library for the ESP8266

Same functionality as the corresponding AVR library but several instances can be active at the same time.
Speed up to 115200 baud is supported. The constructor also has an optional input buffer size.

Please note that due to the fact that the ESP always have other activities ongoing, there will be some inexactness in interrupt
timings. This may lead to bit errors when having heavy data traffic in high baud rates.

It is not possible to use the standard Arduino attachInterrupt when using this library.

## Capability / Limitations

1. full duplex (loop-back) up to 57600 (latency of interrupt by attachInterrupt() on ESP8266 is SUPER HIGH, approx 300 cycles @ 80Mhz, i dunno why...)
2. half duplex up to 115200 (thx to the high latency, i can't achieve higher baudrate even with a 80Mhz mcu...)
3. transmit to one endpoint at a time (due to the nature of no tx buffering)
4. receive from all endpoints, always (thx to interrupt), but you may encounter data-drop at high baudrate. use a lower baudrate if more than 1 SoftwareSerial is used.

# Issues

1. the first transmit will always have timing problem, another mystery......