Implementation of the Arduino software serial library for the ESP8266

Same functionality as the corresponding AVR libarary but several instances can be active at the same time.
Speed up to 115200 baud is supported. The constructor also has an optional input buffer size.

Please note that during read operations interrupts are disabled for a period of time corresponding to reading
one byte. For low baud rates with heavy load this can be a problem as the ESP WiFi may not get called often
enough and this can cause a crash or watchdog reset. Try to include calls to yield() when possible.

It is not possible to use the standard Arduino attachInterrupt when using this library.

