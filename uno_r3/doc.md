# doc
Arduino Uno R3 documentation

### Common commands
- pinMode(pin, mode)          INPUT, OUTPUT or INPUT_PULLUP
- digitalWrite(pin, value)    HIGH or LOW
- digitalRead(pin)            returns HIGH or LOW
- analogRead(pin)             A0-A5, returns 0-1023 for 0-5 V
- analogWrite(pin, value)     PWM 0-255, only pins 3, 5, 6, 9, 10, 11
- delay(ms)                   blocking wait
- millis()                    milliseconds since boot, for non-blocking timing
- Serial.begin(baud)          start serial, uses pins 0 and 1
- Serial.println(x)           print a line
- Serial.available()          number of bytes waiting
- Serial.read()               read one byte, -1 if none
- map(x, a, b, c, d)          rescale x from range a-b to range c-d
- constrain(x, lo, hi)        clamp x to a range
