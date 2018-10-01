// Modified from [https://playground.arduino.cc/Code/Charlieplex]

#ifndef CHARLIEPLEX_H
#define CHARLIEPLEX_H

#include <Arduino.h>

struct CharliePin {
	byte vcc;
	byte gnd;
};

typedef CharliePin charliePin;

class Charlieplex {
public:
	Charlieplex(byte* userPins, byte numberOfUserPins);
	void write(charliePin pin, bool state);
	void writePwm(charliePin pin, byte dutyCycle);
	void clear();

private:
	void setHigh(byte pin);
	void setPwm(byte pin, byte dutyCycle);
	void setLow(byte pin);
	void setZ(byte pin);
	byte numberOfPins;
	byte* pins;
};

#endif
