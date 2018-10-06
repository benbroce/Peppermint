// Modified from [https://playground.arduino.cc/Code/Charlieplex]

#ifndef CHARLIEPLEX_H
#define CHARLIEPLEX_H

#include <Arduino.h>

// Stores the matrix pair used to control a single LED
struct CharlieLED {
	byte vcc;
	byte gnd;
};
typedef CharlieLED charlieLED;

class Charlieplex {
public:
	Charlieplex(byte* userPins, byte numberOfUserPins);
	void write(charlieLED pin, bool state);
	void writePwm(charlieLED pin, byte dutyCycle);
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
