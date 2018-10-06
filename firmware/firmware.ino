#include <avr/sleep.h>
#include "Charlieplex.h"

#define LED_ON_TIME 3 // (3 - 750) Higher -> +Brightness +Flickering

// Charlieplex Control Pins (RTL Progression w/ BTT Lines)
byte pins[] = {3, 5, 6, 9, 10, 11};
Charlieplex cp = Charlieplex(pins, 6);
charlieLED led[] =
{
    // RED   GREEN  YELLOW
    {4, 1}, {5, 4}, {0, 5},
    {4, 2}, {5, 2}, {0, 4},
    {4, 3}, {5, 3}, {0, 3},
    {4, 5}, {5, 1}, {0, 2},
    {3, 1}, {2, 3}, {1, 5},
    {3, 2}, {2, 5}, {1, 4},
    {3, 4}, {2, 1}, {1, 3},
    {3, 5}, {2, 4}, {1, 2}
};

void setup() {
    cp.clear();

    // Enable pin change interrupt on PB0 (PCINT0)
    PCMSK0 |= bit (PCINT0); //mask for PCINT0
    PCIFR |= bit (PCIF0); //clear any outstanding interrupts
    PCICR |= bit (PCIE0); //enable PCINT0 - PCINT5
}

void loop() {
    // Accurate Timing w/ External Crystal & Timer2?
    // Sleep modes?
    // PCINT Wakeup w/ Touch?
    // Clear when interrupted?

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_mode();

    for (int i = 0; i <= 24; i++) {
        display(i, 100);
        delay(100);
    }
}

// Blank ISR for PCINT0 (Only wakes if sleeping)
EMPTY_INTERRUPT (PCINT0_vect);

// Turn on a given number of LEDs (RTL, BTT order) for a given time
void display(byte number, unsigned long milliseconds) {
    unsigned long startMillis = millis();
    while (millis() - startMillis < milliseconds) { /// will this work on millis() reset?
        for (int i = 0; i < number; i++) {
            cp.write(led[i], HIGH);
            delayMicroseconds(LED_ON_TIME);
            cp.write(led[i], LOW);
        }
    }
}
