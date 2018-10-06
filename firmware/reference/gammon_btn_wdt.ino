#include <avr/sleep.h>
#include <avr/wdt.h>

#define LED 13

// interrupt service routine for when button pressed
void wake ()
{
  wdt_disable();  // disable watchdog
}  // end of wake

// watchdog interrupt
ISR (WDT_vect)
{
  wake ();
}  // end of WDT_vect

void myWatchdogEnable (const byte interval)
{
  noInterrupts ();   // timed sequence below

  MCUSR = 0;                          // reset various flags
  WDTCSR |= 0b00011000;               // see docs, set WDCE, WDE
  WDTCSR =  0b01000000 | interval;    // set WDIE, and appropriate delay
  wdt_reset();

  byte adcsra_save = ADCSRA;
  ADCSRA = 0;  // disable ADC
  power_all_disable ();   // turn off all modules
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);   // sleep mode is set here
  sleep_enable();
  attachInterrupt (digitalPinToInterrupt (2), wake, LOW);   // allow grounding pin D2 to wake us
  interrupts ();
  sleep_cpu ();            // now goes to Sleep and waits for the interrupt
  detachInterrupt (digitalPinToInterrupt (2));     // stop LOW interrupt on pin D2

  ADCSRA = adcsra_save;  // stop power reduction
  power_all_enable ();   // turn on all modules
}  // end of myWatchdogEnable

void setup ()
{
  digitalWrite (2, HIGH);    // pull-up on button
}  // end of setup

void loop()
{
  pinMode (LED, OUTPUT);
  digitalWrite (LED, HIGH);
  delay (5000);
  digitalWrite (LED, LOW);
  delay (5000);

  // sleep bit patterns:
  //  1 second:  0b000110
  //  2 seconds: 0b000111
  //  4 seconds: 0b100000
  //  8 seconds: 0b100001

  // sleep for 8 seconds
  myWatchdogEnable (0b100001);  // 8 seconds

}  // end of loop
