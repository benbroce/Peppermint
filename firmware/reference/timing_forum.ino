
//
// ATmega328p Solar/SuperCap powered Real Time Clock and voltage monitor.
// VirtualWire data  transmission
//
// sjames dot remington / gmail
//
// This code incorporates an RTC formed by Timer2 and a 32768 xtal on OSC pins
// The internal 8MHz RC oscillator is calibrated by comparison with the 32768 Hz standard
//
// Mini-uino board from http://crossroadsfencing.com/BobuinoRev17/index.html
// fuses: 8 MHz internal RC clock, LP xtal (32768 Hz) on OSC pins
//
//  Binary BCD HH:MM:SS time version, RTC maintained in global array RTC_buf[]
//  Data output on PORTD,3 (VirtualWire TX)
//  Power output on PORTD,2 (TX module)

#include <avr/sleep.h>
#include <avr/power.h>
#include <util/delay.h>
#include <VirtualWire.h>

// Global variables for RTC, time and day
// set the time here, if you like
volatile unsigned char RTC_buf[]={
  0,0,0,0,0,0}; //binary BCD hh:mm:ss (24 hour clock)
volatile unsigned int dayno=0; //days since startup

char buf[40]; //sprintf buffer, about 30 chars actually used
int VW_PWR=2;    //433 MHz TX power
int VW_TX=3;     //Virtual Wire TX

/*
** approximate 1ms delays (can't use Arduino delay - timer disabled)
 */

void delay1ms(unsigned int ms)
{
  while (ms--) _delay_ms(1);
}

// everything happens in setup()

void setup()
{
  char t,zero=0;
  unsigned int batt,h,m,s;

  pinMode(VW_PWR,OUTPUT);  //transmitter power
  digitalWrite(VW_PWR,LOW); // TX off
  pinMode(VW_TX,OUTPUT);
  digitalWrite(VW_TX,LOW);

  //PRR Power Reduction Register (set PRADC after ADCSRA=0)
  //Bit 7 - PRTWI: Power Reduction TWI
  //Bit 6 - PRTIM2: Power Reduction Timer/Counter2
  //Bit 5 - PRTIM0: Power Reduction Timer/Counter0
  //Bit 3 - PRTIM1: Power Reduction Timer/Counter1
  //Bit 2 - PRSPI: Power Reduction Serial Peripheral Interface
  //Bit 1 - PRUSART0: Power Reduction USART0
  //Bit 0 - PRADC: Power Reduction ADC

  ADCSRA = 0; //disable ADC
  PRR |= (1<<PRTWI)|(1<<PRSPI)|(1<<PRTIM0)|(1<<PRUSART0)|(1<<PRADC); //need Timers 1 and 2

  OSCCAL_calibrate();  //calibrate the RC osc for accurate timing using the 32 kHz crystal

  timer2_init(); //setup timer2 interrupts for RTC

  vw_set_tx_pin(VW_TX);  //initialize VirtualWire TX
  vw_setup(2000); // and bits per sec

  sei();  //enable interrupts
  t=255; //initialize loop timer

  while(1) {

    // wake up on Timer2 overflow (1/sec)
    // output day, time and cpu voltage via VirtualWire every 10 minutes

    if(t != RTC_buf[3]) {

      t=RTC_buf[3];
      s=10*RTC_buf[4]+RTC_buf[5]; //format time (seconds)
      m=10*RTC_buf[2]+RTC_buf[3]; //minutes
      h=10*RTC_buf[0]+RTC_buf[1]; //hours

      digitalWrite(VW_PWR,HIGH);  //power on TX

      PRR &= ~(1<<PRADC); //turn ADC back on
      ADCSRA = (1<<ADEN);  //enable and
      ADCSRA |= (1<<ADPS0) | (1<<ADPS1) | (1<<ADPS2);  // set prescaler to 128

      delay1ms(10); //let ADC and TX stabilize
      batt = readVcc(); //get Vcc voltage

      // format days, time and battery voltage
      sprintf(buf,"%0u,%0d,%0d,%0d,%0u%c",dayno,h,m,s,batt,zero); //max ~16 characters

      vw_send((uint8_t *)buf, strlen(buf)); //send it
      vw_wait_tx(); // Wait until message is sent

      digitalWrite(VW_TX,LOW); //power off TX
      digitalWrite(VW_PWR,LOW);

      ADCSRA = 0; //ADC off
      PRR |= (1<<PRADC);
    } //end if (t)

    //go back to sleep
    set_sleep_mode(SLEEP_MODE_PWR_SAVE);
    sleep_enable();
    cli(); //time critical steps follow
    MCUCR = (1<<BODS) | (1<<BODSE); // turn on brown-out enable select
    MCUCR = (1<<BODS);        //Brown out off. This must be done within 4 clock cycles of above
    sei();
    sleep_cpu();
  } //end while(1)
}

void loop() {
}  //do everything in setup()

//******************************************************************
//  Timer2 Interrupt Service
//  32 kKz / 256 = 1 Hz with prescaler 128
//  provides binary BCD Real Time Clock
//  no check for illegal values of RTC_buffer upon startup!

ISR (TIMER2_OVF_vect) {

  // RTC function

  RTC_buf[5]++; // increment second

  if (RTC_buf[5] > 9)
  {
    RTC_buf[5]=0; // increment ten seconds
    RTC_buf[4]++;
    if ( RTC_buf[4] > 5)
    {
      RTC_buf[4]=0;
      RTC_buf[3]++; // increment minutes
      if (RTC_buf[3] > 9)
      {
        RTC_buf[3]=0;
        RTC_buf[2]++; // increment ten minutes

        if (RTC_buf[2] > 5)
        {
          RTC_buf[2]=0;
          RTC_buf[1]++; // increment hours
          char b = RTC_buf[0]; // tens of hours, handle rollover at 19 or 23
          if ( ((b < 2) && (RTC_buf[1] > 9)) || ((b==2) && (RTC_buf[1] > 3)) )
          {
            RTC_buf[1]=0;
            RTC_buf[0]++; // increment ten hours and day number, if midnight rollover
            if (RTC_buf[0] > 2) {
              RTC_buf[0]=0;
              dayno++; //one day at a time...
            }
          }
        }
      }
    }
  }
}


/*
// initialize Timer2 as asynchronous 32768 Hz timing source
 */

void timer2_init(void) {
  TCCR2B = 0;  //stop Timer 2
  TIMSK2 = 0; // disable Timer 2 interrupts
  ASSR = (1<<AS2); // select asynchronous operation of Timer2
  TCNT2 = 0; // clear Timer 2 counter
  TCCR2A = 0; //normal count up mode, no port output
  TCCR2B = (1<<CS22) | (1<<CS20); // select prescaler 128 => 1 sec between each overflow

  while (ASSR & ((1<<TCN2UB)|(1<<TCR2BUB))); // wait for TCN2UB and TCR2BUB to clear

  TIFR2 = 0xFF; // clear all interrupt flags
  TIMSK2 = (1<<TOIE2); // enable Timer2 overflow interrupt
}

// Read 1.1V reference against AVcc
// return battery voltage in millivolts
// must be individually calibrated for each CPU

unsigned int readVcc(void) {

  unsigned int result;

  // set the reference to Vcc and the measurement to the internal 1.1V reference

  ADMUX = (1<<REFS0) | (1<<MUX3) | (1<<MUX2) | (1<<MUX1);
  delay1ms(2); // Wait for Vref to settle

  ADCSRA |= (1<<ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // wait until done
  result = ADC;

  // second time is a charm

  ADCSRA |= (1<<ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // wait until done
  result = ADC;

  // calibrated for my Miniduino

  result = 1195700UL / (unsigned long)result; //1126400 = 1.1*1024*1000
  return result; // Vcc in millivolts
}

//
// Calibrate the internal OSCCAL byte, using the external 32768 Hz crystal as reference.
//

void OSCCAL_calibrate(void)  //This version specific to ATmegaXX8
{
  unsigned char calibrate = 0; //not calibrated;
  unsigned int temp;

  TIMSK1 = 0; //disable Timer1,2 interrupts
  TIMSK2 = 0;

  ASSR = (1<<AS2);        //select asynchronous operation of timer2 (32,768kHz)
  OCR2A = 200;            // set timer2 compare value
  TCCR1A = 0;
  TCCR1B = (1<<CS11);     // start timer1 with prescaler 8
  TCCR2B = (1<<CS20);     // start timer2 with no prescaling (ATmega169 use TCCR2A!)

  while (ASSR & ((1<<TCN2UB)|(1<<TCR2BUB)));  //wait for TCN2UB and TCR2BUB to be cleared

  delay1ms(2000); //allow xtal osc to stabilize

  while(!calibrate)
  {
    cli(); // disable global interrupt

    TIFR1 = 0xFF;   // clear Timer1 flags
    TIFR2 = 0xFF;   // clear Timer2 flags

    TCNT1 = 0;      // clear timer1 counter
    TCNT2 = 0;      // clear timer2 counter

    TCCR1B = (1<<CS11); // start timer1 (again)

    while ( !(TIFR2&(1<<OCF2A)) );   // wait for timer2 compareflag

    TCCR1B = 0; // stop timer1

    sei(); // reenable global interrupts

    if ( TIFR1&(1<<TOV1) ) temp = 0xFFFF;   //overflow, load max
    else   temp = TCNT1;

    //expect about (1e6/32768)*201 = 6134 ticks

    if ( (temp >= 6125) && (temp <= 6145) ) calibrate=1;
    if (temp > 6145) OSCCAL--;   //RC oscillator runs too fast, decrease OSCCAL
    if (temp < 6125) OSCCAL++;   //RC oscillator runs too slow, increase OSCCAL

  } //end while(!calibrate)

} //return
