//- -----------------------------------------------------------------------------------------------------------------------
// AskSin++
// 2016-10-31 papa Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
//- -----------------------------------------------------------------------------------------------------------------------

#ifndef __ALARMCLOCK_H__
#define __ALARMCLOCK_H__

#include "Debug.h"
#include "Alarm.h"


namespace as {

#ifndef TICKS_PER_SECOND
  // default 100 ticks per second
  #define TICKS_PER_SECOND 100UL
#endif

#define seconds2ticks(tm) ( tm * TICKS_PER_SECOND )
#define ticks2seconds(tm) ( tm / TICKS_PER_SECOND )

#define decis2ticks(tm) ( tm * TICKS_PER_SECOND / 10 )
#define ticks2decis(tm) ( tm * 10UL / TICKS_PER_SECOND )

#define centis2ticks(tm)  ( tm * TICKS_PER_SECOND / 100 )
#define ticks2centis(tm)  ( tm * 100UL / TICKS_PER_SECOND )

#define millis2ticks(tm) ( tm * TICKS_PER_SECOND / 1000 )
#define ticks2millis(tm) ( tm * 1000UL / TICKS_PER_SECOND )

class AlarmClock: protected Link {

  Link ready;

public:

  void cancel(Alarm& item);

  AlarmClock& operator --();

  bool isready () const {
    return ready.select() != 0;
  }

  bool runready() {
    bool worked = false;
    while( runsingle()==true ) {
      worked=true;
    }
    return worked;
  }

  bool runsingle() {
    Alarm* a = (Alarm*) ready.unlink();
    if (a != 0) {
      a->active(false);
      a->trigger(*this);
      return true;
    }
    return false;
  }

  bool runwait () {
    return runsingle();
  }

  void add(Alarm& item);

  uint32_t get(const Alarm& item) const;

  uint32_t next () const {
    Alarm* n = (Alarm*)select();
    return n != 0 ? n->tick : 0;
  }

  Alarm* first () const {
    return (Alarm*)select();
  }

  // correct the alarms after sleep
  void correct (uint32_t ticks) {
    ticks--;
    Alarm* n = first();
    if( n != 0 ) {
      uint32_t nextticks = n->tick-1;
      n->tick -= nextticks < ticks ? nextticks : ticks;
    }
    --(*this);
  }

};


extern void callback(void);
extern void rtccallback(void);


class SysClock : public AlarmClock {
public:
  static SysClock& instance();

  void init() {
  #if ARDUINO_ARCH_AVR
  #define TIMER1_RESOLUTION 65536UL  // Timer1 is 16 bit
    // use Timer1 on AVR
    TCCR1B = _BV(WGM13);        // set mode as phase and frequency correct pwm (WGM13:0=1000), stop the timer (CS12:0=0)
    TCCR1A = 0;                 // clear control register A
    const unsigned long cycles = (F_CPU / 2000000) * (1000000 / TICKS_PER_SECOND);
    unsigned short pwmPeriod;
    unsigned char clockSelectBits;

    if (cycles < TIMER1_RESOLUTION) {
      clockSelectBits = _BV(CS10); // no prescaling
      pwmPeriod = (unsigned short)cycles;
    }
    else if (cycles < TIMER1_RESOLUTION * 8) {
      clockSelectBits = _BV(CS11); // 1/8 prescaling
      pwmPeriod = cycles / 8;
    }
    else if (cycles < TIMER1_RESOLUTION * 64) {
      clockSelectBits = _BV(CS11) | _BV(CS10); // 1/64 prescaling
      pwmPeriod = cycles / 64;
    }
    else if (cycles < TIMER1_RESOLUTION * 256) {
      clockSelectBits = _BV(CS12); // 1/256 prescaling
      pwmPeriod = cycles / 256;
    }
    else if (cycles < TIMER1_RESOLUTION * 1024) {
      clockSelectBits = _BV(CS12) | _BV(CS10); // 1/1024 prescaling
      pwmPeriod = cycles / 1024;
    }
    else {
      clockSelectBits = _BV(CS12) | _BV(CS10);
      pwmPeriod = TIMER1_RESOLUTION - 1;
    }

    DPRINT("Prescaling set to ");DHEXLN((int)(cycles/pwmPeriod));
	DPRINT("PWM-Period set to ");DHEXLN((int)pwmPeriod);
	
    TCNT1 = 0; 								// reset timer1 counter value
    ICR1 = pwmPeriod;					   	// set input capture register to PWM period
    TCCR1B = _BV(WGM13) | clockSelectBits; 	// set prescaler bits to start counter
  #endif
  #ifdef ARDUINO_ARCH_STM32F1
    // Setup Timer2 on ARM
    Timer2.setMode(TIMER_CH2,TIMER_OUTPUT_COMPARE);
    Timer2.setPeriod(1000000 / TICKS_PER_SECOND); // in microseconds
    Timer2.setCompare(TIMER_CH2, 1); // overflow might be small
  #endif
    enable();
  }

  // Disable Timer1 Overflow Interrupt
  void disable () {
  #ifdef ARDUINO_AVR_ATmega32
    TIMSK &= ~_BV(TOIE1);
  #elif defined(ARDUINO_ARCH_AVR)
    TIMSK1 &= ~_BV(TOIE1);
  #elif defined(ARDUINO_ARCH_STM32F1)
    Timer2.detachInterrupt(TIMER_CH2);
  #endif
  }

  // Enable Timer1 Overflow Interrupt
  void enable () {
  #ifdef ARDUINO_AVR_ATmega32
    TIMSK |= _BV(TOIE1);
  #elif defined(ARDUINO_ARCH_AVR)
    TIMSK1 |= _BV(TOIE1);
  #elif defined(ARDUINO_ARCH_STM32F1)
    Timer2.attachInterrupt(TIMER_CH2,callback);
  #endif
  }

  void add(Alarm& item) {
    AlarmClock::add(item);
  }

  void add(Alarm& item,uint32_t millis) {
    item.tick = (millis2ticks(millis));
    add(item);
  }
};

extern SysClock sysclock;



class RTC : public AlarmClock {
  uint8_t ovrfl;
#if defined(ARDUINO_ARCH_STM32F1) && defined(_RTCLOCK_H_)
  RTClock rt;
#endif
public:

  RTC () : ovrfl(0) {}

  static RTC& instance();

  void init () {
#ifdef ARDUINO_AVR_ATmega32
    TIMSK &= ~(1<<TOIE2); //Disable timer2 interrupts
    ASSR  |= (1<<AS2); //Enable asynchronous mode
    TCNT2 = 0; //set initial counter value
    TCCR2 = (1<<CS22)|(1<<CS20); // mode normal & set prescaller 128
    while (ASSR & (1<<TCN2UB)); //wait for registers update
    TIFR |= (1<<TOV2); //clear interrupt flags
    TIMSK |= (1<<TOIE2); //enable TOV2 interrupt
#elif ARDUINO_AVR_PROMICRO
    TIMSK4  = 0; //Disable timer4 interrupts
    TCNT4 = 0; //set initial counter value
    TCCR4A = 0; // mode normal
    TCCR4B = (1<<CS43); //set prescaler 128
	//Counter Initialization for Asynchronous Mode (according to datasheet)
	PLLCSR = (1<<PLLE); //enable PLL
	_delay_us(100); // wait 100us for PLL to stabilize
    while (PLLCSR & (1<<PLOCK)); //wait for PLL to lock
	PLLFRQ |= (1<<PLLTM0); // enable asnchronous mode with PLL Postcaler Factor of one
    TIFR4  = (1<<TOV4); //clear interrupt flags
    TIMSK4  = (1<<TOIE4); //enable TOV2 interrupt
#elif defined(ARDUINO_ARCH_AVR)
    TIMSK2  = 0; //Disable timer2 interrupts
    ASSR  = (1<<AS2); //Enable asynchronous mode
    TCNT2 = 0; //set initial counter value
    TCCR2A = 0; // mode normal
    TCCR2B = (1<<CS22)|(1<<CS20); //set prescaler 128
    while (ASSR & ((1<<TCN2UB)|(1<<TCR2BUB))); //wait for registers update
    TIFR2  = (1<<TOV2); //clear interrupt flags
    TIMSK2  = (1<<TOIE2); //enable TOV2 interrupt
#elif defined(ARDUINO_ARCH_STM32F1) && defined(_RTCLOCK_H_)
    rt = RTClock(RTCSEL_LSE);
    rt.attachSecondsInterrupt(rtccallback);
#else
  #warning "RTC not supported"
#endif
  }

  // return millis done of the current second
  uint32_t getCurrentMillis () {
#ifdef ARDUINO_AVR_PROMICRO
    return (TCNT4 * 1000) / 255;
#elif ARDUINO_ARCH_AVR
    return (TCNT2 * 1000) / 255;
#else
    return 0; // not supported ???
#endif
  }

  uint32_t getCounter (bool resetovrflow) {
    if( resetovrflow == true ) {
      ovrfl = 0;
    }
#ifdef ARDUINO_AVR_PROMICRO
    return (256 * ovrfl) + TCNT4;
#elif ARDUINO_ARCH_AVR
    return (256 * ovrfl) + TCNT2;
#elif defined(ARDUINO_ARCH_STM32F1) && defined(_RTCLOCK_H_)
    return rtc_get_count();
#else
    return 0;
#endif
  }

  void overflow () {
    ovrfl++;
  }

  void debug () {
    if( select() != 0 ) {
      DDEC((uint16_t)((Alarm*)select())->tick);
      DPRINT(F(" "));
    }
  }

  void add(Alarm& item) {
    AlarmClock::add(item);
  }

  void add(Alarm& item,uint32_t millis) {
    millis += getCurrentMillis();
    item.tick = (millis / 1000); // seconds to wait
    add(item);
  }

  void add(RTCAlarm& item,uint32_t millis) {
    // correct the time with the millis of the current second
    millis += getCurrentMillis();
    item.tick = (millis / 1000); // seconds to wait
    item.millis = millis % 1000; // millis to wait
    add(item);
  }

};

extern RTC rtc;

}

#endif
