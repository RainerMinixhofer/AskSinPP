//- -----------------------------------------------------------------------------------------------------------------------
// AskSin++
// 2017-03-29 papa Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
// 2019-01-11 scuba82: highly inspired by https://github.com/blackhack/ArduLibraries/tree/master/DimmerOne
//- -----------------------------------------------------------------------------------------------------------------------
/*
	Fires an interrupt, each time the zero-cross detection pin changes. Output pin will be held up high (trailing-edge phase cut) for a certain time,
	correspondig to the called dim value. The pre defined values are calculated for 50Hz mains, 8Mhz CPU frequence and a prescaler of 1024.
	PHASECUTMODE == 1 -> leading-edge phase cut; PHASECUTMODE == 0 	trailing-edge phase cut
*/

#include "PhaseCut.h"
namespace as {

#if defined(ARDUINO_ARCH_AVR) && ! (defined(ARDUINO_AVR_ATmega32) || defined(__AVR_ATmega644__))
	PhaseCut phaseCut;

	PhaseCut::PhaseCut()
	{
		isInit = false;
	}

	void PhaseCut::init(uint8_t output_pin)
	{
		if (isInit)
			return;
		isInit = true;
		ZERO_CROSS_PIN = ZEROPIN;
		OUTPUT_PIN = output_pin;
		_timer = 0;
		running = false;

		pinMode(OUTPUT_PIN, OUTPUT);
		pinMode(ZERO_CROSS_PIN, INPUT);

	#if defined(__AVR_ATmega32U4__)
		TCCR3A = 0;
		TCCR3B = 0;
	#else
		TCCR2A = 0;
		TCCR2B = 0;
	#endif

		uint8_t oldSREG = SREG;
		cli();
		SetTimer();
		SREG = oldSREG;

	#if defined(__AVR_ATmega32U4__)
		TIMSK3 |= (1 << OCIE3A);     // Enable COMPA and COMPB interruptions of TIMER3
	#else
		TIMSK2 |= (1 << OCIE2A);     // Enable COMPA and COMPB interruptions of TIMER2
	#endif
		_valid_zero_crossing = true;
	}

	bool PhaseCut::Start()
	{
		if (!isInit)
			return false;
		uint8_t oldSREG = SREG;
		cli();
		SREG = oldSREG;
		attachInterrupt(digitalPinToInterrupt(ZERO_CROSS_PIN), ZeroCrossEventCaller, CHANGE);
		running = true;
		return true;
	}

	bool PhaseCut::Stop()
	{
		if (!isInit)
			return false;
		detachInterrupt(digitalPinToInterrupt(ZERO_CROSS_PIN));
		digitalWrite(OUTPUT_PIN, LOW);
		running = false;
		return false;
	}

	bool PhaseCut::isrunning()
	{
		return running;
	}
	bool PhaseCut::SetDimValue(double value)
	{
		if (!isInit)
			return false;
		_timer = value;
		 
		return true;
	}

	void PhaseCut::SetTimer()
	{
		#if defined(__AVR_ATmega32U4__)
			OCR3A = _timer; 
		#else
			OCR2A = _timer; 
		#endif
	}

	void ZeroCrossEventCaller()
	{
		phaseCut.ZeroCrossEvent();
	}


	ISR(TIMER2_COMPA_vect)
	{
		phaseCut.CmpAEvent();
	}

	void PhaseCut::ZeroCrossEvent()
	{
		if (!_valid_zero_crossing)
			return;
		_valid_zero_crossing = false;
		phaseCut.Fire();
	#if defined(__AVR_ATmega32U4__)
		TCNT3 = 0;   // Restart counter (no need to call cli() inside an ISR)
		TCCR3B |= (0 << WGM33) |(1 << WGM32) |(0 << WGM31) |(0 << WGM30) | (1 << CS30) | (0 << CS31) | (1 << CS32) ; // Enable/start CTC and set prescaler to 1024
	#else
		TCNT2 = 0;   // Restart counter (no need to call cli() inside an ISR)
		TCCR2B |= (1 << WGM21) | (1 << CS20) | (1 << CS21) | (1 << CS22) ; // Enable/start CTC and set prescaler to 1024
	#endif
	}

	void PhaseCut::Fire()
	{
				#if PHASECUTMODE == 1
					digitalWrite(OUTPUT_PIN, LOW);
				#else
					if ( _timer > 0 ) digitalWrite(OUTPUT_PIN, HIGH);
				#endif
		
	}

	void PhaseCut::CmpAEvent()
	{
				#if PHASECUTMODE == 1
					if ( _timer < 75 )digitalWrite(OUTPUT_PIN, HIGH);
				#else
					digitalWrite(OUTPUT_PIN, LOW);
				#endif
				
		#if defined(__AVR_ATmega32U4__)
			TCCR3B = 0; // Disable/stop CTC
		#else
			TCCR2B = 0; // Disable/stop CTC
		#endif
			SetTimer();
			_valid_zero_crossing = true;
	}
#endif
}
