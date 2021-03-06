# arduino-ir-tracking-sensors-test

> NOTE: I've moved this to/rewritten it in a PlatformIO project [here](https://github.com/joedski/arduino-ir-range-sensor).

More specific test for my IR tracking robot head.

I wrote the Distance Sense part of the code as a struct/class singleton to organize my thoughts better.



## Pin Requirements

Since I'm basically bodging Vishay's Fast Proximity stuff on to a simple analog sensor differencer, there's a few pins I am restricted from using due to using Timer 1 and Timer 2.

- Analog Sensor Difference:
  - Inputs:
    - Sensor Input: `A0`
    - VRef Input: `A1`
  - Outputs:
    - Higher Output: `5`
    - Lower Output: `6`
- Distance Detection:
  - Inputs:
    - `A2`: Used to read a pot to determine the effective carrier strength for our IR Signal LED.
    - `8`/`PB0`/`ICP1`: Edge detector, connected to the output of the TSOP38238.
  - Outputs:
    - `3`/`PD3`/`OC2B`: PWM for the IR Signal LED.
  - Blocked:
    - `11`/`OC2A`: Since we're using Pin 3 for PWM output, we don't really have the use of 11.
- RGB Status LEDs:
  - Outputs:
    - `13`: Eh, we can use any pin.  I'll use ... pin 13!  The LED pin!  Used for LEDs!  Hah hah hah.  hah.



## Timer Considerations


### Timer 2: Distance Sense Signal Carrier

- Setup:
  - Control Registers:
    - `TCCR2[A:B]` are set to have only the following bits On:
      - `TCCR2A.WGM20 & TCCR2B.WGM22` - Wave Generation Mode 5: Phase Correct PWM with Top = `OCR2A`.
      - `TCCR2B.CS20` - Use System Clock with No Prescaler.
  - Pin I/O Registers:
    - `DDRD` has the following pin set to Output (bit = On):
      - `DDRD.DDD3`
- Control:
  - `OCR2A` - Timer 2 Top value.
    - This is set to the Half Period (in System Clock Ticks) of our target frequency.
  - `OCR2B` - Timer 2 Carrier Duty Cycle.
    - This is set to `OCR2A * Duty%` to determine the Duty Cycle.
    - A 30% Duty Cycle is `OCR2A * 0.3f`, but as an int obviously.
  - `TCNT2` - Timer 2 Counter.  We reset this to 0 when starting to ensure a well formed initial pulse.
    - Not sure if it's necessary or not, probably not, but oh well?
  - `TCCR2A.COM2B1` - Turn this bit On to turn Pin 3 on for the duty cycle.
  - `TIMSK2.IOIE2` - Enable Timer 2 Overflow Interrupt.  Using the Overflow Interrupt Handler to count Carrier Pulses.


### Timer 1: Distance Sense Event Listener

- Control Register Setup:
  - `TCCR1A` is left with all bits Off.
  - `TCCR1B` is set to have only the following bits On:
    - `TCCR1B.CS11 & TCCR1B.CS10` - Use System Clock with Prescaler of 64.
    - `TCCR1B.ICNC1` - Enable Input Capture Noise Cancelation
- Control:
  - `TCCR1B.ICES1` - Input Capture Edge Select, set to 0 when starting sensing to listen for a falling edge, then set to 1 after the initial falling edge is detected to listen for the corresponding rising edge.
    - We start with falling edge because the TSOP38238 and other such IR sensors have Active-Low outputs, meaning their output is normally high, then is pulled low when they've detected a signal.
    - Thus, Falling Edge means Sensor Began Recognizing a Signal, Rising Edge means Sensor Stopped Recognizing a Signal.
    - Since there's only one Input Capture Event Interrupt Handler, we check this bit to determine which edge we're reacting to.
  - `TIMSK1.ICIE1` - Input Capture Event Interrupt Enable.  Lets us detect those edges!
  - `TCNT1` - Timer 1 Counter.  We reset this to 0 when we hit a falling edge.
  - `ICR1` - Timer 1 Input Capture Register.  We read this when we hit a rising edge.



## Outline of Operation

There are basically 2 parts, only one of which is really all that complicated.  Due to asynchrony, the main body of the Loop is a simple Finite State Machine.

- Setup:
  - Analog Sensor Difference:
    - Set Pins `5` and `6` as Outputs.
  - Distance Detection:
    - Setup Timer 2 as our Signal Carrier Modulator:
      - Set the Mode to Mode 5: Phase Correct PWM
      - Disconnect `OC2A` and `OC2B` to start with
        - `OC2A` is left disconnected; `OC2B` will be enabled later during Carrier Bursts.
      - Set `OCR2A` to the Half Period of our frequency
      - Set `OCR2B` to a fraction of `OCR2A` according to the desired cycle duty.
      - TODO: Enable the Timer Overflow Interrupt `TIMSK2.TOIE2` and define `ISR(TIMER2_OVF_vect)`?  Could be used to send only the desired number of pulses.
      - Set Pin `3` (`PD3`) as on Output.
    - Setup Timer 1 as our Resposne Timer:
      - Set the Mode to Mode 5: Phase Correct PWM
      - Set Timer 1's Prescaler to 64
      - Set Edge Trigger Direction to Falling
      - Enable the Input Edge Detector (`ICES`) and Input Noise Cancelation. (`ICNC`)
        - Later, `TIMSK1.ICIE` will be enabled to actually call the Input Capture ISR
      - Set Pin `8` (`ICP1`/`PB0`) as an Input.
      - Define the Input Capture ISR `ISR(TIMER1_CAPT_vect)`
  - RGB Status LEDs:
    - Main Strip Turn On!
- Operation:
  - Update from Analog Sensor:
    - Read Differential Input `A0` and VRef Input `A1`.
    - Update `5` and `6` after comparing those values.
  - Update from Distance Detection:
    - Read Power Setting `A2`.
    - Configure Timer 1 and Timer 2 according to that value.
      - ??
      - First thing will be to just vary the cycle duty on Timer 2.  I'll do from 0% ~ 70%.
      - Calculate Timer 1 lower and upper bounds based on calculated timing for a 25 pulse burst on Timer 2.
    - Initiate 25 pulse burst on IR Emitter on pin `3`.
    - Enable Timer 2's Overflow Interrupt `TIMSK2.TOIE2`.
    - Set Timer 1 to listen for falling edge and enable Input Capture Interrupt `TIMSK1.ICIE`.
    - On Timer 2 Overflow: Increment
    - On Input Capture Interrupt:
      - If set for falling edge, input pulse has started:
        - Reset Timer 1 Count to 0.
        - Set Timer 1 to listen for rising edge.
        - Return.
      - If set for rising edge, input pulse has ended:
        - Read Timer 1 Count and store.
        - Disable Input Capture Interrupt `TIMSK1.ICIE`
        - Set Timer 1 to listen for rising edge.
        - Update State to Input Capture Completion.
    - On Input Capture Completion:
      - Check if received pulse length was within calculated bounds for a 25 pulse burst.
        - Signal by status LED if yes or no.


### Calculating the Bounds of Timer 1

Since initially, I'm going with the center frequency of the TSOP38238, 38kHz, I can use the typical Sensor Output Pulse Times as the acceptable pulse range.  In the Fast Proximity note they used Time Carrier is Received (Tpi) less 5 pulses (5 / f0) as the lower bound and plus 6 pulses (6 / f0) as the upper bound of time for the Time Sensor Outputs (Tpo) to be considered a valid receipt of a carrier-modulated signal.

- Tpi - 5 / f0 \< Tpo \< Tpi + 6 / f0

Ordinarily, we could just assume the duration of a pulse length is a fixed value if the number of pulses is also fixed, but since frequency modulation may be a thing, we first need to calculate the actual time taken by a given pulse length.

- Given:
  - Carrier Pulse Count (Carrier-Pulses) n = 25
  - Carrier Modulation Frequency (Hz) f0 = 38k
  - Half Period (Timer 2 Ticks) t2 = 16MHz / (2 * f0)
    - f0 (Hz) = 16MHz / 2 / t2 = 16MHz / (2 * t2)
  - Time of Pulse Train Input (Seconds) Tpi = n / f0
  - Timer 1 Prescaler (Unitless) T1pre = 64
- Find Timer 1 Count Lower Bound (Timer 1 Ticks) T1nLower, Timer 1 Count Upper Bound (Timer 1 Ticks) T1nUpper
  - T1nLower = T1tps * ((n - 5) / f0)
    - :: (Timer-1-Ticks / Seconds) * ((Carrier-Pulses) / (Carrier-Pulses / Seconds))
    - :: Timer-1-Ticks
    - = 16MHz / T1pre * (n - 5) / f0
    - = (16MHz / (T1pre * f0)) * (n - 5)
    - NOTE: n - 5 is for the normal frequency.  If using off-center frequencies for the carrier modulation, consider numbers as low as n - 15.
  - T1nUpper = T1tps * ((n + 6) / f0)
    - = 16MHz / T1pre * (n + 6) / f0
    - = (16MHz / (T1pre * f0)) * (n + 6)
  - Find Timer 1 Ticks per Second (Timer-1-Ticks / Seconds) T1tps
    - T1tps = 16MHz / T1pre



## Sources

Useful links:
- [Vishay App Note on Fast Proximity Sensing](http://www.vishay.com/docs/82741/tssp4056sensor.pdf)
- [Vishay App Note on Slow Proximity Sensing with Directionality](https://www.vishay.com/docs/82729/tsspagcpsensor.pdf)
  - [Code mentioned in the Slow Proximity Sensing note](http://www.vishay.com/doc?82728)
- [Next best thing to the ATMega32 data sheet regarding FWM](https://www.arduino.cc/en/Tutorial/SecretsOfArduinoPWM)
- [Question on using Timer 1 to measure the time between a rising edge and falling edge](https://arduino.stackexchange.com/questions/8782/how-to-use-timer1-at328mega-to-measure-the-time-between-rising-edges-of-two-in)
  - Which is basically what I need to do, inverted...
- [FastLED Library](http://fastled.io/)
  - Includes not just a library for driving all manner of RGB LEDs, but also some very handy and performant 8-bit math functions for various animation calculations.
  - Also HSV color model support, which is always excellent.
