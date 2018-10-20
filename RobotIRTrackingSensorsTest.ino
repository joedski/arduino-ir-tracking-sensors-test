/**
 * A simple test that indicates whether an input is higher
 * or lower than a given reference.  IT was easier than
 * using a bunch of op amps.
 *
 * Inputs:
 * - IN_SENSE: The pin to read from for a sensor reading.
 * - IN_VREF: Pin to read for the center voltage reference.
 *     The voltage read from IN_SENSE is compared against this.
 *
 * Outputs:
 * - OUT_HIGHER: PWM output used to indicate the sensor voltage is
 *     higher than the reference.
 * - OUT_LOWER: PWM output used to indicate the sensor voltage is
 *     lower than the reference.
 */

// #include <FastLED.h>


// A0 is used for the diodes
const int IN_SENSE = 0
// A1 is used for the center voltage reference
const int IN_VREF = 1

// Pin to light up more when the higher photodiode conducts more
const int OUT_HIGHER = 5
// Pin to light up more when the lower photodiode conducts more
const int OUT_LOWER = 6

enum class States {
  LeftRightSense,
  DistanceSenseBeginning,
  DistanceSenseWaiting,
  DistanceSenseEnded,
  MotorDrive
}

// Since this will be updated by interrupts, it must be marked volatile.
volatile int currentState = States.LeftRightSense;


// Functionality
// --------------------------------

void setupDiodeDifferential() {
  pinMode(OUT_HIGHER, OUTPUT);
  pinMode(OUT_LOWER, OUTPUT);
}

void updateDiodeDifferential() {
  // put your main code here, to run repeatedly:
  int diodeValue = analogRead(IN_SENSE);
  int refValue = analogRead(IN_VREF);
  int difference = (diodeValue - refValue) / 2;

  if (difference > 0) {
    analogWrite(OUT_HIGHER, difference);
    analogWrite(OUT_LOWER, 0);
  }
  else if (difference < 0) {
    analogWrite(OUT_HIGHER, 0);
    analogWrite(OUT_LOWER, 0 - difference);
  }
  else {
    analogWrite(OUT_HIGHER, 0);
    analogWrite(OUT_LOWER, 0);
  }
}


// Distance Stuff

struct DistanceSense {
  // The _analog_ input we sample to determine
  // the effective strength of the carrier.
  const uint8_t signalStrengthInput;

  // NOTE: The other two pins we use here, OC2B and ICP1,
  // are not mentioned by name.
  // In the case of OC2B (PD3/Arduino Digital 3), we simply set
  // TCCR2A.COM2B1 on and off to toggle its activation.
  // And for ICP1 (PB8/Arduino Digital 8), we twiddle TIMSK1.ICIE1
  // to enable input capture event interrupts.

  struct Carrier {
    // Frequency of the carrier modulation.
    // :: Hz
    float f0;
    // Value on domain [0.0 1.0] expressing the duty of the carrier modulation.
    // :: Duty
    float duty;

    // How many Carrier-Pulses we emit per Burst.
    // :: Carrier-Pulses / Carrier-Burst
    uint8_t burstCountMax;
    // How many carrier pulses we should take the time-duration of
    // when calculating the timer-1-ticks-duration of the lower-bound
    // valid-received-signal value for Timer 1.
    // :: Carrier-Pulses
    uint8_t burstCountLowerBound;
    // Same as burstCountLowerBound, but for the upper-bound value.
    // :: Carrier-Pulses / Carrier-Burst
    uint8_t burstCountUpperBound;

    // Number of carrier pulses counted
    // so we know when to stop.
    // :: Carrier-Pulses
    volatile uint8_t pulsesRemaining;
    // :: Timer-2-Increments
    // :: (Clock-Ticks) / (2 * Carrier-Frequency)
    // = 16MHz / (2 * f0)
    uint8_t halfPeriod;
    // :: Timer-2-Increments
    // :: (Timer-2-Ticks) * Duty
    // = halfPeriod * duty
    uint8_t dutyCompValue;

    // Value recorded from strength input.
    // Has no real meaning out of context,
    // rather it is used as an abstract strength indication
    // from which abritrary implementation derives concrete values.
    // Or, it's just an input.  I do things with it.
    // :: Unitless
    int16_t strength;
  } carrier;

  struct Receiver {
    // The Timer 1 Counter must be higher than this value
    // by the time the sensor output goes inactive
    // for the signal to be considered valid.
    // :: Timer-1-Ticks
    uint16_t tickCountLowerBound;
    // Same as above, but the Timer 1 Counter must be
    // lower than this value.
    // :: Timer-1-Ticks
    uint16_t tickCountUpperBound;

    volatile uint16_t ticksElapsed;
  } receiver;

  /**
   * Set the carrier frequency and associated timer values.
   */
  void setF0(float nextF0) {
    carrier.f0 = nextF0;
    carrier.halfPeriod = (uint8_t)(16e6 / (2 * carrier.f0));
    carrier.dutyCompValue = (uint8_t)((float)carrier.halfPeriod * carrier.duty);
  }

  /**
   * Set the carrier duty and associated timer values.
   */
  void setDuty(float nextDuty) {
    carrier.duty = nextDuty;
    carrier.dutyCompValue = (uint8_t)((float)carrier.halfPeriod * carrier.duty);
  }

  /**
   * Updates the lower and uppert bounds for the receiver timer.
   * If the duration (in timer ticks) that the receiver is on
   * is outside of these bounds, then the signal is considered invalid.
   */
  void updateReceiverTickBounds() {
    float ticksPerPulse = 16e6 / (64e0 * carrier.f0);
    receiver.tickCountLowerBound = (uint16_t)(ticksPerPulse * carrier.burstCountLowerBound);
    receiver.tickCountUpperBound = (uint16_t)(ticksPerPulse * carrier.burstCountUpperBound);
  }

  void setup() {
    // First, setting up Timer 1
    // -------------------------

    // This is basically copied verbatim from the Slow Proximity note.
    // Granted, it does exactly what I want, and there's
    // only so many ways to twiddle bits.

    // Set Wave Gen Mode to Mode 5: PWM Phase Correct with Top = OCR2A
    //   (TCCR2A.WGM20 & TCCR2B.WGM22)
    // Use System Clock with No Prescaler (TCCR2B.CS20)
    // NOTE: We won't use OC2A for output, but we will use OC2B
    // in 1,0 mode (clear on match counting up, set on match counting down).
    TCCR2A = (1<<WGM20);// | (0<<COM2B1);
    TCCR2B = (1<<WGM22) | (1<<CS20);

    // set arduino D3, AVR PD3 as output pin
    // it can be hard-disabled as an output by setting it as an input pin,
    // but I think twiddling COM2B1 is fine.
    DDRD |= (1<<DDD3);

    // Next, setting up Timer 2
    // ------------------------

    // Here, we do the following:
    // - Enable Input Capture Noise Cancelation (TCCR1B.ICNC1)
    // - Set the Prescaler to 64 (TCCR1B.CS11 & TCCR1B.CS10)
    // - And, not really necessary during setup, but set the
    //   Input Capture Edge Select to Falling (TCCR1B.ICES1)
    TCCR1B = (1<<ICNC1) | (0<<ICES1) | (1<<CS11) | (1<<CS10);

    // We'll set OCR1A and OCR1B to appropriate values later,
    // then enable TIMSK1.ICIE1, etc.
  }

  void startSensing() {
    // To start sensing, we do the following:
    updateCarrierStrength();
    updateTimer1Params();
    updateTimer2Params();
    enableIO();
    ::currentState = States::DistanceSenseWaiting;
  }

  void updateCarrierStrength() {
    carrier.strength = ::analogRead(signalStrengthInput);

    // For now, we're just setting the duty based on this...
    // Which means we don't need to set any Timer 1 parameters.
    setDuty((float)carrier.strength / 1024.0);
    updateReceiverTickBounds();
  }

  void updateTimer2Params() {
    // Reset the pulses remaining..
    carrier.pulsesRemaining = carrier.burstCountMax;

    // set timer2 TOP value
    OCR2A = carrier.halfPeriod;
    // set switching value for the output pin
    OCR2B = carrier.dutyCompValue;
  }

  void updateTimer1Params() {
    // And set Input Capture Edge Select to Falling Edge.
    TCCR1B &= ~(1<<ICES1);
  }

  void enableIO() {
    // These's are all done in 1 body since they're slightly more timing critical.

    // reset timer 2 to 0.
    // This should ensure a well formed initial pulse
    // in the carirer waveform.
    TCNT2 = 0;

    // start switching on the emitter.
    TCCR2A |= (1<<COM2B1);
    // Enable Timer 2's overflow interrupt so we can start counting pulses.
    TIMSK2 |= (1<<TOIE2);

    // Enable Timer 1's input capture event interrupt to listen for
    // IR sensor changes.
    TIMSK1 |= (1<<ICIE1);
  }

  bool wasSignalAcceptable() {
    return (
      receiver.ticksElapsed > receiver.tickCountLowerBound
      && receiver.ticksElapsed < recover.tickCountUpperBound
    );
  }
} distanceSense = {
  .signalStrengthInput = 2,

  .carrier = {
    .f0 = 38e3,
    .duty = 0.0,

    .burstCountMax = 25,
    .burstCountLowerBound = 25 - 5,
    .burstCountUpperBound = 25 + 6,

    .pulsesRemaining = 0,

    .halfPeriod = 0,
    .duty = 0,
    .strength = 0
  },

  .receiver = {
    .tickCountLowerBound = 0,
    .tickCountUpperBound = 0,

    .ticksElapsed = 0
  }
};

ISR(TIMER1_CAPT_vect) {
  if (TCCR1B & (1<<ICES1) == 0) {
    // We were called on a falling edge.
    // The IR sensor picked up a signal!
    // Zero-out the timer counter,
    TCNT1 = 0;
    // then set edge detection to rising edge.
    TCCR1B &= ~(1<<ICES1);
  }
  else {
    // We were called on a rising edge.
    // The IR sensor has stopped receiving a signal!
    // Read the count from the Input Capture Register
    // rather than the Timer Counter Register TCNT1,
    // as this hopefully reduces the chance of corruption,
    // (although given we have a premultiplier of 64,
    // that's probably not a problem?)
    distanceSense.receiver.ticksElapsed = ICR1;
    // Disable input capture interrupts,
    TIMSK1 &= ~(1<<ICIE1);
    // And update the state machine's state.
    currentState = States::DistanceSenseEnded;
  }
}

ISR(TIMER2_OVF_vect) {
  distanceSense.carrier.pulsesRemaining -= 1;
  if (distanceSense.carrier.pulsesRemaining == 0) {
    // Once we've counted down...
    // Disable the overflow interrupt,
    TIMSK2 &= ~(1<<TOIE2);
    // ... and that's it!
  }
}


// Main
// --------------------------------

void setup() {
  setupDiodeDifferential();
  distanceSense.setup();

  // For now, just setting the LED...
  pinMode(13, OUTPUT);
}

void loop() {
  switch (currentState) {
    default:
    case States::LeftRightSense:
      updateDiodeDifferential();
      currentState = States::DistanceSenseBeginning;
      break;

    case States::DistanceSenseBeginning:
      distanceSense.startSensing();
      break;

    case States::DistanceSenseWaiting:
      // we just do nothing!
      break;

    case States::DistanceSenseEnded:
      // ... well, it works!
      digitalWrite(13, (
        distanceSense.wasSignalAcceptable()
          ? HIGH
          : LOW
      ));

      // if (distanceSense.wasSignalAcceptable()) {
      //   // ... TODO: do something!
      // }

      // Back to start!
      currentState = States::LeftRightSense;
      break;
  }
}
