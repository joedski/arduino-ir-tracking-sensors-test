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
  uint8_t signalStrengthInput;

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
    volatile uint8_t pulsesCounted;
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

    // We'll set OCR2A and OCR2B to appropriate values later,
    // since they're determined at run time.
    // // set timer2 TOP value
    // OCR2A = HALF_PERIOD;
    // // set switching value for the output pin
    // OCR2B = DUTY_LOW;

    // set arduino D3, AVR PD3 as output pin
    // it can be hard-disabled as an output by setting it as an input pin,
    // but I think twiddling COM2B1 is fine.
    DDRD |= (1<<DDD3);

    // Next, setting up Timer 2
    // ------------------------

    // Here, we do the following:
    // Enable Input Capture Noise Cancelation (TCCR1B.ICNC1)
    // Set the Prescaler to 64 (TCCR1B.CS11 & TCCR1B.CS10)
    // And, not necessary, but set the Input Capture Edge Select to Falling (TCCR1B.ICES1)
    TCCR1B = (1<<ICNC1) | (0<<ICES1) | (1<<CS11) | (1<<CS10);

    // We'll set OCR1A and OCR1B to appropriate values later,
    // then enable TIMSK1.ICIE1, etc.
  }
} distanceSense = {
  .signalStrengthInput = 2,

  .carrier = {
    .f0 = 38e3,
    .duty = 0.0,

    .burstCountMax = 25,
    .burstCountLowerBound = 25 - 5,
    .burstCountUpperBound = 25 + 6,

    .pulsesCounted = 0,
    .halfPeriod = 0,
    .duty = 0,
    .strength = 0
  },

  .receiver = {
    .tickCountMin = 0,
    .tickCountMax = 0
  }
};

// // Vishay's Fast Proximity App Note seemed to indicate this was a good count,
// // even when going off the center frequnecy.
// const uint8_t CARRIER_BURST_COUNT = 25;
// // How many carrier modulation pulses long
// // is the lower bound for an acceptable signal
// const uint8_t CARRIER_BURST_COUNT_LOWER = CARRIER_BURST_COUNT - 5;
// // How many carrier modulation pulses long
// // is the upper bound for an acceptable signal
// const uint8_t CARRIER_BURST_COUNT_UPPER = CARRIER_BURST_COUNT + 6;
// // Analog pin we read the strength setting from.
// const uint8_t IN_DISTANCE_SIGNAL_STRENGTH = 2;
// // Track the number of pulses for Timer 1 to tick for.
// volatile uint8_t distanceCarrierBurstCount = 0;
// // Half period for our carrier.
// // t2 = 16MHz / (2 * f0)
// uint8_t distanceCarrierHalfPeriod = 210;
// // Duty cycle.
// // = t2 * d%; so 30% duty is t2 * 0.3;
// uint8_t distanceCarrierDuty = 63;
// int16_t distanceCarrierStrength = 0;
//
// // These two values define the range of acceptable values
// // for Timer 1.
// // Minimum acceptable count on Timer 1
// uint16_t distanceReceiverMinCount = 0;
// // Maximum acceptable count on Timer 1
// uint16_t distanceReceiverMaxCount = 0;

// // These start and stop Arduino Pin 3 pulsing.
// #define stopPulsing() TCCR2A &= ~(1<<COM2B1)
// #define startPulsing() TCCR2A |= (1<<COM2B1)
// #define setPulseDuty(duty) OCR2B = duty

// void beginDistanceSense() {
//   // Step 1: Read our sensor.
//   // We need just an 8-bit number, so just take off the 2 LSBs.
//   distanceSignalStrength = analogRead(IN_DISTANCE_SIGNAL_STRENGTH) >> 2;
//
//   // Step 2: Configure Timers.
//   // TODO: Try out various ways to calculate strength.
//   // For now, just varying the duty cycle from 0% ~ 70%:
//   // NOTE: This might not work, it might not actually detect it until like 30%.
//   // Or it might work well enough.  I'd need to dig through the data sheets again.
//   distanceSignalDuty = lerp8by8(0, distanceSignalHalfPeriod, distanceSignalStrength);
//
//   // Technically, by not changing the frequency of Timer 2,
//   // we don't need to recalculate the lower and upper acceptable bounds
//   // of Timer 1.
//   // It's easier to modify in the future, though, if I leave it here.
//   // TODO everything!
// }


// Main
// --------------------------------

void setup() {
  setupDiodeDifferential();
  distanceSense.setup();
}

void loop() {
  switch (currentState) {
    default:
    case States::LeftRightSense:
      updateDiodeDifferential();
      currentState = States::DistanceSenseBeginning;
      break;

    case States::DistanceSenseBeginning:
      break;

    case States::DistanceSenseWaiting:
      // we just do nothing!
      break;

    case States::DistanceSenseEnded:
      break;
  }
}
