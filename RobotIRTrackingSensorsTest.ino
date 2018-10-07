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

// Vishay's Fast Proximity App Note seemed to indicate this was a good count,
// even when going off the center frequnecy.
const uint8_t BURST_COUNT = 25;
// Analog pin we read the strength setting from.
const uint8_t IN_DISTANCE_SIGNAL_STRENGTH = 2;
// Track the number of pulses for Timer 1 to tick for.
volatile uint8_t distanceSignalBurstCount = 0;
// Half period for our carrier.
// t2 = 16MHz / (2 * f0)
uint8_t distanceSignalHalfPeriod = 210;
// Duty cycle.
// = t2 * d%; so 30% duty is t2 * 0.3;
uint8_t distanceSignalDuty = 63;
int16_t distanceSignalStrength = 0;

// // These start and stop Arduino Pin 3 pulsing.
// #define stopPulsing() TCCR2A &= ~(1<<COM2B1)
// #define startPulsing() TCCR2A |= (1<<COM2B1)
// #define setPulseDuty(duty) OCR2B = duty

void setupDistanceSense() {
  // First, setting up Timer 1
  // -------------------------

  // This is basically copied verbatim from the Slow Proximity note.
  // Granted, it does exactly what I want, and there's
  // only so many ways to twiddle bits.

  // Mode5: PWM phase correct with TOP=OCR2A
  // OC2A pin not used, OC2B disconnected for now
  TCCR2A = (1<<WGM20) | (0<<COM2B1);
  // FOC not used, no prescaler, final bit of Mode5 setting
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

  // We don't need to do much here other than
  // set a prescaler of 64.

  TCCR1B = (1<<ICNC1) | (0<<ICES1) | (1<<CS11) | (1<<CS10);

  // We'll set OCR1A and OCR1B to appropriate values later,
  // then enable TIMSK1.ICIE1, etc.
}

void beginDistanceSense() {
  // Step 1: Read our sensor.
  // We need just an 8-bit number, so just take off the 2 LSBs.
  distanceSignalStrength = analogRead(IN_DISTANCE_SIGNAL_STRENGTH) >> 2;

  // Step 2: Configure Timers.
  // TODO: Try out various ways to calculate strength.
  // For now, just varying the duty cycle from 0% ~ 70%:
  // NOTE: This might not work, it might not actually detect it until like 30%.
  // Or it might work well enough.  I'd need to dig through the data sheets again.
  distanceSignalDuty = lerp8by8(0, distanceSignalHalfPeriod, distanceSignalStrength);

  // Technically, by not changing the frequency of Timer 1,
  // we don't need to recalculate the lower and upper acceptable bounds
  // of Timer 2.
  // It's easier to modify in the future, though, if I leave it here.
  // TODO everything!
}


// Main
// --------------------------------

void setup() {
  setupDiodeDifferential();
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
