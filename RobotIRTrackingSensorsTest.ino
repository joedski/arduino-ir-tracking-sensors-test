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

// A0 is used for the diodes
#define IN_SENSE 0
// A1 is used for the center voltage reference
#define IN_VREF 1

// Pin to light up more when the higher photodiode conducts more
#define OUT_HIGHER 5
// Pin to light up more when the lower photodiode conducts more
#define OUT_LOWER 6


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


// Main
// --------------------------------

void setup() {
  setupDiodeDifferential()
}

void loop() {
  updateDiodeDifferential()
}
