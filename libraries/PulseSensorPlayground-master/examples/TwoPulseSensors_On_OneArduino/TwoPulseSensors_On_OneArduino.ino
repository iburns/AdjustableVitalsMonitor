/*
   Arduino Sketch to detect pulses from two PulseSensors.

   See https://www.pulsesensor.com to get started.

   Copyright World Famous Electronics LLC - see LICENSE
   Contributors:
     Joel Murphy, https://pulsesensor.com
     Yury Gitman, https://pulsesensor.com
     Bradford Needham, @bneedhamia, https://bluepapertech.com

   Licensed under the MIT License, a copy of which
   should have been included with this software.

   This software is not intended for medical use.
*/

/*
   Every Sketch that uses the PulseSensor Playground must
   define USE_ARDUINO_INTERRUPTS before including PulseSensorPlayground.h.
   Here, #define USE_ARDUINO_INTERRUPTS false tells the library to
   not use interrupts to read data from the PulseSensor.

   If you want to use interrupts, simply change the line below
   to read:
     #define USE_ARDUINO_INTERRUPTS true

   Set US_PS_INTERRUPTS to false if either
   1) Your Arduino platform's interrupts aren't yet supported
   by PulseSensor Playground, or
   2) You don't wish to use interrupts because of the side effects.

   NOTE: if US_PS_INTERRUPTS is false, your Sketch must
   call pulse.sawNewSample() at least once every 2 milliseconds
   to accurately read the PulseSensor signal.
*/
#define USE_ARDUINO_INTERRUPTS false
#include <PulseSensorPlayground.h>


/*
   The format of our output.

   Set this to PROCESSING_VISUALIZER if you're going to run
    the multi-sensor Processing Visualizer Sketch.
    See https://github.com/WorldFamousElectronics/PulseSensorAmped_2_Sensors

   Set this to SERIAL_PLOTTER if you're going to run
    the Arduino IDE's Serial Plotter.
*/
const int OUTPUT_TYPE = SERIAL_PLOTTER;

/*
   Number of PulseSensor devices we're reading from.
*/
const int PULSE_SENSOR_COUNT = 2;

/*
     PIN_POWER0 = the output pin that the red (power)
      pin of the first PulseSensor will be connected to. If you don't
      want to use pins to power the PulseSensors, you can remove
      the code dealing with PIN_POWER0 and PIN_POWER1.
     PIN_INPUT0 = Analog Input. Connected to the pulse sensor
      purple (signal) wire.
     PIN_BLINK0 = digital Output. Connected to an LED (and 220 ohm resistor)
      that will flash on each detected pulse.
     PIN_FADE0 = digital Output. PWM pin onnected to an LED (and resistor)
      that will smoothly fade with each pulse.

     PIN_POWER1, PIN_INPUT1, etc. = the corresponding pins for
      the second PulseSensor.

     NOTE: PIN_FADE0 and PIN_FADE1 must be pins that support PWM.
       If USE_INTERRUPTS is true, Do not use pin 9 or 10 for PIN_FADE0
       or PIN_FADE1, because those pins' PWM interferes with the sample timer.
*/
const int PIN_POWER0 = 7;
const int PIN_INPUT0 = A0;
const int PIN_BLINK0 = 13;    // Pin 13 is the on-board LED
const int PIN_FADE0 = 5;

const int PIN_POWER1 = 8;
const int PIN_INPUT1 = A1;
const int PIN_BLINK1 = 12;
const int PIN_FADE1 = 11;

const int THRESHOLD = 550;   // Adjust this number to avoid noise when idle

/*
   samplesUntilReport = the number of samples remaining to read
   until we want to report a sample over the serial connection.

   We want to report a sample value over the serial port
   only once every 20 milliseconds (10 samples) to avoid
   doing Serial output faster than the Arduino can send.
*/
byte samplesUntilReport;
const byte SAMPLES_PER_SERIAL_SAMPLE = 10;

/*
   All the PulseSensor Playground functions.
   We tell it how many PulseSensors we're using.
*/
PulseSensorPlayground pulseSensor(PULSE_SENSOR_COUNT);

void setup() {
  /*
     Use 250000 baud because that's what the Processing Sketch expects to read,
     and because that speed provides about 25 bytes per millisecond,
     or 50 characters per PulseSensor sample period of 2 milliseconds.

     If we used a slower baud rate, we'd likely write bytes faster than
     they can be transmitted, which would mess up the timing
     of readSensor() calls, which would make the pulse measurement
     not work properly.
  */
  Serial.begin(250000);

  /*
     Set the PulseSensor power pins.
     That is, turn on the PulseSensors.
  */
  pinMode(PIN_POWER0, OUTPUT);
  digitalWrite(PIN_POWER0, HIGH);

  pinMode(PIN_POWER1, OUTPUT);
  digitalWrite(PIN_POWER1, HIGH);

  /*
     Configure the PulseSensor manager,
     telling it which PulseSensor (0 or 1)
     we're configuring.
  */

  pulseSensor.analogInput(PIN_INPUT0, 0);
  pulseSensor.blinkOnPulse(PIN_BLINK0, 0);
  pulseSensor.fadeOnPulse(PIN_FADE0, 0);

  pulseSensor.analogInput(PIN_INPUT1, 1);
  pulseSensor.blinkOnPulse(PIN_BLINK1, 1);
  pulseSensor.fadeOnPulse(PIN_FADE1, 1);

  pulseSensor.setSerial(Serial);
  pulseSensor.setOutputType(OUTPUT_TYPE);
  pulseSensor.setThreshold(THRESHOLD);

  // Skip the first SAMPLES_PER_SERIAL_SAMPLE in the loop().
  samplesUntilReport = SAMPLES_PER_SERIAL_SAMPLE;

  // Now that everything is ready, start reading the PulseSensor signal.
  if (!pulseSensor.begin()) {
    /*
       PulseSensor initialization failed,
       likely because our Arduino platform interrupts
       aren't supported yet.

       If your Sketch hangs here, try changing USE_ARDUINO_INTERRUPTS to false.
    */
    for (;;) {
      // Flash the led to show things didn't work.
      digitalWrite(PIN_BLINK0, LOW);
      delay(50);
      digitalWrite(PIN_BLINK0, HIGH);
      delay(50);
    }
  }
}

void loop() {

  if (pulseSensor.sawNewSample()) {

    if (--samplesUntilReport == (byte) 0) {
      samplesUntilReport = SAMPLES_PER_SERIAL_SAMPLE;

      pulseSensor.outputSample();

      /*
         If a beat has happened on a given PulseSensor
         since we last checked, write the per-beat information
         about that PulseSensor to Serial.
      */

      for (int i = 0; i < PULSE_SENSOR_COUNT; ++i) {
        if (pulseSensor.sawStartOfBeat(i)) {
          pulseSensor.outputBeat(i);
        }
      }
    }

    /*******
      Here is a good place to add code that could take up
      to a millisecond or so to run.
    *******/
  }

  /******
     Don't add code here, because it could slow the sampling
     from the PulseSensor.
  ******/
}
