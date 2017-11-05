/*

   OpenEMR - Vital Signs
   Panth, Brennan, Omar, and Ian

*/

// include the library code:
#define USE_ARDUINO_INTERRUPTS false
#include <LiquidCrystal.h>
#include <PulseSensorPlayground.h>
#include <DHT11.h>

#define DHT_PIN 6

dht11 DHT11;
/** PIN IDs **/

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
/* Pins being used
    0, 1, 2, 3, 4, 5, _, 7, 8, _, 10, 11, 12, 13
*/
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
int ledPinID = 13;
int buttonID = 10;
int sensorPinID = 1;
int lightSensorID = 8;
int redLightID = 7;
int PulseSensorPurplePin = 0;        // Pulse Sensor PURPLE WIRE connected to ANALOG PIN 0
const int PIN_INPUT = A0;
const int PIN_BLINK = 13;    // Pin 13 is the on-board LED
const int THRESHOLD = 550;   // Adjust this number to avoid noise when idle

int Signal;                // holds the incoming raw data. Signal value can range from 0-1024
int Threshold = 550;            // Determine which Signal to "count as a beat", and which to ingore.


//alpha is used in the original proposed code by the company (see below).
double alpha = 0.75;
//lasttime is used to have a very precise measurement of the time so the calculated pulse is correct.
unsigned long lasttime = 0;

/** Primary  Variables **/

double _BPM;
double _RedLight;
double _IRLight;
double _temperature = 0;
double _SPO2;
double _BP;
double _severity;



/** State Controller **/
/*  0 = BPM
 *  1 = IR
 *  2 = Red
 *  3 = Temp
 */
int _state = 0;

/** Smoothing Variables **/
 const int numReadings = 15;

double _ir_readings[numReadings];      // the readings from the analog input
int _ir_readIndex = 0;              // the index of the current reading
double _ir_total = 0;                  // the running total
double _ir_average = 0;                // the average

double _re_readings[numReadings];      // the readings from the analog input
int _re_readIndex = 0;              // the index of the current reading
double _re_total = 0;                  // the running total
double _re_average = 0;                // the average

unsigned long _lastTempTime = 0;

bool buttonUsed = false;

PulseSensorPlayground pulseSensor;

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup() {
  Serial.begin(115200);

  // Button
  pinMode(buttonID, INPUT);

  //pulseSensor.analogInput(PIN_INPUT);
  //pulseSensor.blinkOnPulse(PIN_BLINK);
  //pulseSensor.fadeOnPulse(PIN_FADE);

  pulseSensor.setSerial(Serial);
  //pulseSensor.setOutputType(OUTPUT_TYPE);
  pulseSensor.setThreshold(THRESHOLD);

  // Now that everything is ready, start reading the PulseSensor signal.
  if (!pulseSensor.begin()) {
    /*
       PulseSensor initialization failed
    */
    for (;;) {
      // Flash the led to show things didn't work.
      digitalWrite(PIN_BLINK, LOW);
      delay(50);
      digitalWrite(PIN_BLINK, HIGH);
      delay(50);
    }
  }

  // Red Light
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);

  lasttime = micros();
  _lastTempTime = micros();

  DHT11.attach(DHT_PIN);

  // Clear smoothing variables
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    _ir_readings[thisReading] = 0;
  }
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    _re_readings[thisReading] = 0;
  }
}

void loop() {

    _BPM = 0;

  //lcd.print(digitalRead(buttonID));

  if (digitalRead(buttonID) == HIGH && buttonUsed == false) {
    buttonUsed = true;
    //_state = (_state + 1) % 4;
    Serial.print(_state);
    //lcd.print(1);
  } else if (digitalRead(buttonID) == HIGH && buttonUsed == true) {
    Serial.print(_state);
  } else {
    buttonUsed = false;
    //lcd.print(0);
    Serial.print(_state);
  }
  
  delay(20);

  /** BPM BPM BPM **/
  pulseSensor.sawNewSample();

  //  Signal = analogRead(PulseSensorPurplePin);  // Read the PulseSensor's value.
  //  Assign this value to the "Signal" variable.

  ////Serial.println(Signal);                    // Send the Signal value to Serial Plotter.

  lcd.clear();
  // BPM - Sample
  //lcd.print(pulseSensor.getLatestSample(0));
  //lcd.print(" B:");
  // BPM - Beats Per Minute
  //lcd.print(pulseSensor.getBeatsPerMinute(0));
  
  _BPM = pulseSensor.getBeatsPerMinute(0) / 2;

  // write the latest sample to Serial.
  pulseSensor.outputSample();

  /*
     If a beat has happened since we last checked,
     write the per-beat information to Serial.
  */
  if (pulseSensor.sawStartOfBeat()) {
    pulseSensor.outputBeat();
  }

  // IR 
  static double oldValue = 0;

  //Wait 10 ms between each measurement.
  while (micros() - lasttime < 10000)
  {
    delayMicroseconds(100);
  }

  //Read the signal.
  double rawValue = analogRead (sensorPinID);
  lasttime += 10000;

  //In the "original" code example, "value" was sent to the computer. This calculation basically smoothens "rawValue".
  //double value = alpha * oldValue + (1 - alpha) * rawValue;

  //Send back the measured value.
  Serial.println (rawValue);
  //oldValue = value;
  
  //lcd.print(" I:");
  //lcd.print(rawValue);

  // subtract the last reading:
  _ir_total = _ir_total - _ir_readings[_ir_readIndex];
  // read from the sensor:
  _ir_readings[_ir_readIndex] = rawValue;
  // add the reading to the total:
  _ir_total = _ir_total + _ir_readings[_ir_readIndex];
  // advance to the next position in the array:
  _ir_readIndex = _ir_readIndex + 1;

  // if we're at the end of the array...
  if (_ir_readIndex >= numReadings) {
    // ...wrap around to the beginning:
    _ir_readIndex = 0;
  }

  // calculate the average:
  _ir_average = _ir_total / numReadings;

  _IRLight = _ir_average;
  
  // Red Light Sensor;
  //lcd.setCursor(0, 1);
  //lcd.print("R:");
  //lcd.print(analogRead(2));

  _RedLight = analogRead(2);

  /** Temperature **/
  
  //lcd.print(" T:");

  //Wait 10 ms between each measurement.
  /*
  if (micros() - _lastTempTime > 2000000)
  {
    switch (chk)
    {
      case 0: _temperature = DHT11.temperature; break;
      default: break;
    }
    _temperature = DHT11.temperature;

      /*
    if (DHT11.temperature > 0.0) {
      _Temperature = DHT11.temperature;
    }

    _lastTempTime = micros();

  }
*/

  if (micros() - _lastTempTime > 2500000)
  {
    _state = (_state + 1) % 5;
    _lastTempTime = micros();
  }

  //lcd.print(_temperature);

  /** SpO2 Equation **/

  float x = (float)_RedLight/_IRLight;

  _SPO2 = min(100, pow((0.30367*x),2)+0.1*x+1.0267);

  if (_SPO2 > 100) {
    _SPO2 = 100;
  }

  /** BP **/
  double y = 0.525*(_BPM)+78.5;
  
  _BP = y;

  /** Severity **/

  float s1 = abs((_temperature - 33.2)/((_temperature + 33.2) / 2)) * 0.1;
  double s2 = abs((_BPM - 80)/((_BPM + 80) / 2)) * 0.2;
  double s3 = abs((_BP - 110)/((_BP + 110) / 2)) * 0.2;
  double s4 = abs((_SPO2 - 1)/((_SPO2 + 1) / 2)) * 0.5; 

  double _s = s1+s2+s3+s4;
  _severity = _s;

  double s_1 = abs((_temperature - 33.2)/((_temperature + 33.2) / 2));
  double s_2 = abs((_BPM - 80)/((_BPM + 80) / 2));
  double s_3 = abs((_BP - 110)/((_BP + 110) / 2));
  double s_4 = abs((_SPO2 - 1)/((_SPO2 + 1) / 2));

  double ranks[] = {s_1, s_2, s_3, s_4};

  double low1,low2,high1,high2,middle1,middle2,lowest,highest;

  if (ranks[1] > ranks[0]) {
        low1 = ranks[0];
        high1 = ranks[1];
  } else { 
        low1 = ranks[1];
        high1 = ranks[0];
  }

  if (ranks[2] < ranks[3]) {
        low1 = ranks[2];
        high1 = ranks[3];
  } else { 
        low1 = ranks[3];
        high1 = ranks[2];
  }


  if (ranks[2] < ranks[3]) {
        low1 = ranks[2];
        high1 = ranks[3];
  } else { 
        low1 = ranks[3];
        high1 = ranks[2];
  }

    if (low1 < low2) {
        lowest = low1;
        middle1 = low2;
    } else {
        lowest = low2;
        middle1 = low1;
    }

    if (high1 > high2) {
        highest = high1;
        middle2 = high2;
    } else {
        highest = high2;
        middle2 = high1;
    }

    if (middle1 < middle2) {
      
    } else {
      float temp = middle2;
      middle2 = middle1;
      middle1 = temp;
    }

  /** OUTPUT **/
  switch(_state) {
    case 0: //  BPM
      lcd.print("Severity: ");
      lcd.print(_severity, 4);
      lcd.setCursor(0,1);
      if (lowest > 0.01) {
      if (lowest == s_1) lcd.print("Temp > ");
      if (lowest == s_2) lcd.print("HR > ");
      if (lowest == s_3) lcd.print("BP > ");
      if (lowest == s_4) lcd.print("SPO2 > ");
      }

      if (middle1 > 0.01) {
      if (middle1 == s_1) lcd.print("Temp > ");
      if (middle1 == s_2) lcd.print("HR > ");
      if (middle1 == s_3) lcd.print("BP > ");
      if (middle1 == s_4) lcd.print("SPO2 > ");
      }

      if (middle2 > 0.01) {
      if (middle2 == s_1) lcd.print("Temp > ");
      if (middle2 == s_2) lcd.print("HR > ");
      if (middle2 == s_3) lcd.print("BP > ");
      if (middle2 == s_4) lcd.print("SPO2 > ");
      }

      if (highest > 0.01) {
      if (highest == s_1) lcd.print("Temp");
      if (highest == s_2) lcd.print("HR");
      if (highest == s_3) lcd.print("BP");
      if (highest == s_4) lcd.print("SPO2");
      }
      
      break;
    case 1:
      lcd.print("BPM: ");
      lcd.print(_BPM);
      lcd.setCursor(0,1);
      lcd.print("Sample: ");
      lcd.print(pulseSensor.getLatestSample(0));
      break;
    case 2:
      lcd.print("SpO2: ");
      lcd.print(_SPO2);
      break;
    case 3:   //  Temp
      lcd.print("Temp (C):");
      lcd.print(_temperature);
      break;
    case 4:   //  Temp
      lcd.print("BP (Sys):  ");
      lcd.print(_BP);
      break;
    default:
      lcd.print("Error ");
      break;
  }

  /** DO LAST **/
  if (DHT11.temperature > 1) {
    _temperature = DHT11.temperature;
  }

  int chk = DHT11.read();
}

