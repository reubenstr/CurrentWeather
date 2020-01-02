#include <Arduino.h>
#include <AccelStepper.h>
#include <Adafruit_NeoPixel.h>

/*


// Calculate steps per inch.
(steps-pre-revolution * microsteps) /  (belt-pitch(inches) * pulley-teeth) = steps per inch
(200 * 16) / (0.0787402 * 36) = 1128.888

*/

#define PIN_LED_PRESSURE_POINTER 12
#define PIN_LED_GAUGES 11
#define PIN_LED_TEMPERATURE_POINTER 10
#define PIN_LED_TEXT 9
#define PIN_LED_HUMIDITY_POINTER 8
#define PIN_LED_CONNECTION_STATUS 13

#define PIN_DRIVER_1_STEP 3
#define PIN_DRIVER_1_DIR 2
#define PIN_DRIVER_2_STEP 5
#define PIN_DRIVER_2_DIR 4
#define PIN_DRIVER_3_STEP 7
#define PIN_DRIVER_3_DIR 6

#define PIN_LIMIT_TEMPERATURE A3
#define PIN_LIMIT_PRESSURE A2
#define PIN_LIMIT_HUMIDITY A1

#define PIN_SWITCH_MODE A0

#define STEPPER_STEPS_PER_INCH 1129
#define STEPPER_ACCELERATION 100000 // Instantious full speed.
#define STEPPER_MAX_SPEED 2000
#define STEPPER_SPEED 2000

#define STEPPER_TEMPERATURE_STEPS_FROM_LIMIT_AS_HOME 200
#define STEPPER_PRESSURE_STEPS_FROM_LIMIT_AS_HOME 200
#define STEPPER_HUMIDITY_STEPS_FROM_LIMIT_AS_HOME 200

#define SWITCH_LIMIT_ACTIVATED 0

#define LED_BRIGHTNESS 127



AccelStepper stepperTemperature(AccelStepper::DRIVER, PIN_DRIVER_3_STEP, PIN_DRIVER_3_DIR);
AccelStepper stepperPressure(AccelStepper::DRIVER, PIN_DRIVER_2_STEP, PIN_DRIVER_2_DIR);
AccelStepper stepperHumidity(AccelStepper::DRIVER, PIN_DRIVER_1_STEP, PIN_DRIVER_1_DIR);

Adafruit_NeoPixel stripTemperature = Adafruit_NeoPixel(1, PIN_LED_TEMPERATURE_POINTER, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel stripPressure = Adafruit_NeoPixel(1, PIN_LED_PRESSURE_POINTER, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel stripHumidity = Adafruit_NeoPixel(1, PIN_LED_HUMIDITY_POINTER, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel stripGauges = Adafruit_NeoPixel(80, PIN_LED_GAUGES, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel stripText = Adafruit_NeoPixel(43, PIN_LED_TEXT, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel stripConnectionStatus = Adafruit_NeoPixel(1, PIN_LED_CONNECTION_STATUS, NEO_GRB + NEO_KHZ800);

bool toggle;

bool switchPreviousState;

void updateCounters()
{
   
}


void setup()
{
  // Initialize limit switch and toggle switch pins.
  pinMode(PIN_LIMIT_TEMPERATURE, INPUT);
  digitalWrite(PIN_LIMIT_TEMPERATURE, HIGH); // turn on pullup resistors
  pinMode(PIN_LIMIT_PRESSURE, INPUT);
  digitalWrite(PIN_LIMIT_PRESSURE, HIGH); // turn on pullup resistors
  pinMode(PIN_LIMIT_HUMIDITY, INPUT);
  digitalWrite(PIN_LIMIT_HUMIDITY, HIGH); // turn on pullup resistors
  pinMode(PIN_SWITCH_MODE, INPUT);
  digitalWrite(PIN_SWITCH_MODE, HIGH); // turn on pullup resistors

  // Initialize steppers.
  stepperTemperature.setMaxSpeed(STEPPER_MAX_SPEED);
  stepperTemperature.setAcceleration(STEPPER_ACCELERATION);
  stepperPressure.setMaxSpeed(STEPPER_MAX_SPEED);
  stepperPressure.setAcceleration(STEPPER_ACCELERATION);
  stepperHumidity.setMaxSpeed(STEPPER_MAX_SPEED);
  stepperHumidity.setAcceleration(STEPPER_ACCELERATION);

  stepperTemperature.setSpeed(STEPPER_SPEED);
  stepperPressure.setSpeed(STEPPER_SPEED);
  stepperHumidity.setSpeed(STEPPER_SPEED);

  // Initialize WS2812b LED .
  stripTemperature.setBrightness(LED_BRIGHTNESS);
  stripTemperature.begin();
  stripTemperature.show();
  stripPressure.setBrightness(LED_BRIGHTNESS);
  stripPressure.begin();
  stripPressure.show();
  stripHumidity.setBrightness(LED_BRIGHTNESS);
  stripHumidity.begin();
  stripHumidity.show();
  stripGauges.setBrightness(LED_BRIGHTNESS);
  stripGauges.begin();
  stripGauges.show();
  stripText.setBrightness(LED_BRIGHTNESS);
  stripText.begin();
  stripText.show();
  stripConnectionStatus.setBrightness(LED_BRIGHTNESS);
  stripConnectionStatus.begin();
  stripConnectionStatus.show();

  toggle = false;

  LedStartupPattern();

  HomeSteppers();

  SetTextColors();


  switchPreviousState = digitalRead(PIN_SWITCH_MODE);

}

void loop()
{


  if (digitalRead(PIN_SWITCH_MODE) != switchPreviousState)
  {
      switchPreviousState = digitalRead(PIN_SWITCH_MODE);
      SetInsideOutsideLedColors();
      delay(50); // hard debounce delay as this will not effect timings (enough to have an impact)
  }

  SetConnectionStatusLED();

  /*
    if (stepperTemperature.distanceToGo() == 0)
    {
    	// Random change to speed, position and acceleration
    	// Make sure we dont get 0 speed or accelerations
    	delay(1000);
      
      if (toggle)
      {
        toggle = false;
        stepperTemperature.moveTo(0);
         stepperPressure.moveTo(0);
          stepperHumidity.moveTo(0);
       
      }
      else 
      {
        toggle = true;
        stepperTemperature.moveTo(STEPPER_STEPS_PER_INCH);
        stepperPressure.moveTo(STEPPER_STEPS_PER_INCH);
        stepperHumidity.moveTo(STEPPER_STEPS_PER_INCH);
       
      }    	
    }
    */

  if (digitalRead(PIN_LIMIT_HUMIDITY) == SWITCH_LIMIT_ACTIVATED)
  {
    stepperHumidity.setCurrentPosition(0);
    stepperHumidity.moveTo(0);
  }

  // Step motors if needed (does not use acceleration).
  stepperTemperature.run();
  stepperPressure.run();
  stepperHumidity.run();
}

// Home steppers into known position using limit switches.
void HomeSteppers()
{

  bool stepperTemperatureHomeFlag = false;
  bool stepperPressureHomeFlag = false;
  bool stepperHumidityHomeFlag = false;

  stepperTemperature.moveTo(-100000);
  stepperPressure.moveTo(-100000);
  stepperHumidity.moveTo(-100000);

  // Phase 1: all three steppers contact limit switches.
  while (1)
  {

    if (digitalRead(PIN_LIMIT_TEMPERATURE) == SWITCH_LIMIT_ACTIVATED)
    {
      stepperTemperatureHomeFlag = true;
    }

    if (digitalRead(PIN_LIMIT_PRESSURE) == SWITCH_LIMIT_ACTIVATED)
    {
      stepperPressureHomeFlag = true;
    }

    if (digitalRead(PIN_LIMIT_HUMIDITY) == SWITCH_LIMIT_ACTIVATED)
    {
      stepperHumidityHomeFlag = true;
    }

    if (stepperTemperatureHomeFlag == false)
    {
      stepperTemperature.run();
    }

    if (stepperPressureHomeFlag == false)
    {
      stepperPressure.run();
    }

    if (stepperHumidityHomeFlag == false)
    {
      stepperHumidity.run();
    }

    if (stepperTemperatureHomeFlag && stepperPressureHomeFlag && stepperHumidityHomeFlag)
    {
      break;
    }
  }

  stepperTemperature.setCurrentPosition(0);
  stepperPressure.setCurrentPosition(0);
  stepperHumidity.setCurrentPosition(0);

  stepperTemperature.moveTo(STEPPER_TEMPERATURE_STEPS_FROM_LIMIT_AS_HOME);
  stepperPressure.moveTo(STEPPER_PRESSURE_STEPS_FROM_LIMIT_AS_HOME);
  stepperHumidity.moveTo(STEPPER_HUMIDITY_STEPS_FROM_LIMIT_AS_HOME);

  // Phase 2: back away from limit switches
  while (1)
  {
    if (stepperTemperature.distanceToGo() != 0)
    {
      stepperTemperature.run();
    }
    if (stepperPressure.distanceToGo() != 0)
    {
      stepperPressure.run();
    }
    if (stepperHumidity.distanceToGo() != 0)
    {
      stepperHumidity.run();
    }

    if (stepperTemperature.distanceToGo() == 0 &&
        stepperPressure.distanceToGo() == 0 &&
        stepperHumidity.distanceToGo() == 0)
    {
      stepperTemperature.setCurrentPosition(0);
      stepperPressure.setCurrentPosition(0);
      stepperHumidity.setCurrentPosition(0);
      break;
    }
  }
}

void SetTextColors()
{

  // Text pattern LED order (43 total)
  // 3x humidity
  // 3x pressure
  // 3x tempurature
  // 12x cloud cover
  // 5x inside
  // 5x outside
  // 12x current weather

  // humidity
  for (int i = 0; i < 3; i++)
  {
    stripText.setPixelColor(i, stripText.Color(0, 0, 255));
  }

  // pressure
  for (int i = 3; i < 6; i++)
  {
    stripText.setPixelColor(i, stripText.Color(0,255, 0));
  }

  // tempurature
  for (int i = 6; i < 9; i++)
  {
    stripText.setPixelColor(i, stripText.Color(255, 0, 0));
  }

  // cloud cover
  for (int i = 9; i < 3 + 21; i++)
  {
    stripText.setPixelColor(i, stripText.Color(0, 0, 0));
  }

  SetInsideOutsideLedColors();
  
  // current weather
    for (int i = 31; i < 43; i++)
    {
      stripText.setPixelColor(i, stripText.Color(150, 0, 20));
    }

  stripText.show();
}


void SetInsideOutsideLedColors()
{
  if (digitalRead(PIN_SWITCH_MODE) == SWITCH_LIMIT_ACTIVATED)
  {
    // inside
    for (int i = 21; i < 26; i++)
    {
      stripText.setPixelColor(i, stripText.Color(0, 75, 127));
    }
    // outside
    for (int i = 26; i < 31; i++)
    {
      stripText.setPixelColor(i, stripText.Color(0, 0, 0));
    }
  }
  else
  {
    // outside
    for (int i = 21; i < 26; i++)
    {
      stripText.setPixelColor(i, stripText.Color(0, 0, 0));
    }
    for (int i = 26; i < 31; i++)
    {
      stripText.setPixelColor(i, stripText.Color(0, 75, 127));
    }
  }

  stripText.show();
}


void LedStartupPattern()
{
  for (int i = 0; i < 43; i++)
  {
    if (i < 20)
    {
      stripGauges.setPixelColor(i + 0, stripGauges.Color(0, 0, 255));
      stripGauges.setPixelColor(40 - i, stripGauges.Color(0, 255, 0));
      stripGauges.setPixelColor(i + 40, stripGauges.Color(255, 0, 0));
      stripGauges.show();
    }

    stripText.setPixelColor(i, stripText.Color(127, 127, 0));
    stripText.show();

    delay(25);
  }

  for (int i = 0; i < 20; i++)
  {

    stripGauges.setPixelColor(i + 0, Wheel(i * 256 / 20));
    stripGauges.setPixelColor(40 - i, Wheel(i * 256 / 20));
    stripGauges.setPixelColor(i + 40, Wheel(i * 256 / 20));
    stripGauges.show();
  }
}

void SetConnectionStatusLED()
{
  stripConnectionStatus.setPixelColor(0, stripConnectionStatus.Color(255, 0, 0));
  stripConnectionStatus.show();
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos)
{
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85)
  {
    return stripConnectionStatus.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170)
  {
    WheelPos -= 85;
    return stripConnectionStatus.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return stripConnectionStatus.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
