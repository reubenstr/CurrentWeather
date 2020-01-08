/*
	Current Weather

	Motorized temperature, pressure, and humidity gauges with
  WS2812s backlights mounted on a wall hanging frame.

  Reuben Strangelove
  Winter 2019/2020

  MCU: Arduino Nano (a faster MCU should be considered)
  WiFi: ESP8266 ESP01 Module
  Sensor: BME280 for inside temperature, pressure, and humidity.

  Outside weather provided by the ESP01 module connecting to openweathermap.org API.

  A toggle on the mode ("Weather Source") switch during homing halts
  motors after backing off the limit switches for easier position calibration.

*/

#define DEBUG_OUTPUT 1

#include <Arduino.h>
#include <Wire.h>
#include <AccelStepper.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

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

// Steps per inch = (steps-pre-revolution * microsteps) / (belt-pitch(units in inches) * pulley-teeth)
//(200 * 16) / (0.0787402 * 36) = 1128.888
long const STEPPER_ACCELERATION = 2500; // Instantious full speed.
long const STEPPER_MAX_SPEED = 20000;
long const STEPPER_SPEED = 4000;
float const STEPPER_STEPS_PER_INCH = 1128.888;

float const STEPPER_INCHES_PER_TEMPERATURE_UNIT = 0.107; // Distance between ticks on the gauge display
float const STEPPER_INCHES_PER_PRESSURE_UNIT = 0.130;    // Distance between ticks on the gauge display
float const STEPPER_INCHES_PER_HUMITIY_UNIT = 0.130;     // Distance between ticks on the gauge display

// TODO Unique offsets
float const STEPPER_MAX_POSITION = STEPPER_STEPS_PER_INCH * 13.161 - STEPPER_INCHES_PER_PRESSURE_UNIT;

// Distance from limit switch to zero position on the weather data scale (manualy calibrated through observation).
const int STEPPER_TEMPERATURE_STEPS_FROM_LIMIT_AS_HOME = 321;
const int STEPPER_PRESSURE_STEPS_FROM_LIMIT_AS_HOME = 118;
const int STEPPER_HUMIDITY_STEPS_FROM_LIMIT_AS_HOME = 138;

const int SWITCH_LIMIT_ACTIVATED = 0; // Switch toggles high or low (1 or 0)

const int LED_BRIGHTNESS = 127;
const int LED_POINTER_BRIGHTNESS = 255;
const int LED_NUM_LEDS_PER_GAUGE = 20;

bool mode;
const bool MODE_INSIDE = true;
const bool MODE_OUTSIDE = false;

const int UART_FLUSH_TIMEOUT = 250;         // ms
const int UART_DISCONNECTED_TIMEOUT = 5000; // ms
const int UART_EXPECTED_BYTES_PER_DATA_SET = 10;

byte uartReadData[10];
int uartReadCount = 0;
int uartTimeoutCount = 0;
unsigned int lastUartReadMillis = 0;
bool uartActive = false;

struct Weather
{
  int weatherId;
  int temperature;
  int humidity;
  int pressure;
  int sum;
};

Weather weather;
unsigned int const insideWeatherDataUpdateRate = 25; // ms, effects stepper smoothness.

float const temperatureInsideCalibrationOffset = -6.0;
float const pressureInsideCalibrationOffset = 0;
float const humidityInsideCalibrationOffset = 7.0;

AccelStepper stepperTemperature(AccelStepper::DRIVER, PIN_DRIVER_3_STEP, PIN_DRIVER_3_DIR);
AccelStepper stepperPressure(AccelStepper::DRIVER, PIN_DRIVER_2_STEP, PIN_DRIVER_2_DIR);
AccelStepper stepperHumidity(AccelStepper::DRIVER, PIN_DRIVER_1_STEP, PIN_DRIVER_1_DIR);

Adafruit_NeoPixel stripTemperature = Adafruit_NeoPixel(1, PIN_LED_TEMPERATURE_POINTER, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel stripPressure = Adafruit_NeoPixel(1, PIN_LED_PRESSURE_POINTER, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel stripHumidity = Adafruit_NeoPixel(1, PIN_LED_HUMIDITY_POINTER, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel stripGauges = Adafruit_NeoPixel(80, PIN_LED_GAUGES, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel stripText = Adafruit_NeoPixel(43, PIN_LED_TEXT, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel stripConnectionStatus = Adafruit_NeoPixel(1, PIN_LED_CONNECTION_STATUS, NEO_GRB + NEO_KHZ800);

Adafruit_BME280 bme;
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();
bool bmeStatus;

void setup()
{
  Serial.begin(115200);

  // Initialize limit switch and toggle switch pins.
  pinMode(PIN_LIMIT_TEMPERATURE, INPUT);
  pinMode(PIN_LIMIT_PRESSURE, INPUT);
  pinMode(PIN_LIMIT_HUMIDITY, INPUT);
  pinMode(PIN_SWITCH_MODE, INPUT);
  digitalWrite(PIN_LIMIT_TEMPERATURE, HIGH); // turn on pullup resistors
  digitalWrite(PIN_LIMIT_PRESSURE, HIGH);    // turn on pullup resistors
  digitalWrite(PIN_LIMIT_HUMIDITY, HIGH);    // turn on pullup resistors
  digitalWrite(PIN_SWITCH_MODE, HIGH);       // turn on pullup resistors

  // Initialize stepper motors.
  stepperTemperature.setAcceleration(STEPPER_ACCELERATION);
  stepperPressure.setAcceleration(STEPPER_ACCELERATION);
  stepperHumidity.setAcceleration(STEPPER_ACCELERATION);
  stepperTemperature.setMaxSpeed(STEPPER_MAX_SPEED);
  stepperPressure.setMaxSpeed(STEPPER_MAX_SPEED);
  stepperHumidity.setMaxSpeed(STEPPER_MAX_SPEED);
  stepperTemperature.setSpeed(STEPPER_SPEED);
  stepperPressure.setSpeed(STEPPER_SPEED);
  stepperHumidity.setSpeed(STEPPER_SPEED);

  // Initialize WS2812b LED strips.
  stripTemperature.setBrightness(LED_POINTER_BRIGHTNESS);
  stripTemperature.begin();
  stripTemperature.show();
  stripPressure.setBrightness(LED_POINTER_BRIGHTNESS);
  stripPressure.begin();
  stripPressure.show();
  stripHumidity.setBrightness(LED_POINTER_BRIGHTNESS);
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

  // Initialize BME280 sensor.
  bmeStatus = bme.begin(BME280_ADDRESS_ALTERNATE);
  //bme.setSampling(bme.MODE_NORMAL, bme.SAMPLING_X4, bme.SAMPLING_X4, bme.SAMPLING_X4, bme.FILTER_OFF, bme.STANDBY_MS_250);
  

  // Initialize LEDs and motor positions.
  LedStartupPattern();
  HomeSteppers();
  SetTextColors();
  SetGaugeColors();
}

// Main execution loop.
void loop()
{
  CheckModeSwitch();
  CheckUartForData();
  UpdateStatusLED();
  UpdatePointerPositions();
  UpdatePointerColors();
  CheckLimitSwitches();
  UpdateStepperMotors();
  UpdateCloudCoverAndPrecipitationLEDs();
}

// Step motors (if needed) (does not use acceleration).
void UpdateStepperMotors()
{
  stepperTemperature.run();
  stepperPressure.run();
  stepperHumidity.run();
}

// Update LEDs and steppers with weather information.
void UpdatePointerPositions()
{
  static unsigned long oldMillis = 0;
  float temperatureData;
  float pressureData;
  float humidityData;

  // Select weather data source: inside or outside (sensor or ESP via UART)
  if (mode == MODE_INSIDE)
  {
    // Add delay due to execution overhead to prevent stepper motor slow down.
    if (millis() > oldMillis + insideWeatherDataUpdateRate)
    {
      oldMillis = millis();

      // Acquire data from BME280 sensor (per Adafruit example sketch).
      sensors_event_t temp_event, pressure_event, humidity_event;
      bme_temp->getEvent(&temp_event);
      bme_pressure->getEvent(&pressure_event);
      bme_humidity->getEvent(&humidity_event);

      temperatureData = round((temp_event.temperature * 9.0 / 5.0) + 32.0 + temperatureInsideCalibrationOffset); // Convert C to F
      pressureData = round(pressure_event.pressure + pressureInsideCalibrationOffset);
      humidityData = round(humidity_event.relative_humidity + humidityInsideCalibrationOffset);
    }
  }
  else if (mode == MODE_OUTSIDE)
  {
    // Data from ESP (via UART).
    temperatureData = weather.temperature;
    pressureData = weather.pressure;
    humidityData = weather.humidity;

    temperatureData = 10; // TEMP
    pressureData = 960;   // TEMP
    humidityData = 10;    // TEMP
  }

  // Convert temperature to position (add 10 degrees offset to scale temperature out of negative values).
  float positionTemperature = (temperatureData + 10) * STEPPER_STEPS_PER_INCH * STEPPER_INCHES_PER_TEMPERATURE_UNIT;
  if (positionTemperature > STEPPER_MAX_POSITION)
  {
    positionTemperature = STEPPER_MAX_POSITION;
  }
  stepperTemperature.moveTo((long)positionTemperature);

  // Convert pressure to position (subtrace 950 units to scale into gauge and stepper space).
  float positionPressure = (pressureData - 950) * STEPPER_STEPS_PER_INCH * STEPPER_INCHES_PER_PRESSURE_UNIT;
  if (positionPressure > STEPPER_MAX_POSITION)
  {
    positionPressure = STEPPER_MAX_POSITION;
  }
  stepperPressure.moveTo((long)positionPressure);

  // Convert humidity to position.
  float positionHumdity = (humidityData)*STEPPER_STEPS_PER_INCH * STEPPER_INCHES_PER_HUMITIY_UNIT;
  if (positionHumdity > STEPPER_MAX_POSITION)
  {
    positionHumdity = STEPPER_MAX_POSITION;
  }
  stepperHumidity.moveTo((long)positionHumdity);

  if (DEBUG_OUTPUT)
  {
    static unsigned int oldMillisDebug;
    if (millis() > oldMillisDebug + 250)
    {
      oldMillisDebug = millis();
      Serial.println((String) "T: " + temperatureData);
      Serial.println((String) "P: " + pressureData);
      Serial.println((String) "H: " + humidityData);
    }
  }
}

// Update pointers colors.
void UpdatePointerColors()
{
  int startColorPosition = 100;
  int endColorPosition = 170;

  // Scale position to desired color wheel colors.
  float positionPercentageTemperature = stepperTemperature.currentPosition() / STEPPER_MAX_POSITION * 100;
  int mappedPositionToColorWheelTemperature = map(positionPercentageTemperature, 0, startColorPosition, endColorPosition, 0);
  stripTemperature.setPixelColor(0, Wheel(mappedPositionToColorWheelTemperature));

  float positionPercentagePressure = stepperPressure.currentPosition() / STEPPER_MAX_POSITION * 100;
  int mappedPositionToColorWheelPressure = map(positionPercentagePressure, 0, startColorPosition, endColorPosition, 0);
  stripPressure.setPixelColor(0, Wheel(mappedPositionToColorWheelPressure));

  float positionPercentageHumidity = stepperHumidity.currentPosition() / STEPPER_MAX_POSITION * 100;
  int mappedPositionToColorWheelHumidity = map(positionPercentageHumidity, 0, startColorPosition, endColorPosition, 0);
  stripHumidity.setPixelColor(0, Wheel(mappedPositionToColorWheelHumidity));

  stripTemperature.show();
  stripPressure.show();
  stripHumidity.show();
}

// Check for UART data.
// Manual timeout for fragment flushing as data is expected every 2000ms.
void CheckUartForData()
{
  static unsigned long oldMillis = 0;

  // Check packet timeout.
  if (millis() > (oldMillis + UART_FLUSH_TIMEOUT))
  {
    oldMillis = millis();
    uartReadCount = 0;
  }

  // Check for UART timeout.
  if (millis() > (lastUartReadMillis + UART_DISCONNECTED_TIMEOUT))
  {
    uartActive = false;
  }

  // Check for available data.
  if (Serial.available())
  {
    // Reset timeout.
    oldMillis = millis();
    lastUartReadMillis = millis();

    uartReadData[uartReadCount] = Serial.read();

    if (DEBUG_OUTPUT)
    {
      Serial.print(String(uartReadData[uartReadCount]) + ", ");
    }

    uartReadCount++;

    // Process uart data.
    if (uartReadCount == UART_EXPECTED_BYTES_PER_DATA_SET)
    {
      uartReadCount = 0;

      Weather weatherTemp;
      weatherTemp.weatherId = uartReadData[0] + (uartReadData[1] << 8);
      weatherTemp.temperature = uartReadData[2] + (uartReadData[3] << 8);
      weatherTemp.humidity = uartReadData[4] + (uartReadData[5] << 8);
      weatherTemp.pressure = uartReadData[6] + (uartReadData[7] << 8);
      weatherTemp.sum = uartReadData[8] + (uartReadData[9] << 8);

      // Check checksum.
      if (weatherTemp.sum != weatherTemp.weatherId + weatherTemp.temperature + weatherTemp.humidity + weatherTemp.pressure)
      {
        return;
      }

      uartActive = true;

      weather = weatherTemp;
    }
  }
}

// Check if the mode switch has toggled.
void CheckModeSwitch()
{
  if (digitalRead(PIN_SWITCH_MODE) != mode)
  {
    mode = digitalRead(PIN_SWITCH_MODE);
    SetInsideOutsideLedColors();
  }
}

// Check if any of the steppers triggered a limit switch.
void CheckLimitSwitches()
{
  // TODO: check if limits are repeatively triggered as that indicates a mechanical failure.
  if (digitalRead(PIN_LIMIT_TEMPERATURE) == SWITCH_LIMIT_ACTIVATED ||
      digitalRead(PIN_LIMIT_PRESSURE) == SWITCH_LIMIT_ACTIVATED ||
      digitalRead(PIN_LIMIT_HUMIDITY) == SWITCH_LIMIT_ACTIVATED)
  {
    HomeSteppers();
  }
}

void UpdateCloudCoverAndPrecipitationLEDs()
{

static unsigned long oldMillis;

//if (millis() > oldMillis + 100)

if (mode == MODE_INSIDE)
{
 for (int i = 9; i < 3 + 21; i++)
  {
    stripText.setPixelColor(i, stripText.Color(0, 0, 0));
  }
}
else
{
  if (weather.weatherId == 




}

stripText.show();

}


// Home steppers into known position using limit switches.
// Phase 1: all three steppers contact limit switches.
// Phase 2: all three steppers back away from limit switches into calibrated positions.
void HomeSteppers()
{
  bool stepperTemperatureHomeFlag = false;
  bool stepperPressureHomeFlag = false;
  bool stepperHumidityHomeFlag = false;

  // Negative speed determines direction.
  stepperTemperature.setSpeed(-STEPPER_SPEED);
  stepperPressure.setSpeed(-STEPPER_SPEED);
  stepperHumidity.setSpeed(-STEPPER_SPEED);

  bool haltForCalibrationFlag = false;
  int oldSwitchState = digitalRead(PIN_SWITCH_MODE);

  // Phase 1
  while (!stepperTemperatureHomeFlag || !stepperPressureHomeFlag || !stepperHumidityHomeFlag)
  {
    if (oldSwitchState != digitalRead(PIN_SWITCH_MODE))
    {
      haltForCalibrationFlag = true;
    }

    if (digitalRead(PIN_LIMIT_TEMPERATURE) != SWITCH_LIMIT_ACTIVATED)
    {
      stepperTemperature.runSpeed();
    }
    else
    {
      stepperTemperatureHomeFlag = true;
    }

    if (digitalRead(PIN_LIMIT_PRESSURE) != SWITCH_LIMIT_ACTIVATED)
    {
      stepperPressure.runSpeed();
    }
    else
    {
      stepperPressureHomeFlag = true;
    }

    if (digitalRead(PIN_LIMIT_HUMIDITY) != SWITCH_LIMIT_ACTIVATED)
    {
      stepperHumidity.runSpeed();
    }
    else
    {
      stepperHumidityHomeFlag = true;
    }
  }

  stepperTemperature.setCurrentPosition(0);
  stepperPressure.setCurrentPosition(0);
  stepperHumidity.setCurrentPosition(0);

  stepperTemperature.moveTo(STEPPER_TEMPERATURE_STEPS_FROM_LIMIT_AS_HOME);
  stepperPressure.moveTo(STEPPER_PRESSURE_STEPS_FROM_LIMIT_AS_HOME);
  stepperHumidity.moveTo(STEPPER_HUMIDITY_STEPS_FROM_LIMIT_AS_HOME);

  // Phase 2
  while (stepperTemperature.distanceToGo() != 0 ||
         stepperPressure.distanceToGo() != 0 ||
         stepperHumidity.distanceToGo() != 0)
  {
    stepperTemperature.run();
    stepperPressure.run();
    stepperHumidity.run();
  }

  stepperTemperature.setCurrentPosition(0);
  stepperPressure.setCurrentPosition(0);
  stepperHumidity.setCurrentPosition(0);

  if (haltForCalibrationFlag)
  {
    while (1)
      ; // Halt program for home position calibration.
  }
}

void UpdateStatusLED()
{
  if (mode == MODE_INSIDE)
  {
    if (bmeStatus)
    {
      stripConnectionStatus.setPixelColor(0, stripConnectionStatus.Color(0, 255, 0));
    }
    else
    {
      stripConnectionStatus.setPixelColor(0, stripConnectionStatus.Color(255, 0, 0));
    }
  }
  else if (mode == MODE_OUTSIDE)
  {
    if (uartActive)
    {
      stripConnectionStatus.setPixelColor(0, stripConnectionStatus.Color(0, 255, 0));
    }
    else
    {
      stripConnectionStatus.setPixelColor(0, stripConnectionStatus.Color(255, 0, 0));
    }
  }

  stripConnectionStatus.show();
}

void SetTextColors()
{
  // Text pattern LED order (43 total)
  // 3x humidity
  // 3x pressure
  // 3x temperature
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
    stripText.setPixelColor(i, stripText.Color(0, 255, 0));
  }

  // temperature
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

void FatalError()
{
  int brightness = 50;

  for (uint16_t  i = 0; i < stripGauges.numPixels(); i++)
  {
    stripGauges.setPixelColor(i, stripGauges.Color(brightness, 0, 0));
    stripGauges.show();
  }

  for (uint16_t  i = 0; i < stripText.numPixels(); i++)
  {
    stripText.setPixelColor(i, stripGauges.Color(brightness, 0, 0));
    stripText.show();
  }

  stripTemperature.setPixelColor(0, stripGauges.Color(brightness, 0, 0));
  stripText.show();
  stripPressure.setPixelColor(0, stripGauges.Color(brightness, 0, 0));
  stripText.show();
  stripHumidity.setPixelColor(0, stripGauges.Color(brightness, 0, 0));
  stripText.show();

  while (1)
    ; // Halt program.
}

void SetInsideOutsideLedColors()
{
  if (mode == MODE_INSIDE)
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

void SetGaugeColors()
{
  for (int i = 0; i < LED_NUM_LEDS_PER_GAUGE; i++)
  {
    int mappedPositionToColorWheel = map(i, 0, LED_NUM_LEDS_PER_GAUGE, 170, 0);
    stripTemperature.setPixelColor(0, Wheel(mappedPositionToColorWheel));

    stripGauges.setPixelColor(i + 0, Wheel(mappedPositionToColorWheel));
    stripGauges.setPixelColor(40 - i, Wheel(mappedPositionToColorWheel));
    stripGauges.setPixelColor(i + 40, Wheel(mappedPositionToColorWheel));
    stripGauges.show();
  }
}

// Dynamic startup pattern for visual effect.
void LedStartupPattern()
{
  for (int i = 0; i < 43; i++)
  {
    if (i < LED_NUM_LEDS_PER_GAUGE)
    {
      stripGauges.setPixelColor(i + 0, Wheel(256 / 43 * i));
      stripGauges.setPixelColor(LED_NUM_LEDS_PER_GAUGE * 2 - i, Wheel(256 / 43 * i));
      stripGauges.setPixelColor(i + LED_NUM_LEDS_PER_GAUGE * 2, Wheel(256 / 43 * i));
      stripGauges.show();
    }

    stripText.setPixelColor(i, Wheel(255 / 43 * i));
    stripText.show();

    delay(25);
  }
}

// Input a value 0 to 255 to get a color value (of a pseudo-rainbow).
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