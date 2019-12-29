#include <AccelStepper.h>
#include <Adafruit_NeoPixel.h>

#define PIN_LED_TEMPERATURE 12
#define PIN_LED_PRESSURE 11
#define PIN_LED_HUMIDITY 10
#define PIN_LED_MARKINGS 9
#define PIN_LED_TEXT 8
#define PIN_LED_CONNECTION 13

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

AccelStepper stepperTemperature(AccelStepper::DRIVER, PIN_DRIVER_1_STEP, PIN_DRIVER_1_DIR);
AccelStepper stepperPressure(AccelStepper::DRIVER, PIN_DRIVER_2_STEP, PIN_DRIVER_2_DIR);
AccelStepper stepperHumidity(AccelStepper::DRIVER, PIN_DRIVER_3_STEP, PIN_DRIVER_3_DIR);

Adafruit_NeoPixel stripTemperature = Adafruit_NeoPixel(1, PIN_LED_TEMPERATURE, NEO_GRB + NEO_KHZ400);
Adafruit_NeoPixel stripPressure = Adafruit_NeoPixel(1, PIN_LED_PRESSURE, NEO_GRB + NEO_KHZ400);
Adafruit_NeoPixel stripHumidity = Adafruit_NeoPixel(1, PIN_LED_HUMIDITY, NEO_GRB + NEO_KHZ400);
Adafruit_NeoPixel stripMarkings = Adafruit_NeoPixel(1, PIN_LED_MARKINGS, NEO_GRB + NEO_KHZ400);
Adafruit_NeoPixel stripText = Adafruit_NeoPixel(1, PIN_LED_TEXT, NEO_GRB + NEO_KHZ400);
Adafruit_NeoPixel stripConnection = Adafruit_NeoPixel(1, PIN_LED_CONNECTION, NEO_GRB + NEO_KHZ400);

bool toggle;

void setup()
{  
  // Initialize switch pins.
  pinMode(PIN_LIMIT_TEMPERATURE, INPUT);           
  digitalWrite(PIN_LIMIT_TEMPERATURE, HIGH);  // turn on pullup resistors
  pinMode(PIN_LIMIT_PRESSURE, INPUT);           
  digitalWrite(PIN_LIMIT_PRESSURE, HIGH);   // turn on pullup resistors
  pinMode(PIN_LIMIT_HUMIDITY, INPUT);           
  digitalWrite(PIN_LIMIT_HUMIDITY, HIGH);   // turn on pullup resistors
  pinMode(PIN_SWITCH_MODE, INPUT);           
  digitalWrite(PIN_SWITCH_MODE, HIGH);   // turn on pullup resistors
  
  // Initialize steppers.
  stepperTemperature.setMaxSpeed(1000);
  stepperTemperature.setAcceleration(1000);
  stepperPressure.setMaxSpeed(1000);
  stepperPressure.setAcceleration(1000);
  stepperHumidity.setMaxSpeed(1000);
  stepperHumidity.setAcceleration(1000);

  // Initialize WS2812b LED .
  stripTemperature.begin();
  stripTemperature.show();
  stripPressure.begin();
  stripPressure.show();
  stripHumidity.begin();
  stripHumidity.show();
  stripMarkings.begin();
  stripMarkings.show();
  stripText.begin();
  stripText.show();  
  stripConnection.begin();
  stripConnection.show();  


  toggle = false;
  
}

void loop()
{

    SetConnectionStatusLED();

  
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
        stepperTemperature.moveTo(5000);
        stepperPressure.moveTo(5000);
        stepperHumidity.moveTo(5000);
       
      }    	
    }
    
  stepperTemperature.run();
  stepperPressure.run();
  stepperHumidity.run();
    
}


void SetConnectionStatusLED() 
{   
    stripConnection.setPixelColor(0, stripConnection.Color(255, 0, 0));
    stripConnection.show(); 
}


// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) 
{
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return stripConnection.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return stripConnection.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return stripConnection.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
