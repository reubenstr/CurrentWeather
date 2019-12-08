// Random.pde
// -*- mode: C++ -*-
//
// Make a single stepper perform random changes in speed, position and acceleration
//
// Copyright (C) 2009 Mike McCauley
// $Id: Random.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $

#include <AccelStepper.h>

// Define a stepper and the pins it will use


AccelStepper stepper(AccelStepper::FULL2WIRE, 7, 6);

bool toggle;


void setup()
{  

  toggle = false;

 pinMode(9, OUTPUT); 
 pinMode(8, OUTPUT); 

  digitalWrite(9, HIGH);
   digitalWrite(8, HIGH);

  
}

void loop()
{
    if (stepper.distanceToGo() == 0)
    {
    	// Random change to speed, position and acceleration
    	// Make sure we dont get 0 speed or accelerations
    	delay(1000);
      
      if (toggle)
      {
        toggle = false;
        stepper.moveTo(0);
      }
      else 
      {
        toggle = true;
        stepper.moveTo(5000);
      }
            
    	
    	stepper.setMaxSpeed(500000);
    	stepper.setAcceleration(500000);
    }
    stepper.run();
}
