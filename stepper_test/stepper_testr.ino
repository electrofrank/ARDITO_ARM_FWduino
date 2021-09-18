// ProportionalControl.pde
// -*- mode: C++ -*-
//
// Make a single stepper follow the analog value read from a pot or whatever
// The stepper will move at a constant speed to each newly set posiiton,
// depending on the value of the pot.
//
// Copyright (C) 2012 Mike McCauley
// $Id: ProportionalControl.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $

#include <AccelStepper.h>

//base stepper
#define BASE_EN    27
#define BASE_DIR   29
#define BASE_STEP  31
//shoulder stepper (SPALLA)
#define SHOULDER_EN  35
#define SHOULDER_DIR  37
#define SHOULDER_STEP  39
//elbow stepper (GOMITO)
#define ELBOW_EN  43
#define ELBOW_DIR  45
#define ELBOW_STEP  47


// Define a stepper and the pins it will use
AccelStepper SHOULDER_STEPPER(1, SHOULDER_STEP, SHOULDER_DIR);
AccelStepper ELBOW_STEPPER(1, ELBOW_STEP, ELBOW_DIR);
AccelStepper BASE_STEPPER(1, BASE_STEP, BASE_DIR);

; // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

// This defines the analog input pin for reading the control voltage
// Tested with a 10k linear pot between 5v and GND

void setup()
{
  Serial.begin(115200);
  SHOULDER_STEPPER.setMaxSpeed(8000);
  SHOULDER_STEPPER.setAcceleration(1500.0);
  
  ELBOW_STEPPER.setMaxSpeed(4000);
  ELBOW_STEPPER.setAcceleration(1000.0);
  
  BASE_STEPPER.setMaxSpeed(8000);
  BASE_STEPPER.setAcceleration(1500.0);

  BASE_STEPPER.setSpeed(1000); //SI APRE

  ELBOW_STEPPER.setSpeed(1000); //SI APRE
  SHOULDER_STEPPER.setSpeed(1000); //SI APRE

}

void loop()
{

  //BASE_STEPPER.runToNewPosition(degToSTEP(-10)); //SI APRE

  //SHOULDER_STEPPER.runToNewPosition(degToSTEP(-20));
  //ELBOW_STEPPER.runToNewPosition(degToSTEP(-20));
  //digitalWrite(ELBOW_EN,HIGH);

  //SHOULDER_STEPPER.runToNewPosition(degToSTEP(5));

  BASE_STEPPER.runSpeed(); //SI APRE
  

  //BASE_STEPPER.runToNewPosition(degToSTEP(10));
   //ELBOW_STEPPER.runToNewPosition(degToSTEP(-10)); //SI APRE

    //SHOULDER_STEPPER.runToNewPosition(degToSTEP(5));

  //ELBOW_STEPPER.runToNewPosition(degToSTEP(2));

  //digitalWrite(SHOULDER_EN,HIGH);


}

int degToSTEP(float input) {
  //la risoluzione è 1.8gradi (200 a rivoluzione)
  int stepNum = (input * 500) / (1.8 / 1600); //1600microstep + riduzione 75:1 + 3:1
  return stepNum;

}
