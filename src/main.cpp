#include <Wire.h>
#include <elapsedMillis.h>
#include <Adafruit_PWMServoDriver.h>
#include "FDC1004.h"
#include <TeensyThreads.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <Arduino.h>
#include <TimeLib.h>
#include <Bounce2.h>
#include <malloc.h>

//keep track of elasped time for power off sequence
elapsedMillis timeElapsed;



void setup() 
{
// initialize kill output pin
pinMode(5, OUTPUT);
digitalWrite(5, HIGH);

// initialize interrupt input pin
 pinMode(6, INPUT_PULLUP);
 attachInterrupt(digitalPinToInterrupt(6), powerOff, LOW);


}




void loop()
{







}