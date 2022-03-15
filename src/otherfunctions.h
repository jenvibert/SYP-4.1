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


void powerOff()
{
  
  // if elapsed time is greater than time interval, then can respond to power button
  unsigned int interval = 6000; //one minute in ms

  while(timeElapsed > interval)
  {
  // EEPROM.write(address, data); FILL THIS IN LATER FOR CALIBRATION DATA
  // writes information to EEPROM address that is specified above (0) and a byte of data that must be specified.
  // set KILL low and turn off the system
  digitalWrite(5, LOW);
  }
}