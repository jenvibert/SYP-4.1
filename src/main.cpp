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

time_t getTeensy3Time() {
  return Teensy3Clock.get();
}


void setup() 
{
// initialize kill output pin
pinMode(5, OUTPUT);
digitalWrite(5, HIGH);

// initialize interrupt input pin
 pinMode(6, INPUT_PULLUP);
 attachInterrupt(digitalPinToInterrupt(6), powerOff, LOW);

// Serial INIT
  Serial.begin(115200); // serial baud rate
  Serial.setTimeout(2);

  // I2C Bus init
  Wire.begin();
  Wire1.begin();
  Wire2.begin();

  Wire.setClock(400000);
  Wire1.setClock(400000);
  Wire2.setClock(400000);

  // Display startup screen
  setSyncProvider(getTeensy3Time);
  display.clearDisplay();
  display.fillRect(0,0,128,64, SSD1306_WHITE);
  display.setTextSize(1);
  display.setTextColor(SSD1306_BLACK);
  display.setCursor(20,16);
  display.println("FMG Controller");
  display.setCursor(19,30);
  display.print("Date:");
  display.print(month());
  display.print("-");
  display.print(day());
  display.print("-");
  display.println(year());
  display.setCursor(24,44);
  display.print("Version 0.01");
  display.display();
  delay(1000);

  const int t1 = threads.addThread(manageSerial, nullptr, 128); 
  if (t1 == -1) {
        Serial.println("error creating thread t1");
  }
  // set up thread for writing and reading from I2C

  const int t2 = threads.addThread(manageI2C, nullptr, 128);
  if (t2 == -1) {
        Serial.println("error creating thread t2");
  }



}




void loop()
{







}