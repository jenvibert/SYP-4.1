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


// OLED init

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire2, OLED_RESET);


// buttons and UI init
const int leftButtonPin = 9;
const int centerButtonPin = 8;
const int rightButtonPin = 7;

Bounce leftDebouncer = Bounce(leftButtonPin, 10);
Bounce centerDebouncer = Bounce(centerButtonPin, 10);
Bounce rightDebouncer = Bounce(rightButtonPin, 10);

byte previousStateLB = HIGH;         // what state was the button last time

unsigned int selectedIndex = 0;            // how many times has it changed to low
unsigned long selectedIndexChangeTime = 0;         // when count changed
unsigned int lastSelectedIndex = 0;     // last count printed

byte previousStateCB = HIGH;         // what state was the button last time
unsigned int centerButtonStatus = 0;            // how many times has it changed to low
unsigned long centerButtonChangeTime = 0;         // when count changed
unsigned int centerButtonPreviousStatus = 0;     // last count printed

byte previousStateRB = HIGH;         // what state was the button last time


// GLOBAL VARIABLES FOR GUI
// keep track of selected index, by default the first item in the menu, and keep track of the indices of the visible items
//int selectedIndex = 0;
int visibleIndices[5] = {0,1,2,3,4};

String menuItems[5][20] = {

{"Status","Mode","Settings","Debug"},
{"Start","Stop","Memory Usage:","Back"},
{"DAQ","2DoF","Back"},
{"Sensor setup","Set Map Values","Back"},
{"DAQ Output","Log","Back",""}

};

int currentMenuLevel = 0;
int lastMenuLevel = 0;
String menuTitle = "";

bool runningFunction = false; // flag to tell the loop that we are running a setup or mode function and to do a special update of the menu
int currentMode = 0; // mode 0 = mode not set. mode 1 = DAQ mode 2 = 2DoF mode
bool setupComplete = false; // on each startup setup needs to be verified - if existing setup okay to use, must select load setup from setup menu, else make new setup, then this flag will be set to true.
bool startFlag = false; // this flag tells the threads that communicate with the devices that it is okay to start. 
String statusMessage = "Idle";
bool sensorSetupRunning = false;
bool sensorCalibrationRunning = false;
bool sensorsOriented;
int sensorArrayCount = 0;
int mapMin = 0;
int mapMax = 255;
int servoMin = 0;
int servoMax = 180;

// set up time variable
time_t RTCTime;

// Sensor Array: id, address, bus, minValue, maxValue

int deviceArray[24][3] = {0}; // raw device array to hold all connected I2C devices (the board is only capable of connecting to 24 different devices).
String sensorArray[24][7] = {0}; // cleaned up sensor array - sensors are added from deviceArray to this one during calibration
// sensor array format: sensorID (0-24), I2C address, BusID, minValue, maxValue, purpose (0 = FMG, 1 = prosthesis torque), capdac

// options for FDC1004 set up
#define UPPER_BOUND  0X4000                 // max readout capacitance
#define LOWER_BOUND  (-1 * UPPER_BOUND)
#define CHANNEL 0                          // channel to be read
#define MEASURMENT 0                       // measurment channel
char result[100];
char userInput;
FDC1004 FDC;

// init variables to pass data between threads

volatile double forearmCapacitance[24] = {0};
volatile double forearmForce[24] = {0};
volatile double prosthesisCapacitance[6] = {0};
volatile double prosthesisTorque[6] = {0};
volatile double servoAngle[11] = {0};

volatile char startByte;
volatile int stiffness;

// mutex and variable to indicate if data is ready
static std::atomic<bool> dataReady;
static std::atomic<bool> serialCommandsReady;
static Threads::Mutex mutex;
static void manageSerial();
static void manageI2C();


// elapsed time for power button controller. Init pin 13 as led pin.
elapsedMillis timeElapsed;
int led = 13;

// map function for floating point
double mapf(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


// get time

time_t getTeensy3Time() {
  return Teensy3Clock.get();
}

// power off interrupt function
void powerOff()
{
  
  // if elapsed time is greater than time interval, then can respond to power button
  unsigned int interval = 6000; //one minute in ms

  while(timeElapsed > interval)
  {

    // insert any other code here needed to cleanly power down system here, can save data to EEPROM

  // EEPROM.write(address, data); // writes information to EEPROM address that is specified above (0) and a byte of data that must be specified.

    //do stuff
    digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(250);               // wait for a second
    digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
    delay(250);               // wait for a second
    digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(250);               // wait for a second
    digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
    delay(250);               // wait for a second
    digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(250);               // wait for a second
    digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
    delay(250);               // wait for a second
    digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(250);               // wait for a second
    digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
    delay(250);               // wait for a second

    // set KILL low and turn off the system
    digitalWrite(12, LOW);
  }
}

// Function to concatenate integers from serial read of stiffness values
unsigned concatenate(unsigned x, unsigned y) 
{
    unsigned pow = 10;
    while(y >= pow)
        pow *= 10;
    return x * pow + y;        
}

// functions to configure and read FDC data
void configureMeasurementonFDCwithAddressAndBus(TwoWire &bus, int addr, int capdac, int i) // the i here is the sensor index in the array - this is where cap dac values are stored.
{
  FDC.configureMeasurementSingle(MEASURMENT, CHANNEL, capdac, bus, addr);
  FDC.triggerSingleMeasurement(MEASURMENT, FDC1004_400HZ, bus, addr);
}

float getReadingFromFDCwithAddressAndBus(TwoWire &bus, int addr, int capdac, int i) // the i here is the sensor index in the array - this is where cap dac values are stored.
{
  uint16_t value[2];
  if (! FDC.readMeasurement(MEASURMENT, value, bus, addr))
  {
    int16_t msb = (int16_t) value[0];
    int32_t capacitance = ((int32_t)457) * ((int32_t)msb); //in attofarads
    capacitance /= 1000;   //in femtofarads
    capacitance += ((int32_t)3028) * ((int32_t)capdac);

    if (msb > UPPER_BOUND)               // adjust capdac accordingly
    {
      if (capdac < FDC1004_CAPDAC_MAX)
        capdac++;
    }
    else if (msb < LOWER_BOUND)
    {
      if (capdac > 0)
        capdac--;
    }
    if (sensorsOriented == true)
      sensorArray[i][6] = String(capdac);
    else
      deviceArray[i][2] = capdac;
      
    return (float)capacitance/1000;
  }
  else
  {
    return 0;
  }
}


static void manageSerial(void *arg)
{
  // producer aspect of thread needs to read the userInput (in the form of A0XXC, B0XXC, D000C as sent by the python experiment script), and parse it into a servoState and a stiffnessValue
  // those will be volatile global variables declared above
  // then the thread will act as a consumer and will need to wait until the force sensor I2C thread reads the force sensors and updates the current force sensor readout, then it needs to print it to
  // the serial interface.
  
  // the other feature needed is to set up the system into simple force sensor read out mode - ie to do data vis or run algorithms in matlab. The D000C command will simply
  // return all force sensor values in a csv list, whereas A and B start bytes will only send out the average of the flexor and extensor sensors along with servo angles etc

  // setTimeSlice() for this thread sets the run time of the thread... default is 100ms which is too long, we will change this to 1ms

  //threads.setTimeSlice(threads.id(), 100); // get current running thread's ID (which should be the manageSerial thread) and set its time slice to 2 ms

  // first read serial and get the user input

  char sb;
  unsigned s;

  // initializing these variables here because we don't want to lock the mutex while serial is being sent.

  double forearmCap[24] = {0};
  double forearmF[24] = {0};
  double prosthesisCap[6] = {0};
  double prosthesisTor[6] = {0};
  double servoAng[11] = {0};

  

  while(true) 
  {
    // only run check 
    Serial.print("manage serial thread running");

    if (Serial.available() > 0) // infinitely check for serial data, if available update the MCU global struct for what the user input was.
    {

      // if the mcu gets data from the python script, then run

        char userInput = Serial.read();
        
        Serial.print("user input received:"); 
        Serial.print(userInput);

        // format of serial input should be 'A0XXC' or 'B0XXC' where A and B are the start bytes, and C is the stop byte.
        // XX represents the stiffness value in N/m. The 0 is there as a placeholder because I'm too lazy to troubleshoot why it blanks out the first byte
        
        if (userInput == 'A' || userInput == 'B' || userInput == 'D') // detect start byte 'A' or 'B' or 'D' then read whole serial line until stop byte 'C' 
        {

            sb = userInput;
            
            Serial.print("user input received:"); 
            Serial.print(sb);
            
            // this printout is there to help us troubleshoot via the python terminal whether the arduino is getting the right serial code
            
            s = 0;
        
            while (userInput != 'C')
            {
                userInput = Serial.read();
                int userInt = Serial.parseInt();

                Serial.print(userInput);
                Serial.print(userInt);
                
                if (userInput != 'C')
                  s = concatenate(s, userInt);
              
            }

          Serial.print(",");
          
          Serial.print("stiffness = ");
          Serial.print(s);

          // this printout is there to troubleshoot from the python terminal whether the arduino is getting the right stiffness number
          
          Serial.print(",");

          // before we write the user input to the global struct variable, we will check if the mutex is locked or unlocked.

          while (mutex.try_lock() == 0) // try to lock the mutex to write the serial instructions to the thread. if it is locked, then yield the thread
            threads.yield();

          startByte = sb; // set the start byte that was received
          stiffness = s; // set the stiffness that was received

          mutex.unlock(); // unlock the mutex as we have finished writing the serial data to global struct.
          serialCommandsReady = true;
        }
    }

    // the other thing this function does is write the updated force data and other information to the serial interface.

    while (!dataReady)
      threads.yield();

    dataReady = false;

    // lock the shared data struct before reading.

    while (mutex.try_lock() == 0)
      threads.yield();

    // now loop through the force/capacitance/servo data arrays and get the data

    int i;

    for (i = 0; i < 24; i++)
    {
      forearmCap[i] = forearmCapacitance[i];
      forearmF[i] = forearmForce[i];

      if (i <= 6)
      {
        prosthesisCap[i] = prosthesisCapacitance[i];
        prosthesisTor[i] = prosthesisTorque[i];
      }
        
      if (i <= 11)
        servoAng[i] = servoAngle[i];
    }

    // once we have finished reading the arrays we can unlock the mutex
    mutex.unlock();

    // now print the variables in the format expected by the python script
    // if mode A or B, print:
    // if mode D, print: force[i], force[end]

    if (startByte == 'D') // if its the D startByte, we are requesting a simple readout of all the serial data.
    {
      for (i=0; i< 24; i++)
      {
        Serial.print(forearmF[i]);
      }
      for (i=0; i< 24; i++)
      {
        Serial.print(forearmCap[i]);
      }
      for (i=0; i< 6; i++)
      {
        Serial.print(prosthesisTor[i]);
      }
      for (i=0; i< 6; i++)
      {
        Serial.print(prosthesisCap[i]);
      }
      for (i=0; i< 11; i++)
      {
        Serial.print(servoAng[i]);
      }
    }
    else
    {
      // requesting data formatted for the experiment, which will be: (user input received A0XXC, stiffness:XX, avgFlexorCap, avgFlexorForce, avgExtCap, avgExtForce, tactorAngle)
      // this presents a single DoF 

      

    }
  }
}

static void manageI2C(void* arg)
{
  // manages communication to I2C devices on Wire0 and Wire1. Wire2 is dedicated to the display to prevent thread blocking.
  // also, the sensors and servos will need to be read/written to at ~400Hz, whereas the display can be slower at ~20Hz, and we do not want to block the display while running loops.
  
  // first this acts as a consumer thread. we need to lock mutex, get the serial input, unlock mutex and then produce the sensor values, then lock mutex, write to arrays, then unlock mutex

  // setTimeSlice() for this thread sets the run time of the thread... default is 100ms which is too long, we will change this to 5ms

  //threads.setTimeSlice(threads.id(), 100); // get current running thread's ID (which should be the manageI2C thread) and set its time slice to 5 ms

  char sb;
  int stiff;

  while(true)
  {
    Serial.print("manage I2C thread running");
    // waiting for serialCommandToBeReady only for DAQ mode

    while (!serialCommandsReady)
    {
      threads.yield(); // if serial commands not ready then yield the thread
    }

    serialCommandsReady = false;

    while(mutex.try_lock() == 0) // don't read from global serial data variables until manageSerial has unlocked.
    {
      threads.yield();
    }

    sb = startByte;
    stiff = stiffness;

    mutex.unlock();

    // got the start byte and stiffness value.
    // now we produce sensor data.
    double forearmCap[24] = {0};
    double forearmF[24] = {0};
    double prosthesisCap[6] = {0};
    double prosthesisTor[6] = {0};
    double servoAng[11] = {0};

    // if start byte is A or B, get first four sensors' capacitance data, average and write as a single value
    if (startByte == 'A' || startByte == 'B')
    {
      // first configure sensors
      int i;
      for (i = 0; i<sensorArrayCount; i++)
      {
        if (i<4) // get first four sensors (index going medially over flexor bulk)
        {
          char addrChar[sensorArray[i][1].length() + 1];
          strcpy(addrChar, sensorArray[i][1].c_str());
          int addr = strtoul(addrChar, NULL, 16); // get the I2C address
          int bus = sensorArray[i][2].toInt(); // get the I2C bus
          int capdac = sensorArray[i][6].toInt();

          // configure measurement at specified address and bus from device ID
          if (bus == 0)
            configureMeasurementonFDCwithAddressAndBus(Wire, addr, capdac, i);
          else if (bus == 1)
            configureMeasurementonFDCwithAddressAndBus(Wire1, addr, capdac, i);
          else if (bus == 2)
            configureMeasurementonFDCwithAddressAndBus(Wire2, addr, capdac, i);
        }
        else
        {
          break;
        }
      }

      delay(3); //delay 3 ms to let FDC capture data

      // now read data

      for (i = 0; i<sensorArrayCount; i++)
      {
        if (i<4) // get first four sensors (index going medially over flexor bulk)
        {
          char addrChar[sensorArray[i][1].length() + 1];
          strcpy(addrChar, sensorArray[i][1].c_str());
          int addr = strtoul(addrChar, NULL, 16); // get the I2C address
          int bus = sensorArray[i][2].toInt(); // get the I2C bus
          int capdac = sensorArray[i][6].toInt();

          float cap = 0;
          // configure measurement at specified address and bus from device ID
          if (bus == 0)
            cap = getReadingFromFDCwithAddressAndBus(Wire, addr, capdac, i);
          else if (bus == 1)
            cap = getReadingFromFDCwithAddressAndBus(Wire1, addr, capdac, i);
          else if (bus == 2)
            cap = getReadingFromFDCwithAddressAndBus(Wire2, addr, capdac, i);
          
          forearmCap[i] = cap;
          forearmF[i] = mapf(cap, sensorArray[i][3].toFloat(), sensorArray[i][4].toFloat(), mapMin, mapMax);
          float positionMeters = forearmF[i]/stiff;
          servoAng[i] = mapf(positionMeters, 0, 1, 0, 180);

          if (startByte == 'A')
          {
            // run the servo as well. insert code to run PWM servo driver here.
            


          }

        }
        else
        {
          break;
        }
      }



      startByte = 'E'; // reset start byte so this only runs once per serial command

    }
    else if (startByte == 'D')
    {
      


      // don't reset start byte so this runs continuously till interrupted by another start byte.
    }
    // if start byte is D get all sensor data and write to array 

    // lock mutex and write to global arrays

    while(mutex.try_lock() == 0)
    {
      threads.yield();
    }

    int i;
    for (i = 0; i < 24; i++)
    {
      forearmCapacitance[i] = forearmCap[i];
      forearmForce[i] = forearmF[i];

      if (i <= 6)
      {
        prosthesisCapacitance[i] = prosthesisCap[i];
        prosthesisTorque[i] = prosthesisTor[i];
      }
        
      if (i <= 11)
        servoAngle[i] = servoAng[i];
    }

    // once we have finished reading the arrays we can unlock the mutex
    mutex.unlock();




    dataReady = true; // set the data ready flag for the manageSerial thread to start running

  }

  
}




// this method takes a list of strings and a selected index, along with a subtitle and message to display in the status bar. 
// view type tells the function how to draw the list
  // 0 means its the main menu system, so you get the status bar, the top bar, and the menu list
  // 1 means its in the sensor setup mode, you get one line at the top (the subTitle) which tells the operation that was done (i.e calibrate min/max) and the first two items in the list are continue/retry buttons.
  // 2 means its in the debug daq mode - this shows the subtitle as the name of the mode (Debug-DAQ) and the time, and then a list of the sensors and their live values, with a back button at the top
  // 3 means its in the debug log mode - this shows the subtitle with the name of the mode (Debug-Log) and a list of up to 10 errors that are logged, with a back button at the top.
  
// at the start we give it the first row of the array, which is the main menu


void displayListWithTitleAndSelectionAndMessage(String subTitle, String * listOfItems, int selectedIndex, String message, int viewType)
{
  // first figure out what to draw, then draw it.

  // draw a white rect for the main screen contents, leaving the top 10 pixels dark for the status bar. 
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0);
  
  
  if (viewType == 0 || viewType == 2 || viewType == 3)
  {
    // white screen only for main menus
    display.fillRect(0,9,128,54, SSD1306_WHITE);
    display.setTextColor(SSD1306_WHITE);

    // show subtitles as formatted here only for viewTypes 0,2 and 3


    if (subTitle.length() > 1) // if a subtitle is passed, calculate the blank space needed to right justify the time and print it.
    {
      display.print(subTitle);
      int spaceLength = 13-subTitle.length(); // 13 is the optimal number between the length of the title and the time
      int i;
      for (i=0; i < spaceLength; i++)
        display.print(" ");
    }
    else
    {
      display.print("FMG"); // print the status bar text and time, to be aware if the program froze for some reason.
      display.print("          ");
    }

    // show time only for viewTypes 0, 2 and 3

    if (hour() < 10)
      display.print("0");
    display.print(hour());
    display.print(":");
    if (minute() < 10)
      display.print("0");
    display.print(minute());
    display.print(":");
    if (second() < 10)
      display.print("0");
    display.println(second());

    // make status bar and print message, but only for view types 0, 2 and 3.
    display.fillRect(0,54,128,10,SSD1306_BLACK);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,55);
    display.print(message);

  }
  else if (viewType == 1)
  {
    // if viewtype is 1 (sensor setup mode), don't show the time. 

    // keep display black
    // print the "subtitle" which is the operation that was done (i.e calibrate min/max)
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.print(subTitle);


  }

  // make a blank scroll bar. We need to put in a triangle if there are objects above or below at coordinates in commands shown below.
  //display.drawRect(118,9,128,64, SSD1306_BLACK);
  // display.fillTriangle(123,11,120,17,126,17, SSD1306_BLACK);
  // display.fillTriangle(123,60,120,54,126,54, SSD1306_BLACK);
  // after the above code, we will have a blank screen with a top bar and then a white rect where we can display our menu items.
  if (viewType == 0 || viewType == 2 || viewType == 3)
    display.setTextColor(SSD1306_BLACK);
  
  display.setCursor(0,9); // move cursor to just below the top bar

  // we can only display 5 objects on the screen at any given time. 
  // check that the selected index is indeed visible.
  
  // this is the same for all view types

  bool selectedIndexVisible = false;
  int visibleIndex = 0;

  int i;
  for (i = 0; i <= 3; i++)
  {
    if (selectedIndex == visibleIndices[i])
    {
  // selected index is visible, so set the flag to be true
      selectedIndexVisible = true;
      visibleIndex = i;
      break;
    }
  }
  

   if (selectedIndexVisible) // if the selected index is visible, draw the items in the passed menu array that are at the visible indices, and move the selection box over the selected index
   {
       int i;
       for (i = 0; i <= 3; i++)
       {
          //set cursor for each element based on i+1 (1-5), draw all the visible items
           int cursorLoc = i+1;
           display.setCursor(0,10*cursorLoc);
           display.println(listOfItems[visibleIndices[i]] );
       }
       // then draw a box around the selected item
       int cursorLoc = visibleIndex+1;
      if (viewType == 0 || viewType == 2 || viewType == 3)
        display.drawRect(0,10*cursorLoc,128,10, SSD1306_BLACK);
      else
        display.drawRect(0,10*cursorLoc,128,10, SSD1306_WHITE);

   }  
   else // if the selected index is not visible, we need to redefine the visibleindices
   {

     // if selectedindex > than the last visible index, we need to move the visible indices up by 1

    if (selectedIndex > visibleIndices[4])
    {
      int i;
      for (i = 0; i <= 3; i++)
      {
        visibleIndices[i] = visibleIndices[i]+1;
      }
      visibleIndex = 3;
    }
    else if (selectedIndex < visibleIndices[0]) // if selectedindex < than the first visible index, we need to move the visible indices down by 1
    {
      int i;
      for (i = 0; i <= 3; i++)
      {
        visibleIndices[i] = visibleIndices[i]-1;
        //Serial.print(visibleIndices[i]);
      }
      visibleIndex = 0;
    }
    
    // once the visible indices are updated, draw

    int i;
    for (i = 0; i <= 3; i++)
    {
          //set cursor for each element based on i+1 (1-5), draw all the visible items
          int cursorLoc = i+1;

          if (listOfItems[visibleIndices[i]].length() > 1) // if the visible list has more items than the actual list of items, ignore the list of items' empty values
          {
            display.setCursor(0,10*cursorLoc);
            display.println(listOfItems[visibleIndices[i]]);
          }
    }
       // then draw a box around the selected item
    
    
    int cursorLoc = visibleIndex+1;
    
    display.drawRect(0,10*cursorLoc,128,10, SSD1306_BLACK);
    }

  display.display();

}



void setup() 
{
  // POWER BUTTON CONTROLLER INIT

  // initialize kill output pin, set it high within 512ms of power on to prevent shut down.
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);

  // initialize the LED pin as an output.
  pinMode(led, OUTPUT);

  // initialize interrupt input pin
  pinMode(23, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(23), powerOff, LOW);

  // LTC4136 INIT
  pinMode(6, OUTPUT); //init enable pin, ser HIGH
  digitalWrite(6, HIGH);

  // Serial INIT
  Serial.begin(115200); // serial baud rate
  Serial.setTimeout(2);

  // BUTTONS INIT
  pinMode(leftButtonPin, INPUT_PULLUP);
  pinMode(centerButtonPin, INPUT_PULLUP);
  pinMode(rightButtonPin, INPUT_PULLUP);


  // I2C Bus init
  Wire.begin();
  Wire1.begin();
  Wire2.begin();

  Wire.setClock(400000);
  Wire1.setClock(400000);
  Wire2.setClock(400000);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
    
    while(1) // dont proceed, blink error code. long blink, two short blinks, single long blink.
    {
      digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(500);               // wait for a second
      digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
      delay(250);               // wait for a second
      digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(250);               // wait for a second
      digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
      delay(250);
      digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(250);
      digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
      delay(250);
      digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(500);
    }    
  }

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


  // set up thread for 

  // set up thread for communicating with serial for DAQ
  // the manageSerial thread will run the serial interface. it will read serial commands and write pertinent information as needed.
  
  
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

void debounceCenterButton() // gets center button input and updates centerbuttonstatus (1 = selected, 0 = not selected)
{
  if (centerDebouncer.update())
  {
    if (centerDebouncer.fallingEdge())
    {
      if (centerButtonStatus == 1)
        centerButtonStatus = 0;
      else
        centerButtonStatus = 1;
      
      centerButtonChangeTime = millis();
    }
  }
  else
  {
    if (centerButtonStatus != centerButtonPreviousStatus)
    {
      unsigned long nowMillis = millis();
      if (nowMillis - centerButtonChangeTime > 100)
      {
        centerButtonPreviousStatus = centerButtonStatus;
      }
    }
  }
}

void debounceLeftButton()
{
  if (leftDebouncer.update()) // if the debouncer detects a left button press (decrease selected index)
  {
    if (leftDebouncer.fallingEdge()) // and only on the falling edge of the button press
    {
      if (selectedIndex > 0) // and only if the selected index isnt zero,
        selectedIndex = selectedIndex - 1; // decrement the selected index
      selectedIndexChangeTime = millis();  // set the time the valid button press was made at
    }
  }
  else
  {
    if (selectedIndex != lastSelectedIndex) // otherwise a button press wasn't detected 
    {
      unsigned long nowMillis = millis();
      if (nowMillis - selectedIndexChangeTime > 100)
      {
        lastSelectedIndex = selectedIndex; 
      }
    }
  }
}

void debounceRightButton()
{
// get each button status after debouncing
// get size of current menu level 

int i;
for (i = 0; i <= 24; i++) // go through each menu item at the current menu level, and see if its length > 1 (so not an empty string/null)
{
  if (menuItems[currentMenuLevel][i].length() <= 1) //if yes, keep counting. if no, break and use i as size of the menu array a the current menu level.
    break;
}

unsigned int menuLength = i-1; // the max selected index will be menu length, so we have to set it up for zero index by subtracting 1

  

  if (rightDebouncer.update())
  {
    if (rightDebouncer.fallingEdge())
    {
      if (selectedIndex < menuLength)
        selectedIndex = selectedIndex + 1;
      selectedIndexChangeTime = millis();
    }
  }
  else
  {
    if (selectedIndex != lastSelectedIndex)
    {
      unsigned long nowMillis = millis();
      if (nowMillis - selectedIndexChangeTime > 100)
      {
        lastSelectedIndex = selectedIndex;
      }
    }
  }
  
}

void statusMenuFunction()
{
  if (currentMenuLevel==1 && selectedIndex==0)
  {
    // start button clicked:

    // if mode not set, and sensor set up not complete, cannot start
    String errorCode = "Err";
    if (currentMode == 0)
    {
      errorCode = errorCode + ",no mode";
    }
    
    if (setupComplete == false)
    {
      errorCode = errorCode + ",no setup";
    }

    Serial.print(errorCode);

    if (errorCode.length() > 6) // error present, do not start, set message to error
    {
      statusMessage = errorCode;
      startFlag = false;
    }
    else     // if mode set, sensor set up complete, set start flag to True, this is then seen by the sensor and serial threads and those are run
    {
      statusMessage = "Running";
      startFlag = true;
    }
  }
  else if (currentMenuLevel==1 && selectedIndex==1)
  {
    // if process running, stop it. if not, 

    if (startFlag == true)
    {
      statusMessage = "Stopped";
      startFlag = false;
    }
    else
    {
      //display error that nothing is running
      statusMessage = "Err,nothing to stop";
    }
  }
}

void modeMenuFunction()
{
  if (currentMenuLevel==2 && selectedIndex == 0) // mode menu selected and DAQ mode selected
  {

    // set mode flag and set selected mode in main menu
    currentMode = 1;
    menuItems[0][1] = "Mode: DAQ";
 
  }
  else if (currentMenuLevel == 2 && selectedIndex == 1)
  {

    // set mode flag and set selected mode in main menu
    currentMode = 2;
    menuItems[0][1] = "Mode: 2DoF";

  }
}



void getAverageSensorOutputForSamples(int sampleCount, float *sampleSum)
{
  if (sensorsOriented == true)
  {
    // sensors have been oriented so use the main sensorArray
    int i;
    for (i = 0; i < sampleCount; i++) // for specified samples, get a sample from each sensor and add it to the last. then divide by 36000 to get average minValue for each sensor.
    {
      int j;
      for (j=0; j<sensorArrayCount; j++) // loop through deviceArray to configure measurements for connected sensors
      {

        if (sensorArray[j][1].length() > 1) // if the device array row is not empty (ie sensor exists), then write config to sensor.
        {
          char addrChar[sensorArray[j][1].length() + 1];
          strcpy(addrChar, sensorArray[j][1].c_str());
          int addr = strtoul(addrChar, NULL, 16); // get the I2C address
          int bus = sensorArray[j][2].toInt(); // get the I2C bus

          // configure measurement at specified address and bus from device ID
          if (bus == 0)
            configureMeasurementonFDCwithAddressAndBus(Wire, addr, sensorArray[j][6].toInt(), j);
          else if (bus == 1)
            configureMeasurementonFDCwithAddressAndBus(Wire1, addr, sensorArray[j][6].toInt(), j);
          else if (bus == 2)
            configureMeasurementonFDCwithAddressAndBus(Wire2, addr, sensorArray[j][6].toInt(), j);
        }
      }

      delay(3); // delay 3 ms to wait for sensors to collect measurement before reading.

      for (j=0; j<sensorArrayCount; j++) // loop through deviceArray to read measurements for connected sensors
      {
        if (sensorArray[j][1].length() > 1) // if the device array row is not empty (ie sensor exists), then read the sensor.
        {
          char addrChar[sensorArray[j][1].length() + 1];
          strcpy(addrChar, sensorArray[j][1].c_str());
          int addr = strtoul(addrChar, NULL, 16); // get the I2C address
          int bus = sensorArray[j][2].toInt(); // get the I2C bus

          float cap = 0;
          // get reading at specified address and bus from device ID
          if (bus == 0)
            cap = getReadingFromFDCwithAddressAndBus(Wire, addr, sensorArray[j][6].toInt(), j);
          else if (bus == 1)
            cap = getReadingFromFDCwithAddressAndBus(Wire1, addr, sensorArray[j][6].toInt(), j);
          else if (bus == 2)
            cap = getReadingFromFDCwithAddressAndBus(Wire2, addr, sensorArray[j][6].toInt(), j);


          sampleSum[j] = sampleSum[j]+cap;
        }
      }
    }
  }
  else
  {
    // sensors not oriented so use device array
    int i;
    for (i = 0; i < sampleCount; i++) // for specified samples, get a sample from each sensor and add it to the last. then divide by 36000 to get average minValue for each sensor.
    {
      int j;
      for (j=0; j<24; j++) // loop through deviceArray to configure measurements for connected sensors
      {
        if (deviceArray[j][0] != 0) // if the device array row is not empty (ie sensor exists), then write config to sensor.
        {
          int addr = deviceArray[j][0]; // get the I2C address
          int bus = deviceArray[j][1]; // get the I2C bus


          // configure measurement at specified address and bus from device ID
          if (bus == 0)
            configureMeasurementonFDCwithAddressAndBus(Wire, addr, deviceArray[j][2], j);
          else if (bus == 1)
            configureMeasurementonFDCwithAddressAndBus(Wire1, addr, deviceArray[j][2], j);
          else if (bus == 2)
            configureMeasurementonFDCwithAddressAndBus(Wire2, addr, deviceArray[j][2], j);
        }
      }

      delay(3); // delay 3 ms to wait for sensors to collect measurement before reading.

      for (j=0; j<24; j++) // loop through deviceArray to read measurements for connected sensors
      {
        if (deviceArray[j][0] != 0) // if the device array row is not empty (ie sensor exists), then read the sensor.
        {
          int addr = deviceArray[j][0]; // get the I2C address
          int bus = deviceArray[j][1]; // get the I2C bus


          float cap = 0;
          // get reading at specified address and bus from device ID
          if (bus == 0)
            cap = getReadingFromFDCwithAddressAndBus(Wire, addr, deviceArray[j][2], j);
          else if (bus == 1)
            cap = getReadingFromFDCwithAddressAndBus(Wire1, addr, deviceArray[j][2], j);
          else if (bus == 2)
            cap = getReadingFromFDCwithAddressAndBus(Wire2, addr, deviceArray[j][2], j);

          sampleSum[j] = sampleSum[j]+cap;
        }
      }
    }
  }
  
  
}

void calibrateSensors()
{


  sensorCalibrationRunning = true; //set the loop flag for calibration to be true - this way the I2C scanner stuff doesnt run once its done.
  sensorsOriented = false;

  while(sensorCalibrationRunning)
  {
    // we need to get the orientation of the FMG band and order of sensors.
    // this means setting the index sensor on the FCR muscle belly, and other sensors arranged medially until it reaches the last FMG sensor
    // ask user to push on the index sensor over FCR, for 2 seconds, record all three sensors for 2 seconds (400 samples)

    // At this point we have an array of sensor addresses and their buses
    // need to order them starting at FCR going around medially till all of them are addressed.
    
    while(!sensorsOriented)
    {
      int sensorCount;
      int deviceArrayCount = 0;
      
      int j;
      for (j=0; j<24;j++)
      {
        if (deviceArray[j][0] == 0)
        {
          deviceArrayCount = j;
          break;
        }
      }
      
      display.setTextColor(SSD1306_WHITE); // set text colour white, as the screen will now be black.
      display.clearDisplay(); // clear display before showing relax message.
      display.setCursor(0,0);
      display.println("Calibrating sensors.");
      display.println("Relax for minimum\nreading.");
      display.display();

      delay(1000);

      float minValues[deviceArrayCount] = {0};
      
      getAverageSensorOutputForSamples(400,&minValues[0]); // get the minima for the sensors to compare to for calculating sensors that are activated.

      for (sensorCount = 0; sensorCount < deviceArrayCount; sensorCount++) // outer loop to go through the new sensorArray - for every ordered sensor, we need to read all sensors to find the one of interest at sensorCount
      {
        // get sensor data until a sensor is pressed.

        display.setTextColor(SSD1306_WHITE); // set text colour white, as the screen will now be black.
        display.clearDisplay(); // clear display before showing relax message.
        display.setCursor(0,0);
        display.println("Calibrating sensors.");
        display.println("Relax and push on\nsensor 0 to 24 going\nmedially.");
        display.println("Press on sensor ");
        display.print(sensorCount);
        display.display();

        bool sensorAtCountOriented = false;

        while(sensorAtCountOriented == false)
        {
          int i;
          for (i = 0; i < deviceArrayCount; i++)
          { 
            if (deviceArray[i][0] != 0) // check that a sensor exists at that part of the array.
            {

              int addr = deviceArray[i][0]; // get the I2C address
              int bus = deviceArray[i][1]; // get the I2C bus

                // configure measurement at specified address and bus from device ID
              if (bus == 0)
                configureMeasurementonFDCwithAddressAndBus(Wire, addr, deviceArray[i][2], i);
              else if (bus == 1)
                configureMeasurementonFDCwithAddressAndBus(Wire1, addr, deviceArray[i][2], i);
              else if (bus == 2)
                configureMeasurementonFDCwithAddressAndBus(Wire2, addr, deviceArray[i][2], i);

              delay(3);

              float cap = 0;
                // get reading at specified address and bus from device ID
              if (bus == 0)
                cap = getReadingFromFDCwithAddressAndBus(Wire, addr, deviceArray[i][2], i);
              else if (bus == 1)
                cap = getReadingFromFDCwithAddressAndBus(Wire1, addr, deviceArray[i][2], i);
              else if (bus == 2)
                cap = getReadingFromFDCwithAddressAndBus(Wire2, addr, deviceArray[i][2], i);


              float minim = (minValues[i]/400);

              if (cap > minim+3) // if the capacitance is > the sensor minimum plus threshold capacitance, set here for 3pF but can be adjusted if needed.
              {
                // this is the sensor being activated.
                // sensor in deviceArray at index i should be inserted in sensorArray at index sensorCount
                // sensor array format: sensorID (0-24), I2C address, BusID, minValue, maxValue, purpose (0 = FMG, 1 = prosthesis torque)

                sensorArray[sensorCount][0] = String(sensorCount); // set sensorID to sensorCount
                sensorArray[sensorCount][1] = String(deviceArray[i][0], HEX); // set I2C address of sensor at sensorCount to the one that was activated
                sensorArray[sensorCount][2] = String(deviceArray[i][1]); // set I2C bus ID
                sensorArray[sensorCount][3] = String(minim); //set sensor min value
                sensorArray[sensorCount][6] = String(deviceArray[i][2]); // set capdac
                deviceArray[i][0] = 0;
                sensorAtCountOriented = true;
                break;
              }
            }
          }
        }
      }


      int i;

      for (i = 0; i < 24; i++)
      {
        if (sensorArray[i][1].length() < 2) // if the sensor at index i has an i2c address that isnt zero, it exists so add to count
        {
          sensorArrayCount = i;
          break;
        }
      }

      if (deviceArrayCount == sensorArrayCount)
      {
        // then orientation is complete as we have added all sensors from the device array to the sensor array, we can break this loop and go on to the max min calibration.
        sensorsOriented = true;
        break;
      }
    }

    // sensors have been oriented and the min value has been determined. now we need to get the max value.
    // record all sensors for the 3 seconds, find the max sensor, and mark it's sensorID as zero. repeat, but mark next sensor as 1, and so on.
    // first we get the max values for all sensors. we put this in a loop to allow retry if something goes wrong during relaxation trial. display: ask user to relax completely

    bool maxValueSet = false;
    while (maxValueSet == false)
    {
      display.setTextColor(SSD1306_WHITE); // set text colour white, as the screen will now be black.
      display.clearDisplay(); // clear display before showing relax message.
      display.setCursor(0,0);
      display.println("Calibrating sensors.\n");
      display.println("Contract muscles\ntouching band for\n 4 sec to get max.\n");
      display.println("Press select to start");
      display.display();

      while(true) // wait while user contracts and presses select to collect minimum data
      {
        debounceCenterButton();
        if (centerButtonStatus == 1)
        {
          centerButtonStatus = 0;
          break;
        }
      }

      display.clearDisplay(); // clear display before showing relax message.
      display.setCursor(0,0);
      display.println("Calibrating sensors.\n");
      display.println("Contract muscles\ntouching FMG band for\n 4 sec to get max.\n");
      display.println("Getting max values...");
      display.display();

      // for three seconds, collect data from all sensors, and average each sensors data over time to find minimum value.
      delay(500);
      int sampleCount = 20;
      float sampleSum[sensorArrayCount] = {0};


      getAverageSensorOutputForSamples(sampleCount, &sampleSum[0]);

      // 3 seconds of sampling done. calculate average from the array of sample sums

      int i;
      for (i = 0; i<sensorArrayCount; i++)
      {
        sensorArray[i][4] = String(sampleSum[i]/sampleCount);
      }

      Serial.print("sensor max recorded.");
      Serial.print(sensorArray[0][4]);
      Serial.print(sensorArray[1][4]);
      delay(1000);

      // create a function here to show the min values for each attached sensor. we need to get the left and right buttons and move the selected index to scroll the list to show up to 24 sensors.
      // the first two buttons will be "continue to get max" or "retry getting mins"
      // if continue to get max selected, break loop and move on to getting max values, else, do not break loop and restart getting mins.


      while (true)
      {

        debounceRightButton(); // these functions updates the selected index and whether the buttons were pressed or not
        debounceLeftButton();
        debounceCenterButton(); // ask user to press select if satisfied with minimum value calibration, if true, break this loop and move on to max value setup

        String listToDisplay[24]={0};

        int i;
        for (i=0; i < sensorArrayCount; i++)
        {
          listToDisplay[i] = sensorArray[i][0] + " " + sensorArray[i][1] + " " + sensorArray[i][2] + " " + sensorArray[i][3] + " " + sensorArray[i][4];
        }

        displayListWithTitleAndSelectionAndMessage("Calibrate sensor max", &listToDisplay[0], selectedIndex, "Press select to continue", 1);

        if (centerButtonStatus == 1)
        {
          centerButtonStatus = 0;
          maxValueSet = true;
          sensorCalibrationRunning = false;    // tell the main program loop that sensor calibration is done so we can quit the calibration loop once the max value is also determined.
          break;
        }
      }
    } 
  

  // once min value set, we need to get the max values.
  // then we need to get the max values for all sensors - ask user to make a tight fist and hold for 3 seconds.
  // then ask user to open hand and stretch all fingers out as hard as they can and hold for 3 seconds
  // during that time we will capture 3 seconds worth of data, then determine the max value for each sensor.

  }
}

int I2Cscanner(TwoWire &I2CBus, int busID)
{
  byte error, address;
  int nDevices = 0;

  for (address = 1; address < 127; address++) // loop through all I2C addresses
  {
    I2CBus.beginTransmission(address);
    error = I2CBus.endTransmission();

    if (error == 0) // if no I2C errors at the searched address, a device was found.
    {
      // a device was found at this address without any errors. now read it again and see what the devID and manID is to make sure its a sensor.
      Serial.print("current address:");
      Serial.println(address, HEX);
      uint16_t devID = FDC.getDeviceID(I2CBus,address);
      
      if (devID == 0x1004)
      {
        int i;
        for (i = 0; i<24; i++)
        {
          if (deviceArray[i][0]==0) // if the deviceArray address is zero, add a sensor into the array at that index. this should add sensors in order.
          {
            deviceArray[i][0] = address;
            deviceArray[i][1] = busID;
            break;
          }
        }
        nDevices++;
      }
    } 
    else if (error==4) 
    {
      
      Serial.print(F("Unknown error at address 0x"));
      if (address < 16) 
      {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }

  }

  return nDevices;
}


void settingsMenuFunction()
{

  if (selectedIndex == 1) // if calibration selected, check to see if settings are already put in. if not, you have to set up then calibrate
  {
    if (setupComplete == false)
      statusMessage = "Err,need setup first";
    else
    {
      sensorSetupRunning = true; // erase old sensor array and device array.
      sensorArrayCount = 0;
      calibrateSensors();
    }
  }
  else if (selectedIndex == 0)
  {
    sensorSetupRunning = true;
    bool keepScanning = true; // if the user presses select and agrees the number of detected sensors matches the physical config, then set this false to kill the loop and continue.
    
    while (sensorSetupRunning)
    {
      //
      // sensor setup selected.

      display.clearDisplay(); // clear display to draw the sensor set up screens.
      display.setTextColor(SSD1306_WHITE); // set text colour white, as the screen will now be black.


      // run I2C scanner, and create a list of scanned sensors. if the number of sensors matches what is connected, ask user to press select to continue.
      
      int numberOfSensors;
      while (keepScanning)
      {
          // scan for sensors:
        int sensorsBusOne = I2Cscanner(Wire, 0);
        int sensorsBusTwo = I2Cscanner(Wire1, 1);
        numberOfSensors = sensorsBusOne + sensorsBusTwo;
        
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("Scanning:");
        display.print(numberOfSensors);
        display.println(" FDCs detected");
        display.println("If good press select");
        display.display();

        debounceCenterButton();

        if (centerButtonStatus == 1) // select pressed, don't erase the device array and pass it on to the next part of the scan
        {
          centerButtonStatus = 0;
          keepScanning = false;
          break;
        }

        // if the select button wasn't pressed, erase the device array to prepare it for the next iteration of scanning
        int i;
        for (i = 0; i<24; i++)
        {

            deviceArray[i][0] = 0;
            deviceArray[i][1] = 0;
          
        }

      }

      delay(50);

      int i;
      int deviceCounter = 0;
      for (i = 0; i<24; i++)
      {
        if (deviceArray[i][0] != 0)
          deviceCounter++;
      }

      display.setCursor(0,0);
      display.println("Scan complete.");
      display.print(deviceCounter);
      display.println(" sensors confirmed.");
      display.display();

      delay(1000);

      // so now we have a raw array of the sensors with i2c addresses and bus id.
      // next we need to calibrate them, which ends the set up loop.

      calibrateSensors();
      // once calibration loop is done, stop sensor setup too.
      sensorSetupRunning = false;
      setupComplete = true;
    }
  }
}

void loop() 
{
  // main program loop - will manage user interface

  debounceRightButton(); // these functions updates the selected index and whether the buttons were pressed or not
  debounceLeftButton();
  debounceCenterButton();

  // if center button pressed
    // get the item at the selectedIndex in the menuArray
    // go to the individual function to run the selected item
    // individual function runs display update code
  
    // let user know sensor setup complete.

  if (sensorArrayCount > 0)
  {
    if (startFlag == true && currentMode == 1)
      statusMessage = "DAQ on - sensors ok";
    else if (startFlag == false && currentMode == 1)
      statusMessage = "DAQ off - sensors ok";
    else if (startFlag == true && currentMode == 2)
      statusMessage = "2DoF on - sensors ok";
    else if (startFlag == false && currentMode == 2)
      statusMessage = "2DoF off - sensors ok";
    else if (startFlag == false && currentMode == 0)
      statusMessage = "Idle - sensors ok";
  }
  else
  {
    statusMessage = "Idle";
  }
  

  if (centerButtonStatus == 1) // if center button pressed, update the title and menu level and selected index
  {
    centerButtonStatus = 0;

    if (menuItems[currentMenuLevel][selectedIndex] == "Back")
    {
      //Serial.print("going back to ");
      
      if (statusMessage.charAt(0) == 'E' && statusMessage.charAt(1) == 'r') // clear error messages on going back in the menu
        statusMessage = "Idle";

      if (lastMenuLevel != 0)
        menuTitle = menuItems[lastMenuLevel][currentMenuLevel-1]; // title of sub menu
      else
        menuTitle = "";
      

      selectedIndex = currentMenuLevel-1; //reset selected index to current menu level
      currentMenuLevel = lastMenuLevel; // change current menu level to the selected one

    }
    else if (currentMenuLevel == 0)
    {
      //Serial.print("going forward to ");

      menuTitle = menuItems[currentMenuLevel][selectedIndex]; // title of sub menu

      lastMenuLevel = currentMenuLevel; // set the last menu level to be the current one before changing the current one to the new one
      currentMenuLevel = selectedIndex+1; // change current menu level to the selected one
      selectedIndex = 0; //reset selected index to 0
    }
    else
    {
      // a function was selected

      if (currentMenuLevel == 1)
      {
        // the status menu functions should be inserted here
        statusMenuFunction();
      }
      else if (currentMenuLevel == 2)
      {
        // insert mode menu functions
        if (startFlag == false)
          modeMenuFunction(); // show mode menu only if start flag is not true - can't set the mode while the program is running.
        else
          statusMessage == "Err,stop before change";
      }
      else if (currentMenuLevel == 3) // settings menu
      {
        Serial.print("settings menu clicked.");

        if (startFlag == false)
        {
          Serial.print("settings menu function started, as start flag is false");
          settingsMenuFunction();
        }
        else
          statusMessage == "Err,stop before setup";
      }
    }

    
  }

  // update the memory display if the current menu is status menu
  if (currentMenuLevel == 1)
  {
    int mem = mallinfo().arena - mallinfo().keepcost;
    menuItems[currentMenuLevel][2] = "Memory Usage: " + String(mem);
  }



  // then display
  displayListWithTitleAndSelectionAndMessage(menuTitle, menuItems[currentMenuLevel], selectedIndex, statusMessage, 0);
  yield();
}
