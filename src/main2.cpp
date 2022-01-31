#include <Arduino.h>
#include "FDC1004.h"
#include <Wire.h>
#include <util/atomic.h> // For the ATOMIC_BLOCK macro
#include <ctime>
using namespace std;

#define ENCA 21
#define ENCB 20
#define ENCC 22
#define ENCD 23
#define PWM 4
#define PWM1 5
#define IN1 15
#define IN2 14



int deviceArray[24][3] = {0};
int led = 13; // assign led pin

volatile int posi1 = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
volatile int posi2 = 0;

//DECLARE GLOBAL VARIABLES FOR MIN AND MAX OF SENSORS
float avgFlexsor_min;
float avgFlexsor_max;
float avgExtensor_min;
float avgExtensor_max;
float avgFingerTip;

// options for FDC1004 setup
#define UPPER_BOUND 0X4000 // max readout capacitance
#define LOWER_BOUND (-1 * UPPER_BOUND)
#define CHANNEL 0    // channel to be read
#define MEASURMENT 0 // measurment channel
char result[100];
char userInput;
FDC1004 FDC;
int sensorCount = 0;

// make a global array of cap values that are just updated for each loop iteration (otherwise need to create arrays each loop iteration
float capValues[24][3] = {0};


// get time
time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

// functions to configure and read FDC data
void configureMeasurementonFDCwithAddressAndBus(TwoWire &bus, int addr, int capdac) // the i here is the sensor index in the array - this is where cap dac values are stored.
{
  FDC.configureMeasurementSingle(MEASURMENT, CHANNEL, capdac, bus, addr);
  FDC.triggerSingleMeasurement(MEASURMENT, FDC1004_400HZ, bus, addr);
}

float getReadingFromFDCwithAddressAndBus(TwoWire &bus, int addr, int capdac, int i) // the i here is the sensor index in the array - this is where cap dac values are stored.
{
  uint16_t value[2];
  if (!FDC.readMeasurement(MEASURMENT, value, bus, addr))
  {
    int16_t msb = (int16_t)value[0];
    int32_t capacitance = ((int32_t)457) * ((int32_t)msb); //in attofarads
    capacitance /= 1000;                                   //in femtofarads
    capacitance += ((int32_t)3028) * ((int32_t)capdac);

    if (msb > UPPER_BOUND) // adjust capdac accordingly
    {
      if (capdac < FDC1004_CAPDAC_MAX)
        capdac++;
    }
    else if (msb < LOWER_BOUND)
    {
      if (capdac > 0)
        capdac--;
    }

    deviceArray[i][2] = capdac;

    return capacitance;
  }
  else
  {
    return 0;
  }
}

// I2C Scanner to retrieve all device ids
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
      uint16_t devID = FDC.getDeviceID(I2CBus, address);

      if (devID == 0x1004)
      {
        int i;
        for (i = 0; i < 24; i++)
        {
          if (deviceArray[i][0] == 0) // if the deviceArray address is zero, add a sensor into the array at that index. this should add sensors in order.
          {
            deviceArray[i][0] = address;
            deviceArray[i][1] = busID;
            break;
          }
        }
        nDevices++;
      }
    }
    else if (error == 4)
    {

      Serial.print(F("Unknown error at address 0x"));
      if (address < 16)
      {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }
  return nDevices;
}

void clearInputBuffer()
{
  while (Serial.available() > 0) Serial.read();
}

bool waitForInput()
{
  while (true)
  {
    if (Serial.available() > 0)
    {
      Serial.println("Reading Input!");
      char firstByte = Serial.read();
      return (firstByte == 'y');
    }
  }
  return false;
}


void setMotor1 (int dir1, int pwmVal, int pwm, int in1, int in2)
{ //setting up motor
  analogWrite(pwm, pwmVal);
  if (dir1 == 1)
  {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (dir1 == -1)
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void setMotor2 (int dir2, int pwmVal, int pwm, int in1, int in2)
{ //setting up motor
  analogWrite(pwm, pwmVal);
  if (dir2 == 1)
  {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (dir2 == -1)
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void readEncoder1()
{
  int b = digitalRead(ENCB);
  if (b > 0)
  {
    posi1++;
  }
  else
  {
    posi1--;
  }
}

void readEncoder2()
{
  int a = digitalRead(ENCD);
  if (a > 0)
  {
    posi2++;
  }
  else
  {
    posi2--;
  }
}

float avgSensorOutput(TwoWire &bus, int sampleCount, int sensorindex)
{
  float totalForce = 0;
  long cap = 0;

  for (int i = 0; i < sampleCount; i++)
  {
    configureMeasurementonFDCwithAddressAndBus(bus, deviceArray[sensorindex][0], deviceArray[sensorindex][2]);
    delay(5);
    cap = getReadingFromFDCwithAddressAndBus(bus, deviceArray[sensorindex][0], deviceArray[sensorindex][2], sensorindex);
    totalForce += cap;
    
    //Serial.printf("sample: %d\n", i);
  }
  float avgForce = totalForce / sampleCount;
  Serial.printf("Avg sensor value: %f\n", avgForce);
  
  capValues[sensorindex][2] = avgForce;
  return avgForce;
}

void setup()
{
  // delay(2500);

  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder1, RISING);
  pinMode(ENCC, INPUT);
  pinMode(ENCD, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCC), readEncoder2, RISING);

  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM1, OUTPUT);
  

  // initialize the LED pin as an output.
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH); // turn the LED on (HIGH is the voltage level) while set up is ongoing

  // Serial INIT
  Serial.begin(115200); // serial baud rate
  Serial.setTimeout(5);

  // I2C Bus init
  Wire.begin();
  Wire1.begin();
  Wire2.begin();

  Wire.setClock(400000);
  Wire1.setClock(400000);
  Wire2.setClock(400000);

  delay(500);

  // blink LED once to signify set up started
  digitalWrite(led, LOW);
  delay(250);
  digitalWrite(led, HIGH);
  delay(250);
  digitalWrite(led, LOW);
  delay(250);

  // scan for sensors and populate deviceArray
  int numberOfSensors;

  while (true)
  {
    int j;

    for (j = 0; j < 24; j++)
    {
      deviceArray[j][0] = 0;
      deviceArray[j][1] = 0;
    }

    int sensorsBusOne = I2Cscanner(Wire, 0); //setting up first bus for I2C
    int sensorsBusTwo = I2Cscanner(Wire1, 1); //setting up second bus for I2C
    int sensorBusThree = I2Cscanner(Wire2, 2);

    numberOfSensors = sensorsBusOne + sensorsBusTwo + sensorBusThree;
    sensorCount = numberOfSensors; // store number of sensors detected in a global variable so you dont have to keep counting number of sensors in main loop (per previous code)
    Serial.println(sensorCount);
    // when at least one sensor is detected, break the loop. If no sensors detected, try to scan again until sensors are detected. (i.e. if unknown error occurs)
    if (numberOfSensors > 0 && numberOfSensors < 25)
      break;
  }

  digitalWrite(led, HIGH); // set Led high to show that setup complete

  for (int x = 0; x < 24; x++)
  {
    Serial.print("i2c address: ");
    Serial.println(deviceArray[x][0]);
    Serial.print("bus: ");
    Serial.println(deviceArray[x][1]);
  }

  bool calibrationDone = false;
  bool serialInput = false;

  delay(3000);

  while (!calibrationDone)
  {

    Serial.println("1.0 Relax and press y to start calibration");
    waitForInput();
    Serial.println(" ");
    Serial.println("1.1 getting flexor minima, extensor maxima, and fingertip avg...");
    Serial.println("1.2 flexor minima");
    avgFlexsor_min = avgSensorOutput(Wire, 400, 0);
    delay(1000);

    Serial.println("1.3 extensor maxima");
    avgExtensor_max = avgSensorOutput(Wire2, 400, 2);
    delay(1000);

    Serial.println("1.4 finger tip avg");
    avgFingerTip = avgSensorOutput(Wire1, 400, 1);
    delay(1000);
     
    Serial.println("1.5 flexor minima, extensor maxima, and fingertip avg acquired");
    delay(1000);
    Serial.println(" ");

    Serial.println("2.0 Flex as hard as you can and press y to continue calibration");
    waitForInput();
    Serial.println(" ");
    Serial.println("2.1 getting flexor maxima and extensor minima...");
    Serial.println("2.2 flexor maxima");
    avgFlexsor_max = avgSensorOutput(Wire, 400, 0);
    delay(1000);
    
    Serial.println("2.3 extensor minima");
    avgExtensor_min = avgSensorOutput(Wire2, 400, 2);
    delay(1000);

    Serial.println("2.4 flexor maxima and extensor minima acquired");
    delay(1000);
    
    Serial.println(" ");
    Serial.println("3.0 calibration complete. press y to accept or n to restart.");
    serialInput = waitForInput();
    
    if (serialInput == true)
    {
      Serial.println("3.1 calibration complete."); 
      calibrationDone = true;
      break;
    }
    else
    {
      calibrationDone = false;
    }
  }

}

void loop()
{
  // data collection part: sample the sensors, and print them in the necessary key val format to the serial port

  for (int i = 0; i < sensorCount; i++)
  {

    int addr = deviceArray[i][0]; // get the I2C address
    int bus = deviceArray[i][1];
    int capdac = deviceArray[i][2]; // get the capdac value

    if (bus == 0)
      configureMeasurementonFDCwithAddressAndBus(Wire, addr, capdac);
    else if (bus == 1)
      configureMeasurementonFDCwithAddressAndBus(Wire1, addr, capdac);
    else if (bus == 2)
      configureMeasurementonFDCwithAddressAndBus(Wire2, addr, capdac);
  }

  delay(3); // delay 3 ms to let FDC capture data

  for (int i = 0; i < sensorCount; i++)
  {

    int addr = deviceArray[i][0]; // get the I2C address
    int bus = deviceArray[i][1];
    int capdac = deviceArray[i][2]; // get the capdac value

    long cap = 0;
    // configure measurement at specified address and bus from device ID
    if (bus == 0)
      cap = getReadingFromFDCwithAddressAndBus(Wire, addr, capdac, i);
    else if (bus == 1)
      cap = getReadingFromFDCwithAddressAndBus(Wire1, addr, capdac, i);
    else if (bus == 2)
      cap = getReadingFromFDCwithAddressAndBus(Wire2, addr, capdac, i);

    capValues[i][2] = cap;
    capValues[i][0] = addr;
    capValues[i][1] = 0;
  }

  long flexorSensor = capValues[0][2]; //retrieving capacitor value from the first array
  long fingertipSensor = capValues[1][2];  //retrieving capacitor value from second array
  long extensorSensor = capValues[2][2];

  //mapped variables
  // int flexsorSensorMapped = map(flexorSensor, 15000, 55000, 0, 180); // CALL GLOBAL VARIABLE THAT WAS ASSIGNED
  // int fingertipsensormapped = map(fingertipSensor, 15000, 55000, 0, 180);
  // int extensorSensorMapped = map(extensorSensor, 15000, 55000, 0, 180);
  // int forcesum = flexsorSensorMapped - ((extensorSensorMapped + 15) + fingertipsensormapped);

  //mapped variables with calibration values
  int flexsorSensorMapped = map(flexorSensor, avgFlexsor_min, avgFlexsor_max, 0, 180); // CALL GLOBAL VARIABLE THAT WAS ASSIGNED
  int fingertipsensormapped = map(fingertipSensor, 15000, 55000, 0, 180);
  int extensorSensorMapped = map(extensorSensor, avgExtensor_min, avgExtensor_max, 0, 180);
  int forcesum = flexsorSensorMapped - ((extensorSensorMapped ) + fingertipsensormapped);


  int unmappedpos1 = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    unmappedpos1 = posi1;
  }

  int unmappedpos2 = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    unmappedpos2 = posi2;
  }


  //motor power option 2 (always set at max speed)
  long pwr1 = 255;
  long pwr2 = 255;

  int pos1 = map(unmappedpos1, 0, 2527, 0, 180);

  int dir1 = 0;
  if ((pos1 <= (forcesum + 14)) && (pos1 >= (forcesum - 14)))
  {
    dir1 = 0;
    pwr1 = 0;
  }
  else if (pos1 < forcesum)
  {
    dir1 = -1;
  }
  else if (pos1 > forcesum)
  {
    dir1 = 1;
  }

  int pos2 = map(unmappedpos2, 0, 2527, 0, 180);

  int dir2 = 0;
  if ((pos2 <= (forcesum + 14)) && (pos2 >= (forcesum - 14)))
  {
    dir2 = 0;
    pwr2 = 0;
  }
  else if (pos2 < forcesum)
  {
    dir2 = -1;
  }
  else if (pos2 > forcesum)
  {
    dir2 = 1;
  }

  // signal the motor
  setMotor1(dir1, pwr1, PWM, IN1, IN2);
  setMotor2(dir2, pwr2, PWM1, IN1, IN2);

  uint32_t elapsedTime = millis();
  if ((elapsedTime % 75) == 0)
  {
    Serial.println(" ");
    Serial.print("Time: ");
    Serial.println(millis());
    Serial.print("Flexor Raw: ");
    Serial.println(flexorSensor);
    Serial.print("Extensor Raw: ");
    Serial.println(extensorSensor);
    Serial.print("Fingertip Raw: ");
    Serial.println(fingertipSensor);
    Serial.print("Flexsor Sensor Mapped: ");
    Serial.println(flexsorSensorMapped);
    Serial.print("Extensor Sensor Mapped: ");
    Serial.println(extensorSensorMapped);
    Serial.print("Fingertip Sensor Mapped: ");
    Serial.println(fingertipsensormapped);
    Serial.print("Sum of Forces: ");
    Serial.println(forcesum);
    Serial.print("Bottom Position: ");
    Serial.println(pos1);
    Serial.print("Bottom Direction: ");
    Serial.println(dir1);
    Serial.print("Top Position: ");
    Serial.println(pos2);
    Serial.print("Top Direction: ");
    Serial.println(dir2);
    Serial.println(" ");
   }
}