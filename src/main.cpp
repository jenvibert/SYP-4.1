#include <Arduino.h>
#include "FDC1004.h"
#include <Wire.h>
#include <util/atomic.h> // For the ATOMIC_BLOCK macro
#include <ctime>
using namespace std;

#define ENCA 21
#define ENCB 20 
#define PWM 4
#define IN2 14
#define IN1 15

int deviceArray[24][3] = {0}; 
int led = 13; // assign led pin

volatile int posi = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
long prevT = 0;
float eprev = 0;
float eintegral = 0;

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
void configureMeasurementonFDCwithAddressAndBus(TwoWire &bus, int addr, int capdac, int i) // the i here is the sensor index in the array - this is where cap dac values are stored.
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

// method to handle input
bool handleInput()
{
  while (Serial.available() > 0)
  {
    char incomingCharacter = Serial.read();
    switch (incomingCharacter)
    {
    case 'y':
      return true;
      break;

    case 'n':
      return false;
      break;

    default:
      break;
      return false;
    }
  }
  return false;
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){ //setting up motor
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}

void setup() {
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);

  Serial.println("target pos");

  // initialize the LED pin as an output.
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH); // turn the LED on (HIGH is the voltage level) while set up is ongoing

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
    int sensorsBusTwo = I2Cscanner(Wire, 1); //setting up second bus for I2C
    int sensorBusThree = I2Cscanner(Wire, 2);

    numberOfSensors = sensorsBusOne + sensorsBusTwo + sensorBusThree;
    sensorCount = numberOfSensors; // store number of sensors detected in a global variable so you dont have to keep counting number of sensors in main loop (per previous code)
    Serial.println(sensorCount);
    // when at least one sensor is detected, break the loop. If no sensors detected, try to scan again until sensors are detected. (i.e. if unknown error occurs)
    if (numberOfSensors > 0 && numberOfSensors < 25)
      break;
    
  }

  digitalWrite(led, HIGH); // set Led high to show that setup complete
  
  for (int x=0; x<24; x++){
    Serial.print("i2c address: ");
    Serial.println(deviceArray[x][0]);
    Serial.print("bus: ");
    Serial.println(deviceArray[x][1]);
  }

 
}

float avgSensorOutput (int sampleCount){

  float avgFlex_min = 0;
  float avgFlex_max = 0;
  float avgExt_min = 0;
  float avgExt_max = 0;


  for(int i = 0;i<sampleCount;i++){
  Serial.print("Flex");  
  configureMeasurementonFDCwithAddressAndBus(Wire, addr, capdac, i);
  delay(5);
  cap = getReadingFromFDCwithAddressAndBus(Wire, addr, capdac, i);
  avgFlex_min = avgFlex_min+cap;
  }

  for(int i = 0;i<sampleCount;i++){
  Serial.print("Relax");  
  configureMeasurementonFDCwithAddressAndBus(Wire, addr, capdac, i);
  delay(5);
  cap = getReadingFromFDCwithAddressAndBus(Wire, addr, capdac, i);
  avgFlex_max = avgFlex_max+cap;
  }

  for(int i = 0;i<sampleCount;i++){
  Serial.print("Flex");  
  configureMeasurementonFDCwithAddressAndBus(Wire1 addr, capdac, i);
  delay(5);
  cap = getReadingFromFDCwithAddressAndBus(Wire1, addr, capdac, i);
  avgExt_min = avgExt_min+cap;
  }

  for(int i = 0;i<sampleCount;i++){
  Serial.print("Relax");  
  configureMeasurementonFDCwithAddressAndBus(Wire1, addr, capdac, i);
  delay(5);
  cap = getReadingFromFDCwithAddressAndBus(Wire1, addr, capdac, i);
  avgExt_max = avgExt_max+cap;
  }

  avgFlex_min=avgFlex_min/sampleCount;
  avgFlex_max=avgFlex_max/sampleCount;
  avgExt_min=avgExt_min/sampleCount;
  avgExt_max=avgExt_max/sampleCount;

  return avgFlex_min;
  return avgFlex_max;
  return avgExt_min;
  return avgExt_max;
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
          configureMeasurementonFDCwithAddressAndBus(Wire, addr, capdac, i);
        else if (bus == 1)
          configureMeasurementonFDCwithAddressAndBus(Wire1, addr, capdac, i);
        else if (bus == 2)
          configureMeasurementonFDCwithAddressAndBus(Wire2, addr, capdac, i);
      }

      delay(3); // delay 3 ms to let FDC capture data

      for (int i = 0; i < sensorCount; i++)
      {

        int addr = deviceArray[i][0]; // get the I2C address
        int bus = deviceArray[i][1];
        int capdac = deviceArray[i][2]; // get the capdac value

        long cap = 0;
        // configure measurement at specified address and bus from device ID
        if (bus==0)
          cap = getReadingFromFDCwithAddressAndBus(Wire, addr, capdac, i);
        else if (bus ==1)
          cap = getReadingFromFDCwithAddressAndBus(Wire1, addr, capdac, i);
        else if (bus == 2)
          cap=getReadingFromFDCwithAddressAndBus(Wire2, addr, capdac, i);
          
        capValues[i][2] = cap; 
        capValues[i][0] = addr; 
        capValues[i][1] = 0;
    }

        long forearmtopSensor = capValues[0][2]; //retrieving capacitor value from the first array
        long fingertipSensor = capValues[1][2];//retrieving capacitor value from second array
        long forearmbottomSensor = capValues[2][2];

        //motor power option 1 (with variable speed)
        // long sensordiff = abs(forearmSensor-(fingertipSensor+800));
        //long pwr = map(sensordiff, 20, 20000, 0 , 255);
        //if(pwr>255){
        //  pwr=255;
        //}

        
    
        //mapped variables
       int forearmtopsensormapped = map(forearmtopSensor,15000,55000,0,180);
       int fingertipsensormapped = map(fingertipSensor, 15000, 55000, 0, 180);
       int forearmbottomsensormapped = map(forearmbottomSensor,15000,55000,0,180);
       int forcesum = forearmtopsensormapped - ((forearmbottomsensormapped+15) + fingertipsensormapped);

        int unmappedpos = 0; 
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        unmappedpos = posi;
        }
        
        //motor power option 2 (always set at max speed)
         long pwr=255;

         int pos = map(unmappedpos,0,2527,0,180);
        
        int dir=0;
         if ((pos <= (forcesum + 10)) && (pos >= (forcesum - 10))){
          dir=0;
          pwr=0;
          }
        else if (pos<forcesum){
          dir=-1;
        }
        else if (pos>forcesum){
         dir=1;
        }

        // signal the motor
        setMotor(dir,pwr,PWM,IN1,IN2);

        // store previous error
        Serial.print("Time: ");
        Serial.print(millis());
        Serial.println();
        Serial.print("Top of Forearm: ");
        Serial.print(forearmtopsensormapped);
        Serial.print(" ");
        Serial.println();
        Serial.print("Bottom of Forearm: ");
        Serial.print(forearmbottomsensormapped+15);
        Serial.print(" ");
        Serial.println();
        Serial.print("Fingertip: ");
        Serial.print(fingertipsensormapped);
        Serial.print(" ");
        Serial.println();
        Serial.print("Sum of Forces: ");
        Serial.print(forcesum);
        Serial.print(" ");
        Serial.println();
        Serial.print("Position: ");
        Serial.print(pos);
        Serial.print(" ");
        Serial.println();
        Serial.print("Direction: ");
        Serial.print(dir);
        Serial.print(" ");
        Serial.println();
        Serial.print(" ");
        Serial.println();

}




  