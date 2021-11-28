#include <Arduino.h>
#include "FDC1004.h"
#include <Wire.h>
#include <util/atomic.h> // For the ATOMIC_BLOCK macro


#define ENCA 21 // YELLOW
#define ENCB 20 // WHITE
#define PWM 4
#define IN2 14
#define IN1 15
#define POT 23

//int led = 13; // assign led pin
// device array to hold all connected i2c devices and analog outputs. board is only capable of 24 I2C devices and 2 analog outputs so we initialize array of 26
// device array indices: [j][0] = I2C address [j][1] = bus [j][2] = capdac 
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

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
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
  // put your setup code here, to run once:
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(POT,INPUT);
  
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

  Wire.setClock(400000);
  Wire1.setClock(400000);

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

    int sensorsBusOne = I2Cscanner(Wire, 0);
    int sensorsBusTwo = I2Cscanner(Wire, 1);
    numberOfSensors = sensorsBusOne + sensorsBusTwo;
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
  // here we should have a device array full of FDC device ID and bus
  // setup complete.
}



void loop() 
{

  
      // data collection part: sample the sensors, and print them in the necessary key val format to the serial port

      for (int i = 0; i < sensorCount; i++)
      {

        int addr = deviceArray[i][0]; // get the I2C address
        int bus = deviceArray[i][1];
        int capdac = deviceArray[i][2]; // get the capdac value

        if (bus ==0)
          configureMeasurementonFDCwithAddressAndBus(Wire, addr, capdac, i);
        else if (bus ==1)
          configureMeasurementonFDCwithAddressAndBus(Wire1, addr, capdac, i);

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


        capValues[i][2] = cap;
        capValues[i][0] = addr;
        capValues[i][1] = 0;


    }
        long forearmSensor = capValues[0][2];
        long fingertipSensor = capValues[1][2];

        
        //motor power
        long sensordiff = abs(forearmSensor-(fingertipSensor+800));
        long pwr = map(sensordiff, 20, 20000, 0 , 255);
        if(pwr>255){
          pwr=255;
        }
       
        // motor direction

        int dir=0;
        if ((forearmSensor <= ( (fingertipSensor+800) + 500)) && (forearmSensor >= ((fingertipSensor+800) - 500))){
            dir=0;
            pwr=0;
        }
        else if (forearmSensor<(fingertipSensor+800)){
            dir=1;
        }
        else if (forearmSensor>( fingertipSensor+800)){
            dir=-1;
        }
      
        // signal the motor
        setMotor(dir,pwr,PWM,IN1,IN2);

        // store previous error
        Serial.print("Forearm: ");
        Serial.print(forearmSensor);
        Serial.print(" ");
        Serial.println();
        Serial.print("Fingertip: ");
        Serial.print(fingertipSensor+800);
        Serial.print(" ");
        Serial.println();
        Serial.print("Power: ");
        Serial.print(pwr);
        Serial.print(" ");
        Serial.println();
        Serial.print("Sensor Diff: ");
        Serial.print(sensordiff);
        Serial.print(" ");
        Serial.println();

}




  