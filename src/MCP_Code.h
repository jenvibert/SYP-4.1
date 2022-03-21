#include <Adafruit_MCP3008.h>

Adafruit_MCP3008 adc;

int count = 0;
int MCPvalues[16] = {0}; //Array for MCP vals

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("MCP3008 simple test.");

  // Hardware SPI (specify CS, use any available digital)
  // Can use defaults if available, ex: UNO (SS=10) or Huzzah (SS=15)

  adc.begin(10); // CS on Teensy4.1
  adc.begin(38); // CS1 on Teensy4.1 

  // Software SPI (specify all, use any available digital)
  // (sck, mosi, miso, cs);
  //adc.begin(13, 11, 12, 10);
}

void loop() {
   
 
  for (int i = 0; i < 8; i++)
  {  
  MCPvalues[i] = adc.readADC(i);
  Serial.print(adc.readADC(i)); Serial.print("\t")
  }
  for (int i = 0; i < 8; i++)
  {    
  MCPvalues[i+8] = adc.readADC(i);
  Serial.print(adc.readADC(i)); Serial.print("\t")
  }




  delay(1000);



}