#include <Arduino.h>
#include "Wire.h"

#define TCAADDR 0x70
#define TCAADDR1 0x72

void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
  Wire1.beginTransmission(TCAADDR1);
  Wire1.write(1 << i);
  Wire1.endTransmission();
}


// standard Arduino setup()
void setup()
{
    while (!Serial);
    delay(1000);

    Wire.begin();
    Wire1.begin();
    
    Serial.begin(115200);
    Serial.println("\nTCAScanner ready!");
    
    for (uint8_t t=0; t<8; t++) {
      tcaselect(t);
      Serial.print("TCA Port #"); Serial.println(t);

      for (uint8_t addr = 0; addr<=127; addr++) {
        if (addr == TCAADDR) continue;

        Wire.beginTransmission(addr);
        if (!Wire.endTransmission()) {
          Serial.print("Found I2C 0x");  Serial.println(addr,HEX);
        }
      }
      
      for (uint8_t addr = 0; addr<=127; addr++) {
        if (addr == TCAADDR1) continue;

        Wire1.beginTransmission(addr);
        if (!Wire1.endTransmission()) {
          Serial.print("Found I2C 0x");  Serial.println(addr,HEX);
        }

    }
    Serial.println("\ndone");
}