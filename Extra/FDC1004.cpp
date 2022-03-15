// modified from protocentral code to be able to address any translated FDC address on any I2C bus as needed.

/*
usage:

set up I2C bus
make sure to set enable pin to HIGH
initialize FDC object

run configure measurement code with wire and address inputs

run trigger measurement code with wire and address inputs

run read measurement code with wire and address inputs

have an array to store measurement results from each FDC that is present, and store capdac values for each FDC


*/

#include <FDC1004.h>
#include <Wire.h>

#define FDC1004_UPPER_BOUND ((int16_t) 0x4000)
#define FDC1004_LOWER_BOUND (-1 * FDC1004_UPPER_BOUND)

uint8_t MEAS_CONFIG[] = {0x08, 0x09, 0x0A, 0x0B};
uint8_t MEAS_MSB[] = {0x00, 0x02, 0x04, 0x06};
uint8_t MEAS_LSB[] = {0x01, 0x03, 0x05, 0x07};
uint8_t SAMPLE_DELAY[] = {11,11,6,3};

FDC1004::FDC1004(uint16_t rate)
{
  //this->address = 0b1010000;
  this->_rate = rate;
}

void inline FDC1004::write16(uint8_t reg, uint16_t data, TwoWire &I2CBus , uint8_t addr) // modified to give wire and address in arguments, still need to do error checking
{
  I2CBus.beginTransmission(addr);
  I2CBus.write(reg); //send address
  I2CBus.write( (uint8_t) (data >> 8));
  I2CBus.write( (uint8_t) data);
  I2CBus.endTransmission();
}

uint16_t inline FDC1004::read16(TwoWire &I2CBus, uint8_t reg, uint8_t addr) // modified to give wire and address in arguments. still need to do error checking
{
  I2CBus.beginTransmission(addr);
  I2CBus.write(reg);
  I2CBus.endTransmission();
  uint16_t value;
  I2CBus.beginTransmission(addr);
  I2CBus.requestFrom(addr, (uint8_t)2);
  value = I2CBus.read();
  value <<= 8;
  value |= I2CBus.read();
  I2CBus.endTransmission();
  return value;
}

// read device ID for checking if its indeed a sensor
uint16_t FDC1004::getDeviceID(TwoWire &I2CBus, uint8_t addr)
{
  uint16_t devID = read16(I2CBus, (uint8_t)FdcReg::DEVID, addr);
  return devID;
}


//configure a measurement
uint8_t FDC1004::configureMeasurementSingle(uint8_t measurement, uint8_t channel, uint8_t capdac, TwoWire &I2CBus, uint8_t addr)
{
    //Verify data
    if (!FDC1004_IS_MEAS(measurement) || !FDC1004_IS_CHANNEL(channel) || capdac > FDC1004_CAPDAC_MAX) {
        Serial.println("bad configuration");
        return 1;
    }

    //build 16 bit configuration
    uint16_t configuration_data;
    configuration_data = ((uint16_t)channel) << 13; //CHA
    configuration_data |=  ((uint16_t)0x04) << 10; //CHB disable / CAPDAC enable
    configuration_data |= ((uint16_t)capdac) << 5; //CAPDAC value
    write16(MEAS_CONFIG[measurement], configuration_data, I2CBus, addr);
    return 0;
}

uint8_t FDC1004::triggerSingleMeasurement(uint8_t measurement, uint8_t rate, TwoWire &I2CBus, uint8_t addr)
{
  //verify data
    if (!FDC1004_IS_MEAS(measurement) || !FDC1004_IS_RATE(rate)) {
        Serial.println("bad trigger request");
        return 1;
    }
    uint16_t trigger_data;
    trigger_data = ((uint16_t)rate) << 10; // sample rate
    trigger_data |= 0 << 8; //repeat disabled
    trigger_data |= (1 << (7-measurement)); // 0 > bit 7, 1 > bit 6, etc
    write16(FDC_REGISTER, trigger_data, I2CBus, addr);

    return 0;
}

/**
 * Check if measurement is done, and read the measurement into value if so.
  * value should be at least 4 bytes long (24 bit measurement)
 */
uint8_t FDC1004::readMeasurement(uint8_t measurement, uint16_t * value, TwoWire &I2CBus, uint8_t addr)
{
    if (!FDC1004_IS_MEAS(measurement)) {
        Serial.println("bad read request");
        return 1;
    }

    //check if measurement is complete
    uint16_t fdc_register = read16(I2CBus, FDC_REGISTER, addr);
    if (! (fdc_register & ( 1 << (3-measurement)))) {
        Serial.println("measurement not completed");
        return 2;
    }

  //read the value
  uint16_t msb = read16(I2CBus, MEAS_MSB[measurement], addr);
  uint16_t lsb = read16(I2CBus, MEAS_LSB[measurement], addr);
  value[0] = msb;
  value[1] = lsb;
  return 0;
}

/**
 * take a measurement, uses the measurement register equal to the channel number
 */
uint8_t FDC1004::measureChannel(uint8_t channel, uint8_t capdac, uint16_t * value, TwoWire &I2CBus, uint8_t addr)
{
  uint8_t measurement = channel; //4 measurement configs, 4 channels, seems fair
  if (configureMeasurementSingle(measurement, channel, capdac, I2CBus, addr)) return 1;
  if (triggerSingleMeasurement(measurement, this->_rate, I2CBus, addr)) return 1;
  delay(SAMPLE_DELAY[this->_rate]);
  return readMeasurement(measurement, value, I2CBus, addr);
}

/**
 *  function to get the capacitance from a channel.
  */
int32_t FDC1004::getCapacitance(uint8_t channel)
{
    fdc1004_measurement_t value;
    uint8_t result = getRawCapacitance(channel, &value);
    if (result) return 0x80000000;

    int32_t capacitance = ((int32_t)ATTOFARADS_UPPER_WORD) * ((int32_t)value.value); //attofarads
    capacitance /= 1000; //femtofarads
    capacitance += ((int32_t)FEMTOFARADS_CAPDAC) * ((int32_t)value.capdac);
    return capacitance;
}