#ifndef _FDC1004
#define _FDC1004

#include "Arduino.h"
#include "Wire.h"

//Constants and limits for FDC1004
#define FDC1004_100HZ (0x01)
#define FDC1004_200HZ (0x02)
#define FDC1004_400HZ (0x03)
#define FDC1004_IS_RATE(x) (x == FDC1004_100HZ || \
                            x == FDC1004_200HZ || \
                            x == FDC1004_400HZ)

#define FDC1004_CAPDAC_MAX (0x1F)

#define FDC1004_CHANNEL_MAX (0x03)
#define FDC1004_IS_CHANNEL(x) (x >= 0 && x <= FDC1004_CHANNEL_MAX)

#define FDC1004_MEAS_MAX (0x03)
#define FDC1004_IS_MEAS(x) (x >= 0 && x <= FDC1004_MEAS_MAX)

#define FDC_REGISTER (0x0C)

#define ATTOFARADS_UPPER_WORD (457) //number of attofarads for each 8th most lsb (lsb of the upper 16 bit half-word)
#define FEMTOFARADS_CAPDAC (3028) //number of femtofarads for each lsb of the capdac

/* Register addresses for accessing data from within the FDC chip */
enum class FdcReg : uint8_t
{
  MEAS1_MSB = (0x00),       /* MSB portion of Measurement 1 (rst val: 0x0000) */
  MEAS1_LSB = (0x01),       /* LSB portion of Measurement 1 (rst val: 0x0000) */
  MEAS2_MSB = (0x02),       /* MSB portion of Measurement 2 (rst val: 0x0000) */
  MEAS2_LSB = (0x03),       /* LSB portion of Measurement 2 (rst val: 0x0000) */
  MEAS3_MSB = (0x04),       /* MSB portion of Measurement 3 (rst val: 0x0000) */
  MEAS3_LSB = (0x05),       /* LSB portion of Measurement 3 (rst val: 0x0000) */
  MEAS4_MSB = (0x06),       /* MSB portion of Measurement 4 (rst val: 0x0000) */
  MEAS4_LSB = (0x07),       /* LSB portion of Measurement 4 (rst val: 0x0000) */
  CONF_MEAS1 = (0x08),      /* Measurement 1 Configuration (rst val: 0x1C00) */
  CONF_MEAS2 = (0x09),      /* Measurement 2 Configuration (rst val: 0x1C00) */
  CONF_MEAS3 = (0x0A),      /* Measurement 3 Configuration (rst val: 0x1C00) */
  CONF_MEAS4 = (0x0B),      /* Measurement 4 Configuration (rst val: 0x1C00) */
  FDC_CONF = (0x0C),        /* Capacitance to Digital Configuration (rst val: 0x0000) */
  OFFSET_CAL_CIN1 = (0x0D), /* CIN1 Offset Calibration (rst val: 0x0000) */
  OFFSET_CAL_CIN2 = (0x0E), /* CIN2 Offset Calibration (rst val: 0x0000) */
  OFFSET_CAL_CIN3 = (0x0F), /* CIN3 Offset Calibration (rst val: 0x0000) */
  OFFSET_CAL_CIN4 = (0x10), /* CIN4 Offset Calibration (rst val: 0x0000) */
  GAIN_CAL_CIN1 = (0x11),   /* CIN1 Gain Calibration (rst val: 0x4000) */
  GAIN_CAL_CIN2 = (0x12),   /* CIN2 Gain Calibration (rst val: 0x4000) */
  GAIN_CAL_CIN3 = (0x13),   /* CIN3 Gain Calibration (rst val: 0x4000) */
  GAIN_CAL_CIN4 = (0x14),   /* CIN4 Gain Calibration (rst val: 0x4000) */
  MFGID = (0xFE),           /* ID of Texas Instruments (rst val: 0x5449) */
  DEVID = (0xFF),           /* ID of FDC1004 device (rst val: 0x1004) */
};

/********************************************************************************************************
 * typedefs
 *******************************************************************************************************/
typedef struct fdc1004_measurement_t
{
    int16_t value;
    uint8_t capdac;
}fdc1004_measurement_t;

/******************************************************************************************
 * Function Declarations
 ******************************************************************************************/
class FDC1004
{
 public:
    FDC1004(uint16_t rate = FDC1004_100HZ);
    int32_t getCapacitance(uint8_t channel = 1);
    uint8_t getRawCapacitance(uint8_t channel, fdc1004_measurement_t * value);
    uint8_t configureMeasurementSingle(uint8_t measurement, uint8_t channel, uint8_t capdac, TwoWire &I2CBus, uint8_t addr);
    uint8_t triggerSingleMeasurement(uint8_t measurement, uint8_t rate, TwoWire &I2CBus, uint8_t addr);
    uint8_t readMeasurement(uint8_t measurement, uint16_t * value, TwoWire &I2CBus, uint8_t addr);
    uint8_t measureChannel(uint8_t channel, uint8_t capdac, uint16_t * value, TwoWire &I2CBus, uint8_t addr);
	uint16_t inline read16(TwoWire &I2CBus, uint8_t reg, uint8_t addr);
    uint16_t getDeviceID(TwoWire &I2CBus, uint8_t addr);

 private:
    uint8_t _rate;
    uint8_t _last_capdac[4];
    void inline write16(uint8_t reg, uint16_t data, TwoWire &I2CBus, uint8_t addr);

};

#endif