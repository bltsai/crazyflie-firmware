/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2012 BitCraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * vl53l0x.c: Time-of-flight distance sensor driver
 */

#define DEBUG_MODULE "VLX"
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"

#include "deck.h"
#include "system.h"
#include "debug.h"
#include "log.h"
#include "param.h"

#include "i2cdev.h"
#include "vl53l0x.h"

#include "stabilizer_types.h"

#include "estimator.h"
#include "estimator_kalman.h"
#include "arm_math.h"

// Measurement noise model
static float expPointA = 1.0f;
static float expStdA = 0.0025f; // STD at elevation expPointA [m]
static float expPointB = 1.3f;
static float expStdB = 0.2f;    // STD at elevation expPointB [m]
static float expCoeff;

#define RANGE_OUTLIER_LIMIT   1800 // the measured range is in [mm]
#define CORRIDOR_HEIGHT       2650

#define RAW_DATA_FRAME_NUM    5
#define BG_FRAME_NUM          20
#define PEAK_THREHOLD         1.5f

#define LEFT_UP      7
#define RIGHT_UP     63
#define LEFT_DOWN    0
#define RIGHT_DOWN   56

#define MAXRANGE 80

static uint8_t devAddr;
static I2C_Dev *I2Cx;
static bool isInit;

static uint16_t io_timeout = 0;
static bool did_timeout;
static uint16_t timeout_start_ms;

uint16_t range_last = 0;
uint16_t range_last_down = 0;
uint16_t range_last_front = 0;
// uint16_t range_last_top = 0;

// Record the current time to check an upcoming timeout against
#define startTimeout() (timeout_start_ms = xTaskGetTickCount())

// Check if timeout is enabled (set to nonzero value) and has expired
#define checkTimeoutExpired() (io_timeout > 0 && ((uint16_t)xTaskGetTickCount() - timeout_start_ms) > io_timeout)

// Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs
// from register value
// based on VL53L0X_decode_vcsel_period()
#define decodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)

// Encode VCSEL pulse period register value from period in PCLKs
// based on VL53L0X_encode_vcsel_period()
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)

// Calculate macro period in *nanoseconds* from VCSEL period in PCLKs
// based on VL53L0X_calc_macro_period_ps()
// PLL_period_ps = 1655; macro_period_vclks = 2304
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

// read by init and used when starting measurement;
// is StopVariable field of VL53L0X_DevData_t structure in API
static uint8_t stop_variable;

static uint32_t measurement_timing_budget_us;
static uint16_t measurement_timing_budget_ms;

// Get reference SPAD (single photon avalanche diode) count and type
// based on VL53L0X_get_info_from_device(),
// but only gets reference SPAD count and type
static bool vl53l0xGetSpadInfo(uint8_t * count, bool * type_is_aperture);

// Get sequence step enables
// based on VL53L0X_GetSequenceStepEnables()
static void vl53l0xGetSequenceStepEnables(SequenceStepEnables * enables);

// Get sequence step timeouts
// based on get_sequence_step_timeout(),
// but gets all timeouts instead of just the requested one, and also stores
// intermediate values
static void vl53l0xGetSequenceStepTimeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts);

// based on VL53L0X_perform_single_ref_calibration()
static bool vl53l0xPerformSingleRefCalibration(uint8_t vhv_init_byte);

// Decode sequence step timeout in MCLKs from register value
// based on VL53L0X_decode_timeout()
// Note: the original function returned a uint32_t, but the return value is
// always stored in a uint16_t.
static uint16_t vl53l0xDecodeTimeout(uint16_t reg_val);

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L0X_encode_timeout()
// Note: the original function took a uint16_t, but the argument passed to it
// is always a uint16_t.
static uint16_t vl53l0xEncodeTimeout(uint16_t timeout_mclks);

// Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_us()
static uint32_t vl53l0xTimeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);

// Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_mclks()
static uint32_t vl53l0xTimeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);

static uint16_t vl53l0xReadReg16Bit(uint8_t reg);
static bool vl53l0xWriteReg16Bit(uint8_t reg, uint16_t val);
static bool vl53l0xWriteReg32Bit(uint8_t reg, uint32_t val);

static float roomTemp = 0;
static float roll = 0;
static float pitch = 0;

static uint32_t yellowGroupH = 0;
static uint32_t yellowGroupL = 0;
static uint32_t orangeGroupH = 0;
static uint32_t orangeGroupL = 0;


static uint8_t  rawDataFrameIndex = 0;
static float    rawData[64*RAW_DATA_FRAME_NUM] = {0};
static uint8_t  bgFrameIndex = 0;
static float    bgData[64*BG_FRAME_NUM] = {0};

static bool isCalibrate = false;
static float avgCal[64] = {0};
static float stdCal[64] = {0};
static float zscoreData[64*RAW_DATA_FRAME_NUM] = {0};
static float zscore[64] = {0.0f};
static float x_weight_coordinate = 0.0f;
static float y_weight_coordinate = 0.0f;
static int16_t z_total_heat = 0;
static int16_t zscore_int16_t[64] = {0};
static int16_t z_max = 0;
static int16_t z_min = 0;
static bool zscoreWeight[64] = {false};

static float zscore_threshold_high = 10.0f;
static float zscore_threshold_low = 5.0f;

static void rotateColor(uint8_t* color, uint8_t* rColor);
static void labelPixel(int* largestSubset ,int maxSubsetLen, uint8_t* color);
static void findGroup(uint8_t* color);
static bool check_p2p();
static void calibration();
static void zscoreCalculation();
static void get_neighbor(int loc, int* neighbor);
static bool label_neighbor(int result[], int subsetNumber);
static void label_subset(int testset[], int testsetLen, int result[], int subsetNumber);
static bool get_startIndex(int testset[], int testsetLen, int result[], int* startIndex);
static void select_largest_subset(int testset[], int testsetLen, int result[], int subsetNumber, int* maxSubsetLen, int* largestSubset);
static void find_largestSubset(int testset[], int testsetLen, int* maxSubsetLen, int* largestSubset);
static void get_largest_subset(int* largestSubset, int* maxSubsetLen);
// static void rotateXY();
static void get_xy(int* largestSubset, int maxSubsetLen);
static void get_total_heat(int* largestSubset, int maxSubsetLen);

/** Default constructor, uses default I2C address.
 * @see VL53L0X_DEFAULT_ADDRESS
 */

void vl53l0xInit(DeckInfo* info)
{
  if (isInit)
    return;

  i2cdevInit(I2C1_DEV);
  I2Cx = I2C1_DEV;
  devAddr = VL53L0X_DEFAULT_ADDRESS;
  i2cdevWriteByte(I2Cx, TCAADDR, 0x80, 0x80);
  xTaskCreate(vl53l0xTask, VL53_TASK_NAME, VL53_TASK_STACKSIZE, NULL, VL53_TASK_PRI, NULL);

  // pre-compute constant in the measurement noise model for kalman
  expCoeff = logf(expStdB / expStdA) / (expPointB - expPointA);

  isInit = true;
}

bool vl53l0xTest(void)
{
  bool testStatus;

  if (!isInit)
    return false;
       // Measurement noise model
  int addresses[] = {0x80, 0x10, 0x04};
  for (int i=0; i<2; i++){
    i2cdevWriteByte(I2Cx, TCAADDR, addresses[i], addresses[i]);
    testStatus  = vl53l0xTestConnection();
    testStatus &= vl53l0xInitSensor(true);
  }

  return testStatus;
}

void vl53l0xTask(void* arg)
{
  systemWaitStart();
  TickType_t xLastWakeTime;
  int addresses[] = {0x80, 0x10, 0x04}; // 0x80 down, 0x10 front, 0x04 top
  for (int i=0; i<2; i++){
    i2cdevWriteByte(I2Cx, TCAADDR, addresses[i], addresses[i]);
    vl53l0xSetVcselPulsePeriod(VcselPeriodPreRange, 18);
    vl53l0xSetVcselPulsePeriod(VcselPeriodFinalRange, 14);
    vl53l0xStartContinuous(0);
  }

  while (1) {
    xLastWakeTime = xTaskGetTickCount();

    i2cdevWriteByte(I2Cx, TCAADDR, 0x80, 0x80);
    range_last_down = vl53l0xReadRangeContinuousMillimeters();
    i2cdevWriteByte(I2Cx, TCAADDR, 0x10, 0x10);
    range_last_front = vl53l0xReadRangeContinuousMillimeters();
    // i2cdevWriteByte(I2Cx, TCAADDR, 0x04, 0x04);
    // range_last_top = vl53l0xReadRangeContinuousMillimeters();

    if (range_last_down < RANGE_OUTLIER_LIMIT) {
      range_last = range_last_down;
    // } else {
    //   range_last = CORRIDOR_HEIGHT - range_last_top;
    }

    // check if range is feasible and push into the kalman filter
    // the sensor should not be able to measure >3 [m], and outliers typically
    // occur as >8 [m] measurements
    if (getStateEstimator() == kalmanEstimator &&
        range_last < RANGE_OUTLIER_LIMIT) {
      // Form measurement
      tofMeasurement_t tofData;
      tofData.timestamp = xTaskGetTickCount();
      tofData.distance = (float)range_last * 0.001f; // Scale from [mm] to [m]
      tofData.stdDev = expStdA * (1.0f  + expf( expCoeff * ( tofData.distance - expPointA)));
      estimatorKalmanEnqueueTOF(&tofData);
    }

    uint8_t data[128];
    i2cdevWriteByte(I2Cx, TCAADDR, 0x08, 0x08);
    i2cdevRead(I2Cx, GRIDEYE_DEFAULT_ADDRESS, PIXEL0 | GRIDEYE_ADDR_AUTO_INC, 128, data);

    if (!isCalibrate) {
        for(int i=(64*bgFrameIndex);i<(64*(bgFrameIndex+1));i++){
            int map_index = i - (64*bgFrameIndex);
            bgData[i] = ((data[map_index*2+1] << 8) | data[map_index*2]) * 0.25f;
        }
        bgFrameIndex += 1;

        if (bgFrameIndex == BG_FRAME_NUM) {
          calibration();
          if (check_p2p()) {
            isCalibrate = true;
          } else {
            bgFrameIndex = 0;
          }
        }

    } else {
        rawDataFrameIndex %= RAW_DATA_FRAME_NUM;
        for(int i=(64*rawDataFrameIndex);i<(64*(rawDataFrameIndex+1));i++){
            int map_index = i - (64*rawDataFrameIndex);
            rawData[i] = ((data[map_index*2+1] << 8) | data[map_index*2]) * 0.25f;
        }
        zscoreCalculation();
        rawDataFrameIndex += 1;

        uint8_t color[64]={0};
        uint8_t rColor[64]={0};
        int largestSubset[64] = {-1};
        int maxSubsetLen = 0;
        get_largest_subset(largestSubset, &maxSubsetLen);
        get_xy(largestSubset, maxSubsetLen);
        get_total_heat(largestSubset, maxSubsetLen);
        //rotateXY();
        labelPixel(largestSubset, maxSubsetLen, color);
        rotateColor(color, rColor);
        findGroup(rColor);

    }


    vTaskDelayUntil(&xLastWakeTime, M2T(measurement_timing_budget_ms));
  }
}

bool vl53l0xReadRange(zDistance_t* zrange, const uint32_t tick)
{
  bool updated = false;

  if (isInit) {
    if (range_last != 0 && range_last < RANGE_OUTLIER_LIMIT) {
      zrange->distance = (float)range_last * 0.001f; // Scale from [mm] to [m]
      zrange->timestamp = tick;
      updated = true;
    }
  }
  return updated;
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool vl53l0xTestConnection()
{
  bool ret = true;
  ret &= vl53l0xGetModelID() == VL53L0X_IDENTIFICATION_MODEL_ID;
  ret &= vl53l0xGetRevisionID() == VL53L0X_IDENTIFICATION_REVISION_ID;
  return ret;
}

/** Get Model ID.
 * This register is used to verify the model number of the device,
 * but only before it has been configured to run
 * @return Model ID
 * @see VL53L0X_RA_IDENTIFICATION_MODEL_ID
 * @see VL53L0X_IDENTIFICATION_MODEL_ID
 */
uint16_t vl53l0xGetModelID()
{
  return vl53l0xReadReg16Bit(VL53L0X_RA_IDENTIFICATION_MODEL_ID);
}

/** Get Revision ID.
 * This register is used to verify the revision number of the device,
 * but only before it has been configured to run
 * @return Revision ID
 * @see VL53L0X_RA_IDENTIFICATION_REVISION_ID
 * @see VL53L0X_IDENTIFICATION_REVISION_ID
 */
uint8_t vl53l0xGetRevisionID()
{
  uint8_t output = 0;
  i2cdevReadByte(I2Cx, devAddr, VL53L0X_RA_IDENTIFICATION_REVISION_ID, &output);
  return output;
}

// Initialize sensor using sequence based on VL53L0X_DataInit(),
// VL53L0X_StaticInit(), and VL53L0X_PerformRefCalibration().
// This function does not perform reference SPAD calibration
// (VL53L0X_PerformRefSpadManagement()), since the API user manual says that it
// is performed by ST on the bare modules; it seems like that should work well
// enough unless a cover glass is added.
// If io_2v8 (optional) is true or not given, the sensor is configured for 2V8
// mode.
bool vl53l0xInitSensor(bool io_2v8)
{
  uint8_t temp;
  // VL53L0X_DataInit() begin

  // sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
  if (io_2v8)
  {
    i2cdevWriteBit(I2Cx, devAddr, VL53L0X_RA_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, 0, 0x01);
  }

  // "Set I2C standard mode"
  i2cdevWriteByte(I2Cx, devAddr, 0x88, 0x00);

  i2cdevWriteByte(I2Cx, devAddr, 0x80, 0x01);
  i2cdevWriteByte(I2Cx, devAddr, 0xFF, 0x01);
  i2cdevWriteByte(I2Cx, devAddr, 0x00, 0x00);
  i2cdevReadByte(I2Cx, devAddr, 0x91, &stop_variable);
  i2cdevWriteByte(I2Cx, devAddr, 0x00, 0x01);
  i2cdevWriteByte(I2Cx, devAddr, 0xFF, 0x00);
  i2cdevWriteByte(I2Cx, devAddr, 0x80, 0x00);

  // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
  i2cdevReadByte(I2Cx, devAddr, VL53L0X_RA_MSRC_CONFIG_CONTROL, &temp);
  i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_MSRC_CONFIG_CONTROL, temp | 0x12);

  // set final range signal rate limit to 0.25 MCPS (million counts per second)
  vl53l0xSetSignalRateLimit(0.25);

  i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_SYSTEM_SEQUENCE_CONFIG, 0xFF);

  // VL53L0X_DataInit() end

  // VL53L0X_StaticInit() begin

  uint8_t spad_count;
  bool spad_type_is_aperture;
  if (!vl53l0xGetSpadInfo(&spad_count, &spad_type_is_aperture)) { return false; }

  // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
  // the API, but the same data seems to be more easily readable from
  // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
  uint8_t ref_spad_map[6];
  i2cdevRead(I2Cx, devAddr, VL53L0X_RA_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, 6, ref_spad_map);

  // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

  i2cdevWriteByte(I2Cx, devAddr, 0xFF, 0x01);
  i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
  i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
  i2cdevWriteByte(I2Cx, devAddr, 0xFF, 0x00);
  i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

  uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
  uint8_t spads_enabled = 0;

  for (uint8_t i = 0; i < 48; i++)
  {
    if (i < first_spad_to_enable || spads_enabled == spad_count)
    {
      // This bit is lower than the first one that should be enabled, or
      // (reference_spad_count) bits have already been enabled, so zero this bit
      ref_spad_map[i / 8] &= ~(1 << (i % 8));
    }
    else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
    {
      spads_enabled++;
    }
  }

  i2cdevWrite(I2Cx, devAddr, VL53L0X_RA_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, 6, ref_spad_map);

  // -- VL53L0X_set_reference_spads() end

  // -- VL53L0X_load_tuning_settings() begin
  // DefaultTuningSettings from vl53l0x_tuning.h

  i2cdevWriteByte(I2Cx, devAddr, 0xFF, 0x01);
  i2cdevWriteByte(I2Cx, devAddr, 0x00, 0x00);

  i2cdevWriteByte(I2Cx, devAddr, 0xFF, 0x00);
  i2cdevWriteByte(I2Cx, devAddr, 0x09, 0x00);
  i2cdevWriteByte(I2Cx, devAddr, 0x10, 0x00);
  i2cdevWriteByte(I2Cx, devAddr, 0x11, 0x00);

  i2cdevWriteByte(I2Cx, devAddr, 0x24, 0x01);
  i2cdevWriteByte(I2Cx, devAddr, 0x25, 0xFF);
  i2cdevWriteByte(I2Cx, devAddr, 0x75, 0x00);

  i2cdevWriteByte(I2Cx, devAddr, 0xFF, 0x01);
  i2cdevWriteByte(I2Cx, devAddr, 0x4E, 0x2C);
  i2cdevWriteByte(I2Cx, devAddr, 0x48, 0x00);
  i2cdevWriteByte(I2Cx, devAddr, 0x30, 0x20);

  i2cdevWriteByte(I2Cx, devAddr, 0xFF, 0x00);
  i2cdevWriteByte(I2Cx, devAddr, 0x30, 0x09);
  i2cdevWriteByte(I2Cx, devAddr, 0x54, 0x00);
  i2cdevWriteByte(I2Cx, devAddr, 0x31, 0x04);
  i2cdevWriteByte(I2Cx, devAddr, 0x32, 0x03);
  i2cdevWriteByte(I2Cx, devAddr, 0x40, 0x83);
  i2cdevWriteByte(I2Cx, devAddr, 0x46, 0x25);
  i2cdevWriteByte(I2Cx, devAddr, 0x60, 0x00);
  i2cdevWriteByte(I2Cx, devAddr, 0x27, 0x00);
  i2cdevWriteByte(I2Cx, devAddr, 0x50, 0x06);
  i2cdevWriteByte(I2Cx, devAddr, 0x51, 0x00);
  i2cdevWriteByte(I2Cx, devAddr, 0x52, 0x96);
  i2cdevWriteByte(I2Cx, devAddr, 0x56, 0x08);
  i2cdevWriteByte(I2Cx, devAddr, 0x57, 0x30);
  i2cdevWriteByte(I2Cx, devAddr, 0x61, 0x00);
  i2cdevWriteByte(I2Cx, devAddr, 0x62, 0x00);
  i2cdevWriteByte(I2Cx, devAddr, 0x64, 0x00);
  i2cdevWriteByte(I2Cx, devAddr, 0x65, 0x00);
  i2cdevWriteByte(I2Cx, devAddr, 0x66, 0xA0);

  i2cdevWriteByte(I2Cx, devAddr, 0xFF, 0x01);
  i2cdevWriteByte(I2Cx, devAddr, 0x22, 0x32);
  i2cdevWriteByte(I2Cx, devAddr, 0x47, 0x14);
  i2cdevWriteByte(I2Cx, devAddr, 0x49, 0xFF);
  i2cdevWriteByte(I2Cx, devAddr, 0x4A, 0x00);

  i2cdevWriteByte(I2Cx, devAddr, 0xFF, 0x00);
  i2cdevWriteByte(I2Cx, devAddr, 0x7A, 0x0A);
  i2cdevWriteByte(I2Cx, devAddr, 0x7B, 0x00);
  i2cdevWriteByte(I2Cx, devAddr, 0x78, 0x21);

  i2cdevWriteByte(I2Cx, devAddr, 0xFF, 0x01);
  i2cdevWriteByte(I2Cx, devAddr, 0x23, 0x34);
  i2cdevWriteByte(I2Cx, devAddr, 0x42, 0x00);
  i2cdevWriteByte(I2Cx, devAddr, 0x44, 0xFF);
  i2cdevWriteByte(I2Cx, devAddr, 0x45, 0x26);
  i2cdevWriteByte(I2Cx, devAddr, 0x46, 0x05);
  i2cdevWriteByte(I2Cx, devAddr, 0x40, 0x40);
  i2cdevWriteByte(I2Cx, devAddr, 0x0E, 0x06);
  i2cdevWriteByte(I2Cx, devAddr, 0x20, 0x1A);
  i2cdevWriteByte(I2Cx, devAddr, 0x43, 0x40);

  i2cdevWriteByte(I2Cx, devAddr, 0xFF, 0x00);
  i2cdevWriteByte(I2Cx, devAddr, 0x34, 0x03);
  i2cdevWriteByte(I2Cx, devAddr, 0x35, 0x44);

  i2cdevWriteByte(I2Cx, devAddr, 0xFF, 0x01);
  i2cdevWriteByte(I2Cx, devAddr, 0x31, 0x04);
  i2cdevWriteByte(I2Cx, devAddr, 0x4B, 0x09);
  i2cdevWriteByte(I2Cx, devAddr, 0x4C, 0x05);
  i2cdevWriteByte(I2Cx, devAddr, 0x4D, 0x04);

  i2cdevWriteByte(I2Cx, devAddr, 0xFF, 0x00);
  i2cdevWriteByte(I2Cx, devAddr, 0x44, 0x00);
  i2cdevWriteByte(I2Cx, devAddr, 0x45, 0x20);
  i2cdevWriteByte(I2Cx, devAddr, 0x47, 0x08);
  i2cdevWriteByte(I2Cx, devAddr, 0x48, 0x28);
  i2cdevWriteByte(I2Cx, devAddr, 0x67, 0x00);
  i2cdevWriteByte(I2Cx, devAddr, 0x70, 0x04);
  i2cdevWriteByte(I2Cx, devAddr, 0x71, 0x01);
  i2cdevWriteByte(I2Cx, devAddr, 0x72, 0xFE);
  i2cdevWriteByte(I2Cx, devAddr, 0x76, 0x00);
  i2cdevWriteByte(I2Cx, devAddr, 0x77, 0x00);

  i2cdevWriteByte(I2Cx, devAddr, 0xFF, 0x01);
  i2cdevWriteByte(I2Cx, devAddr, 0x0D, 0x01);

  i2cdevWriteByte(I2Cx, devAddr, 0xFF, 0x00);
  i2cdevWriteByte(I2Cx, devAddr, 0x80, 0x01);
  i2cdevWriteByte(I2Cx, devAddr, 0x01, 0xF8);

  i2cdevWriteByte(I2Cx, devAddr, 0xFF, 0x01);
  i2cdevWriteByte(I2Cx, devAddr, 0x8E, 0x01);
  i2cdevWriteByte(I2Cx, devAddr, 0x00, 0x01);
  i2cdevWriteByte(I2Cx, devAddr, 0xFF, 0x00);
  i2cdevWriteByte(I2Cx, devAddr, 0x80, 0x00);

  // -- VL53L0X_load_tuning_settings() end

  // "Set interrupt config to new sample ready"
  // -- VL53L0X_SetGpioConfig() begin

  i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
  i2cdevWriteBit(I2Cx, devAddr, VL53L0X_RA_GPIO_HV_MUX_ACTIVE_HIGH, 4, 0); // active low
  i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_SYSTEM_INTERRUPT_CLEAR, 0x01);

  // -- VL53L0X_SetGpioConfig() end

  measurement_timing_budget_us = vl53l0xGetMeasurementTimingBudget();
  measurement_timing_budget_ms = (uint16_t)(measurement_timing_budget_us / 1000.0f);

  // "Disable MSRC and TCC by default"
  // MSRC = Minimum Signal Rate Check
  // TCC = Target CentreCheck
  // -- VL53L0X_SetSequenceStepEnable() begin

  i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_SYSTEM_SEQUENCE_CONFIG, 0xE8);

  // -- VL53L0X_SetSequenceStepEnable() end

  // "Recalculate timing budget"
  vl53l0xSetMeasurementTimingBudget(measurement_timing_budget_us);

  // VL53L0X_StaticInit() end

  // VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

  // -- VL53L0X_perform_vhv_calibration() begin

  i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_SYSTEM_SEQUENCE_CONFIG, 0x01);
  if (!vl53l0xPerformSingleRefCalibration(0x40)) { DEBUG_PRINT("Failed VHV calibration\n"); return false; }

  // -- VL53L0X_perform_vhv_calibration() end

  // -- VL53L0X_perform_phase_calibration() begin

  i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_SYSTEM_SEQUENCE_CONFIG, 0x02);
  if (!vl53l0xPerformSingleRefCalibration(0x00)) { DEBUG_PRINT("Failed phase calibration\n"); return false; }

  // -- VL53L0X_perform_phase_calibration() end

  // "restore the previous Sequence Config"
  i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_SYSTEM_SEQUENCE_CONFIG, 0xE8);

  // VL53L0X_PerformRefCalibration() end

  return true;
}

// Set the return signal rate limit check value in units of MCPS (mega counts
// per second). "This represents the amplitude of the signal reflected from the
// target and detected by the device"; setting this limit presumably determines
// the minimum measurement necessary for the sensor to report a valid reading.
// Setting a lower limit increases the potential range of the sensor but also
// seems to increase the likelihood of getting an inaccurate reading because of
// unwanted reflections from objects other than the intended target.
// Defaults to 0.25 MCPS as initialized by the ST API and this library.
bool vl53l0xSetSignalRateLimit(float limit_Mcps)
{
  if (limit_Mcps < 0 || limit_Mcps > 511.99f) { return false; }

  // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
  uint16_t fixed_pt = limit_Mcps * (1 << 7);
  return vl53l0xWriteReg16Bit(VL53L0X_RA_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, fixed_pt);
}

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement; the ST API and this library take care of splitting the
// timing budget among the sub-steps in the ranging sequence. A longer timing
// budget allows for more accurate measurements. Increasing the budget by a
// factor of N decreases the range measurement standard deviation by a factor of
// sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
// based on VL53L0X_set_measurement_timing_budget_micro_seconds()
bool vl53l0xSetMeasurementTimingBudget(uint32_t budget_us)
{
  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead      = 1320; // note that this is different than the value in get_
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  uint32_t const MinTimingBudget = 20000;

  if (budget_us < MinTimingBudget) { return false; }

  uint32_t used_budget_us = StartOverhead + EndOverhead;

  vl53l0xGetSequenceStepEnables(&enables);
  vl53l0xGetSequenceStepTimeouts(&enables, &timeouts);

  if (enables.tcc)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables.dss)
  {
    used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables.msrc)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables.pre_range)
  {
    used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables.final_range)
  {
    used_budget_us += FinalRangeOverhead;

    // "Note that the final range timeout is determined by the timing
    // budget and the sum of all other timeouts within the sequence.
    // If there is no room for the final range timeout, then an error
    // will be set. Otherwise the remaining time will be applied to
    // the final range."

    if (used_budget_us > budget_us)
    {
      // "Requested timeout too big."
      return false;
    }

    uint32_t final_range_timeout_us = budget_us - used_budget_us;

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do this both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    uint16_t final_range_timeout_mclks =
      vl53l0xTimeoutMicrosecondsToMclks(final_range_timeout_us,
                                 timeouts.final_range_vcsel_period_pclks);

    if (enables.pre_range)
    {
      final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    uint16_t temp = vl53l0xEncodeTimeout(final_range_timeout_mclks);
    vl53l0xWriteReg16Bit(VL53L0X_RA_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, temp);

    // set_sequence_step_timeout() end

    measurement_timing_budget_us = budget_us; // store for internal reuse
    measurement_timing_budget_ms = (uint16_t)(measurement_timing_budget_us / 1000.0f);
  }
  return true;
}

// Get the measurement timing budget in microseconds
// based on VL53L0X_get_measurement_timing_budget_micro_seconds()
// in us
uint32_t vl53l0xGetMeasurementTimingBudget(void)
{
  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead     = 1910; // note that this is different than the value in set_
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  // "Start and end overhead times always present"
  uint32_t budget_us = StartOverhead + EndOverhead;

  vl53l0xGetSequenceStepEnables(&enables);
  vl53l0xGetSequenceStepTimeouts(&enables, &timeouts);

  if (enables.tcc)
  {
    budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables.dss)
  {
    budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables.msrc)
  {
    budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables.pre_range)
  {
    budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables.final_range)
  {
    budget_us += (timeouts.final_range_us + FinalRangeOverhead);
  }

  measurement_timing_budget_us = budget_us; // store for internal reuse
  measurement_timing_budget_ms = (uint16_t)(measurement_timing_budget_us / 1000.0f);
  return budget_us;
}


// Set the VCSEL (vertical cavity surface emitting laser) pulse period for the
// given period type (pre-range or final range) to the given value in PCLKs.
// Longer periods seem to increase the potential range of the sensor.
// Valid values are (even numbers only):
//  pre:  12 to 18 (initialized default: 14)
//  final: 8 to 14 (initialized default: 10)
// based on VL53L0X_set_vcsel_pulse_period()
bool vl53l0xSetVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks)
{
  uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);

  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  vl53l0xGetSequenceStepEnables(&enables);
  vl53l0xGetSequenceStepTimeouts(&enables, &timeouts);

  // "Apply specific settings for the requested clock period"
  // "Re-calculate and apply timeouts, in macro periods"

  // "When the VCSEL period for the pre or final range is changed,
  // the corresponding timeout must be read from the device using
  // the current VCSEL period, then the new VCSEL period can be
  // applied. The timeout then must be written back to the device
  // using the new VCSEL period.
  //
  // For the MSRC timeout, the same applies - this timeout being
  // dependant on the pre-range vcsel period."


  if (type == VcselPeriodPreRange)
  {
    // "Set phase check limits"
    switch (period_pclks)
    {
      case 12:
        i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
        break;

      case 14:
        i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
        break;

      case 16:
        i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
        break;

      case 18:
        i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
        break;

      default:
        // invalid period
        return false;
    }
    i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);

    // apply new VCSEL period
    i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

    // update timeouts

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)

    uint16_t new_pre_range_timeout_mclks =
      vl53l0xTimeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);

    uint16_t new_pre_range_timeout_encoded = vl53l0xEncodeTimeout(new_pre_range_timeout_mclks);
    vl53l0xWriteReg16Bit(VL53L0X_RA_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, new_pre_range_timeout_encoded);

    // set_sequence_step_timeout() end

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)

    uint16_t new_msrc_timeout_mclks =
      vl53l0xTimeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);

    i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_MSRC_CONFIG_TIMEOUT_MACROP,
      (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));

    // set_sequence_step_timeout() end
  }
  else if (type == VcselPeriodFinalRange)
  {
    switch (period_pclks)
    {
      case 8:
        i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
        i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
        i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
        i2cdevWriteByte(I2Cx, devAddr, 0xFF, 0x01);
        i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_ALGO_PHASECAL_LIM, 0x30);
        i2cdevWriteByte(I2Cx, devAddr, 0xFF, 0x00);
        break;

      case 10:
        i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
        i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
        i2cdevWriteByte(I2Cx, devAddr, 0xFF, 0x01);
        i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_ALGO_PHASECAL_LIM, 0x20);
        i2cdevWriteByte(I2Cx, devAddr, 0xFF, 0x00);
        break;

      case 12:
        i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
        i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
        i2cdevWriteByte(I2Cx, devAddr, 0xFF, 0x01);
        i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_ALGO_PHASECAL_LIM, 0x20);
        i2cdevWriteByte(I2Cx, devAddr, 0xFF, 0x00);
        break;

      case 14:
        i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
        i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
        i2cdevWriteByte(I2Cx, devAddr, 0xFF, 0x01);
        i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_ALGO_PHASECAL_LIM, 0x20);
        i2cdevWriteByte(I2Cx, devAddr, 0xFF, 0x00);
        break;

      default:
        // invalid period
        return false;
    }

    // apply new VCSEL period
    i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

    // update timeouts

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do this both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    uint16_t new_final_range_timeout_mclks =
      vl53l0xTimeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);

    if (enables.pre_range)
    {
      new_final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    uint16_t new_final_range_timeout_encoded = vl53l0xEncodeTimeout(new_final_range_timeout_mclks);
    vl53l0xWriteReg16Bit(VL53L0X_RA_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, new_final_range_timeout_encoded);

    // set_sequence_step_timeout end
  }
  else
  {
    // invalid type
    return false;
  }

  // "Finally, the timing budget must be re-applied"

  vl53l0xSetMeasurementTimingBudget(measurement_timing_budget_us);

  // "Perform the phase calibration. This is needed after changing on vcsel period."
  // VL53L0X_perform_phase_calibration() begin

  uint8_t sequence_config = 0;
  i2cdevReadByte(I2Cx, devAddr, VL53L0X_RA_SYSTEM_SEQUENCE_CONFIG, &sequence_config);
  i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_SYSTEM_SEQUENCE_CONFIG, 0x02);
  bool ret = vl53l0xPerformSingleRefCalibration(0x0);
  i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_SYSTEM_SEQUENCE_CONFIG, sequence_config);

  // VL53L0X_perform_phase_calibration() end

  return ret;
}

// Get the VCSEL pulse period in PCLKs for the given period type.
// based on VL53L0X_get_vcsel_pulse_period()
uint8_t vl53l0xGetVcselPulsePeriod(vcselPeriodType type)
{
  if (type == VcselPeriodPreRange)
  {
    uint8_t temp = 0;
    i2cdevReadByte(I2Cx, devAddr, VL53L0X_RA_PRE_RANGE_CONFIG_VCSEL_PERIOD, &temp);
    return decodeVcselPeriod(temp);
  }
  else if (type == VcselPeriodFinalRange)
  {
    uint8_t temp = 0;
    i2cdevReadByte(I2Cx, devAddr, VL53L0X_RA_FINAL_RANGE_CONFIG_VCSEL_PERIOD, &temp);
    return decodeVcselPeriod(temp);
  }
  else { return 255; }
}

// Start continuous ranging measurements. If period_ms (optional) is 0 or not
// given, continuous back-to-back mode is used (the sensor takes measurements as
// often as possible); otherwise, continuous timed mode is used, with the given
// inter-measurement period in milliseconds determining how often the sensor
// takes a measurement.
// based on VL53L0X_StartMeasurement()
void vl53l0xStartContinuous(uint32_t period_ms)
{
  i2cdevWriteByte(I2Cx, devAddr, 0x80, 0x01);
  i2cdevWriteByte(I2Cx, devAddr, 0xFF, 0x01);
  i2cdevWriteByte(I2Cx, devAddr, 0x00, 0x00);
  i2cdevWriteByte(I2Cx, devAddr, 0x91, stop_variable);
  i2cdevWriteByte(I2Cx, devAddr, 0x00, 0x01);
  i2cdevWriteByte(I2Cx, devAddr, 0xFF, 0x00);
  i2cdevWriteByte(I2Cx, devAddr, 0x80, 0x00);

  if (period_ms != 0)
  {
    // continuous timed mode

    // VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin

    uint16_t osc_calibrate_val = vl53l0xReadReg16Bit(VL53L0X_RA_OSC_CALIBRATE_VAL);

    if (osc_calibrate_val != 0)
    {
      period_ms *= osc_calibrate_val;
    }

    vl53l0xWriteReg32Bit(VL53L0X_RA_SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);

    // VL53L0X_SetInterMeasurementPeriodMilliSeconds() end

    i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_SYSRANGE_START, 0x04); // VL53L0X_REG_SYSRANGE_MODE_TIMED
  }
  else
  {
    // continuous back-to-back mode
    i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_SYSRANGE_START, 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
  }
}

// Stop continuous measurements
// based on VL53L0X_StopMeasurement()
void vl53l0xStopContinuous(void)
{
  i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_SYSRANGE_START, 0x01); // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT

  i2cdevWriteByte(I2Cx, devAddr, 0xFF, 0x01);
  i2cdevWriteByte(I2Cx, devAddr, 0x00, 0x00);
  i2cdevWriteByte(I2Cx, devAddr, 0x91, 0x00);
  i2cdevWriteByte(I2Cx, devAddr, 0x00, 0x01);
  i2cdevWriteByte(I2Cx, devAddr, 0xFF, 0x00);
}

// Returns a range reading in millimeters when continuous mode is active
// (readRangeSingleMillimeters() also calls this function after starting a
// single-shot range measurement)
uint16_t vl53l0xReadRangeContinuousMillimeters(void)
{
  startTimeout();
  uint8_t val = 0;
  while ((val & 0x07) == 0)
  {
    i2cdevReadByte(I2Cx, devAddr, VL53L0X_RA_RESULT_INTERRUPT_STATUS, &val);
    if ((val & 0x07) == 0)
    {
      // Relaxation delay when polling interrupt
      vTaskDelay(M2T(1));
    }
    if (checkTimeoutExpired())
    {
      did_timeout = true;
      return 65535;
    }
  }

  // assumptions: Linearity Corrective Gain is 1000 (default);
  // fractional ranging is not enabled
  uint16_t range = vl53l0xReadReg16Bit(VL53L0X_RA_RESULT_RANGE_STATUS + 10);

  i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_SYSTEM_INTERRUPT_CLEAR, 0x01);

  return range;
}

// Performs a single-shot range measurement and returns the reading in
// millimeters
// based on VL53L0X_PerformSingleRangingMeasurement()
uint16_t vl53l0xReadRangeSingleMillimeters(void)
{
  i2cdevWriteByte(I2Cx, devAddr, 0x80, 0x01);
  i2cdevWriteByte(I2Cx, devAddr, 0xFF, 0x01);
  i2cdevWriteByte(I2Cx, devAddr, 0x00, 0x00);
  i2cdevWriteByte(I2Cx, devAddr, 0x91, stop_variable);
  i2cdevWriteByte(I2Cx, devAddr, 0x00, 0x01);
  i2cdevWriteByte(I2Cx, devAddr, 0xFF, 0x00);
  i2cdevWriteByte(I2Cx, devAddr, 0x80, 0x00);

  i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_SYSRANGE_START, 0x01);

  // "Wait until start bit has been cleared"
  startTimeout();
  uint8_t val = 0x01;
  while (val & 0x01)
  {
    i2cdevReadByte(I2Cx, devAddr, VL53L0X_RA_SYSRANGE_START, &val);
    if (checkTimeoutExpired())
    {
      did_timeout = true;
      return 65535;
    }
  }

  return vl53l0xReadRangeContinuousMillimeters();
}

// Get reference SPAD (single photon avalanche diode) count and type
// based on VL53L0X_get_info_from_device(),
// but only gets reference SPAD count and type
bool vl53l0xGetSpadInfo(uint8_t * count, bool * type_is_aperture)
{
  uint8_t tmp;

  i2cdevWriteByte(I2Cx, devAddr, 0x80, 0x01);
  i2cdevWriteByte(I2Cx, devAddr, 0xFF, 0x01);
  i2cdevWriteByte(I2Cx, devAddr, 0x00, 0x00);

  i2cdevWriteByte(I2Cx, devAddr, 0xFF, 0x06);
  i2cdevWriteBit(I2Cx, devAddr, 0x83, 2, 0x01);
  i2cdevWriteByte(I2Cx, devAddr, 0xFF, 0x07);
  i2cdevWriteByte(I2Cx, devAddr, 0x81, 0x01);

  i2cdevWriteByte(I2Cx, devAddr, 0x80, 0x01);

  i2cdevWriteByte(I2Cx, devAddr, 0x94, 0x6b);
  i2cdevWriteByte(I2Cx, devAddr, 0x83, 0x00);
  startTimeout();

  uint8_t val = 0x00;
  while (val == 0x00) {
    i2cdevReadByte(I2Cx, devAddr, 0x83, &val);
    if (checkTimeoutExpired()) { return false; }
  };
  i2cdevWriteByte(I2Cx, devAddr, 0x83, 0x01);
  i2cdevReadByte(I2Cx, devAddr, 0x92, &tmp);

  *count = tmp & 0x7f;
  *type_is_aperture = (tmp >> 7) & 0x01;

  i2cdevWriteByte(I2Cx, devAddr, 0x81, 0x00);
  i2cdevWriteByte(I2Cx, devAddr, 0xFF, 0x06);
  i2cdevWriteBit(I2Cx, devAddr, 0x83, 2, 0);
  i2cdevWriteByte(I2Cx, devAddr, 0xFF, 0x01);
  i2cdevWriteByte(I2Cx, devAddr, 0x00, 0x01);

  i2cdevWriteByte(I2Cx, devAddr, 0xFF, 0x00);
  i2cdevWriteByte(I2Cx, devAddr, 0x80, 0x00);

  return true;
}

// Get sequence step enables
// based on VL53L0X_GetSequenceStepEnables()
void vl53l0xGetSequenceStepEnables(SequenceStepEnables * enables)
{
  uint8_t sequence_config = 0;
  i2cdevReadByte(I2Cx, devAddr, VL53L0X_RA_SYSTEM_SEQUENCE_CONFIG, &sequence_config);

  enables->tcc          = (sequence_config >> 4) & 0x1;
  enables->dss          = (sequence_config >> 3) & 0x1;
  enables->msrc         = (sequence_config >> 2) & 0x1;
  enables->pre_range    = (sequence_config >> 6) & 0x1;
  enables->final_range  = (sequence_config >> 7) & 0x1;
}

// Get sequence step timeouts
// based on get_sequence_step_timeout(),
// but gets all timeouts instead of just the requested one, and also stores
// intermediate values
void vl53l0xGetSequenceStepTimeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts)
{
  timeouts->pre_range_vcsel_period_pclks = vl53l0xGetVcselPulsePeriod(VcselPeriodPreRange);

  uint8_t temp = 0;
  i2cdevReadByte(I2Cx, devAddr, VL53L0X_RA_MSRC_CONFIG_TIMEOUT_MACROP, &temp);
  timeouts->msrc_dss_tcc_mclks = temp + 1;
  timeouts->msrc_dss_tcc_us =
    vl53l0xTimeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  uint16_t pre_range_encoded = vl53l0xReadReg16Bit(VL53L0X_RA_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI);
  timeouts->pre_range_mclks = vl53l0xDecodeTimeout(pre_range_encoded);
  timeouts->pre_range_us =
    vl53l0xTimeoutMclksToMicroseconds(timeouts->pre_range_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  timeouts->final_range_vcsel_period_pclks = vl53l0xGetVcselPulsePeriod(VcselPeriodFinalRange);

  uint16_t final_range_encoded = vl53l0xReadReg16Bit(VL53L0X_RA_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI);
  timeouts->final_range_mclks = vl53l0xDecodeTimeout(final_range_encoded);

  if (enables->pre_range)
  {
    timeouts->final_range_mclks -= timeouts->pre_range_mclks;
  }

  timeouts->final_range_us =
    vl53l0xTimeoutMclksToMicroseconds(timeouts->final_range_mclks,
                               timeouts->final_range_vcsel_period_pclks);
}

// Decode sequence step timeout in MCLKs from register value
// based on VL53L0X_decode_timeout()
// Note: the original function returned a uint32_t, but the return value is
// always stored in a uint16_t.
uint16_t vl53l0xDecodeTimeout(uint16_t reg_val)
{
  // format: "(LSByte * 2^MSByte) + 1"
  return (uint16_t)((reg_val & 0x00FF) <<
         (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L0X_encode_timeout()
// Note: the original function took a uint16_t, but the argument passed to it
// is always a uint16_t.
uint16_t vl53l0xEncodeTimeout(uint16_t timeout_mclks)
{
  // format: "(LSByte * 2^MSByte) + 1"

  uint32_t ls_byte = 0;
  uint16_t ms_byte = 0;

  if (timeout_mclks > 0)
  {
    ls_byte = timeout_mclks - 1;

    while ((ls_byte & 0xFFFFFF00LU) > 0)
    {
      ls_byte >>= 1;
      ms_byte++;
    }

    return (ms_byte << 8) | (ls_byte & 0xFF);
  }
  else { return 0; }
}

// Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_us()
uint32_t vl53l0xTimeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

// Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_mclks()
uint32_t vl53l0xTimeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

// based on VL53L0X_perform_single_ref_calibration()
bool vl53l0xPerformSingleRefCalibration(uint8_t vhv_init_byte)
{
  i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP

  startTimeout();
  uint8_t temp = 0x00;
  while ((temp & 0x07) == 0)
  {
    i2cdevReadByte(I2Cx, devAddr, VL53L0X_RA_RESULT_INTERRUPT_STATUS, &temp);
    if (checkTimeoutExpired()) { return false; }
  }

  i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_SYSTEM_INTERRUPT_CLEAR, 0x01);

  i2cdevWriteByte(I2Cx, devAddr, VL53L0X_RA_SYSRANGE_START, 0x00);

  return true;
}

uint16_t vl53l0xReadReg16Bit(uint8_t reg)
{
  uint8_t buffer[2] = {};
  i2cdevRead(I2Cx, devAddr, reg, 2, (uint8_t *)&buffer);
  return ((uint16_t)(buffer[0]) << 8) | buffer[1];
}

bool vl53l0xWriteReg16Bit(uint8_t reg, uint16_t val)
{
  uint8_t buffer[2] = {};
  buffer[0] = ((val >> 8) & 0xFF);
  buffer[1] = ((val     ) & 0xFF);
  return i2cdevWrite(I2Cx, devAddr, reg, 2, (uint8_t *)&buffer);
}

bool vl53l0xWriteReg32Bit(uint8_t reg, uint32_t val)
{
  uint8_t buffer[4] = {};
  buffer[0] = ((val >> 24) & 0xFF);
  buffer[1] = ((val >> 16) & 0xFF);
  buffer[2] = ((val >>  8) & 0xFF);
  buffer[3] = ((val      ) & 0xFF);
  return i2cdevWrite(I2Cx, devAddr, reg, 4, (uint8_t *)&buffer);
}

bool check_p2p() {
  for(int i=0; i<64; i++) {
    for(int j=0; j<BG_FRAME_NUM; j++) {
      if (abs(avgCal[i] - bgData[i+j*64]) > PEAK_THREHOLD) return false;
    }
  }
  return true;
}

void calibration() {
  for(int i=0; i<64; i++) {
    float tempSum = 0;
    for(int j=0; j<BG_FRAME_NUM; j++) {
      tempSum += bgData[i+j*64];
    }
    avgCal[i] = tempSum / BG_FRAME_NUM;
  }

  for(int i=0; i<64; i++) {
    float tempSum = 0;
    for(int j=0; j<BG_FRAME_NUM; j++) {
      tempSum += powf(bgData[i+j*64]-avgCal[i], 2);
    }
    stdCal[i] = sqrtf(tempSum/ BG_FRAME_NUM);
  }
}

void zscoreCalculation() {
  float tempMax = 0.0f;
  float tempMin = 30.0f;
  for(int i=0; i<64; i++) {
    float tempSum = 0;
    for(int j=0; j<RAW_DATA_FRAME_NUM; j++) {
      tempSum += rawData[i+j*64];
    }
    float sampleAvg = tempSum / RAW_DATA_FRAME_NUM;
    int map_index = i + (64*rawDataFrameIndex);
    zscoreData[map_index] = (sampleAvg - avgCal[i]) / stdCal[i];

    tempSum = 0;
    for(int j=0; j<RAW_DATA_FRAME_NUM; j++) {
      tempSum += zscoreData[i+j*64];
    }
    zscore[i] = tempSum / RAW_DATA_FRAME_NUM;
    zscore_int16_t[i] = (int)(zscore[i]*100);

    if(zscore[i] > tempMax) {
      tempMax = zscore[i];
    }

    if(zscore[i] < tempMin) {
      tempMin = zscore[i];
    }
  }
  z_max = (int)(tempMax*100);
  z_min = (int)(tempMin*100);
}

void rotateColor(uint8_t* color, uint8_t* rColor)
{
    int x_grid = (RIGHT_UP - LEFT_UP) / 7;
    int y_grid = (LEFT_DOWN - LEFT_UP) / 7;
    for(int i=0; i<8; i++) {
        for (int j=0; j<8; j++) {
            rColor[i*8+j] = color[LEFT_UP + x_grid*j + y_grid*i];
        }
    }

}

void labelPixel(int* largestSubset, int maxSubsetLen, uint8_t* color)
{
    for(int i=0; i<64; i++) {
        color[i] = WHITE;
    }

    for(int i=0; i<maxSubsetLen; i++) {
        int pixelIndex = largestSubset[i];
        color[pixelIndex] = ORANGE;
    }
/*
    for(int i=0; i<64; i++){
        // if ((pixelTemp[i]-roomTemp) < THRESHOLD_LEVEL1){
        if (zscore[i] < 8.0f){
            color[i]=WHITE;
        // }else if ((pixelTemp[i]-roomTemp) < THRESHOLD_LEVEL2){
        }else if (zscore[i] < 10.0f){
            color[i]=YELLOW; // YELLOW
        }else{
            color[i]=ORANGE;
        }
    }
*/
    // if ((pixelTemp[maxPixelIndex]-roomTemp) < THRESHOLD_LEVEL1){
    //     color[maxPixelIndex]=WHITE;
    // }else{
    //     color[maxPixelIndex]=ORANGE;
    // }
}

void findGroup(uint8_t* color)
{
    yellowGroupH = 0;
    yellowGroupL = 0;
    orangeGroupH = 0;
    orangeGroupL = 0;
    for(int i=0; i<32; i++){
        uint8_t cl = color[i];
        if(cl == YELLOW){
            yellowGroupL |= 1<<i;
        }else if(cl == ORANGE){
            orangeGroupL |= 1<<i;
        }
        cl = color[i+32];
        if(cl == YELLOW){
            yellowGroupH |= 1<<i;
        }else if(cl == ORANGE){
            orangeGroupH |= 1<<i;
        }
    }
}


void get_neighbor(int loc, int* neighbor) //neighbor length maximum is 8
{
        if((loc / ARRAY_SIZE)==0 && (loc % ARRAY_SIZE)==0){
            neighbor[0]=loc+1; neighbor[1]=loc+ARRAY_SIZE; neighbor[2]=loc+ARRAY_SIZE+1;
        }else if((loc / ARRAY_SIZE)==0 && (loc % ARRAY_SIZE)>0 && (loc % ARRAY_SIZE)<(ARRAY_SIZE-1)){
            neighbor[0]=loc-1; neighbor[1]=loc+1; neighbor[2]=loc+ARRAY_SIZE-1; neighbor[3]=loc+ARRAY_SIZE; neighbor[4]=loc+ARRAY_SIZE+1;
        }else if((loc / ARRAY_SIZE)==0 && (loc % ARRAY_SIZE)==(ARRAY_SIZE-1)){
            neighbor[0]=loc-1; neighbor[1]=loc+ARRAY_SIZE-1; neighbor[2]=loc+ARRAY_SIZE;
        }else if((loc / ARRAY_SIZE)> 0 && (loc / ARRAY_SIZE)<(ARRAY_SIZE-1) && (loc % ARRAY_SIZE)==0){
            neighbor[0]=loc-ARRAY_SIZE; neighbor[1]=loc-ARRAY_SIZE+1; neighbor[2]=loc+1; neighbor[3]=loc+ARRAY_SIZE; neighbor[4]=loc+ARRAY_SIZE+1;
        }else if((loc / ARRAY_SIZE)>0 && (loc / ARRAY_SIZE)<(ARRAY_SIZE-1) && (loc % ARRAY_SIZE)>0 && (loc % ARRAY_SIZE)<(ARRAY_SIZE-1)){
            neighbor[0]=loc-ARRAY_SIZE-1; neighbor[1]=loc-ARRAY_SIZE; neighbor[2]=loc-ARRAY_SIZE+1; neighbor[3]=loc-1; neighbor[4]=loc+1; neighbor[5]=loc+ARRAY_SIZE-1; neighbor[6]=loc+ARRAY_SIZE; neighbor[7]=loc+ARRAY_SIZE+1;
        }else if((loc / ARRAY_SIZE)>0 && (loc / ARRAY_SIZE)<(ARRAY_SIZE-1) && (loc % ARRAY_SIZE)==(ARRAY_SIZE-1)){
            neighbor[0]=loc-ARRAY_SIZE-1; neighbor[1]=loc-ARRAY_SIZE; neighbor[2]=loc-1; neighbor[3]=loc+ARRAY_SIZE-1; neighbor[4]=loc+ARRAY_SIZE;
        }else if((loc / ARRAY_SIZE)==(ARRAY_SIZE-1) && (loc % ARRAY_SIZE)==0){
            neighbor[0]=loc-ARRAY_SIZE; neighbor[1]=loc-ARRAY_SIZE+1; neighbor[2]=loc+1;
        }else if((loc / ARRAY_SIZE)==(ARRAY_SIZE-1) && (loc % ARRAY_SIZE)>0 && (loc % ARRAY_SIZE)<(ARRAY_SIZE-1)){
            neighbor[0]=loc-ARRAY_SIZE-1; neighbor[1]=loc-ARRAY_SIZE; neighbor[2]=loc-ARRAY_SIZE+1; neighbor[3]=loc-1; neighbor[4]=loc+1;
        }else if((loc / ARRAY_SIZE)==(ARRAY_SIZE-1) && (loc % ARRAY_SIZE)==(ARRAY_SIZE-1)){
            neighbor[0]=loc-ARRAY_SIZE-1; neighbor[1]=loc-ARRAY_SIZE; neighbor[2]=loc-1;
        }
}

// void get_neighbor(int loc, int* neighbor) //neighbor length maximum is 4
// {
//         if((loc / ARRAY_SIZE)==0 && (loc % ARRAY_SIZE)==0){
//             neighbor[0]=loc+1; neighbor[1]=loc+ARRAY_SIZE;
//         }else if((loc / ARRAY_SIZE)==0 && (loc % ARRAY_SIZE)>0 && (loc % ARRAY_SIZE)<(ARRAY_SIZE-1)){
//             neighbor[0]=loc-1; neighbor[1]=loc+1; neighbor[2]=loc+ARRAY_SIZE;
//         }else if((loc / ARRAY_SIZE)==0 && (loc % ARRAY_SIZE)==(ARRAY_SIZE-1)){
//             neighbor[0]=loc-1; neighbor[1]=loc+ARRAY_SIZE;
//         }else if((loc / ARRAY_SIZE)> 0 && (loc / ARRAY_SIZE)<(ARRAY_SIZE-1) && (loc % ARRAY_SIZE)==0){
//             neighbor[0]=loc-ARRAY_SIZE; neighbor[1]=loc+1; neighbor[2]=loc+ARRAY_SIZE;
//         }else if((loc / ARRAY_SIZE)>0 && (loc / ARRAY_SIZE)<(ARRAY_SIZE-1) && (loc % ARRAY_SIZE)>0 && (loc % ARRAY_SIZE)<(ARRAY_SIZE-1)){
//             neighbor[0]=loc-ARRAY_SIZE; neighbor[1]=loc-1; neighbor[2]=loc+1; neighbor[3]=loc+ARRAY_SIZE;
//         }else if((loc / ARRAY_SIZE)>0 && (loc / ARRAY_SIZE)<(ARRAY_SIZE-1) && (loc % ARRAY_SIZE)==(ARRAY_SIZE-1)){
//             neighbor[0]=loc-ARRAY_SIZE; neighbor[1]=loc-1; neighbor[2]=loc+ARRAY_SIZE;
//         }else if((loc / ARRAY_SIZE)==(ARRAY_SIZE-1) && (loc % ARRAY_SIZE)==0){
//             neighbor[0]=loc-ARRAY_SIZE; neighbor[1]=loc+1;
//         }else if((loc / ARRAY_SIZE)==(ARRAY_SIZE-1) && (loc % ARRAY_SIZE)>0 && (loc % ARRAY_SIZE)<(ARRAY_SIZE-1)){
//             neighbor[0]=loc-ARRAY_SIZE; neighbor[1]=loc-1; neighbor[2]=loc+1;
//         }else if((loc / ARRAY_SIZE)==(ARRAY_SIZE-1) && (loc % ARRAY_SIZE)==(ARRAY_SIZE-1)){
//             neighbor[0]=loc-ARRAY_SIZE; neighbor[1]=loc-1;
//         }
// }

bool label_neighbor(int result[], int subsetNumber){
    bool hasNeighbor = false;
    for(int i=0; i<64; i++){
        if(result[i]==subsetNumber){
            int neighbor[8]={-1,-1,-1,-1,-1,-1,-1,-1};
            get_neighbor(i, neighbor);
            for(int i=0; i<8;i++){
                if(neighbor[i] != -1 && result[neighbor[i]] == WAITTOCHECK){
                    result[neighbor[i]]=NEIGHBOR;
                    hasNeighbor = true;
                }
            }
        }
    }
    return hasNeighbor;
}

void label_subset(int testset[], int testsetLen, int result[], int subsetNumber){
    while(label_neighbor(result, subsetNumber)){
        for(int i=0; i< testsetLen; i++){
            if(result[testset[i]] == NEIGHBOR){
                result[testset[i]]=subsetNumber;
            }
        }
        for(int i=0; i<64; i++){
            if(result[i] == NEIGHBOR){
                result[i]=CHECKED;
            }
        }
    }
}

bool get_startIndex(int testset[], int testsetLen, int result[], int* startIndex){
    for(int i=0; i<testsetLen; i++){
        if(result[testset[i]] == WAITTOCHECK){
            *startIndex = i;
            return true;
        }
    }
    return false;
}

void select_largest_subset(int testset[], int testsetLen, int result[], int subsetNumber, int* maxSubsetLen, int* largestSubset){
    int maxSubsetNumber = 0;
    for(int i=0; i<subsetNumber; i++){
        int lengthCount = 0;
        for(int j=0; j<testsetLen; j++){
            if(result[testset[j]] == i){
                lengthCount++;
            }
        }
        if(lengthCount > *maxSubsetLen){ // if equal, no solution currently
            *maxSubsetLen = lengthCount;
            maxSubsetNumber = i;
        }
    }

    // largestSubset = (int*)malloc(sizeof(int)*(*maxSubsetLen));
    int index=0;
    for(int i=0; i<64; i++){
        if(result[i]==maxSubsetNumber){
            largestSubset[index]=i;
            index++;
        }
    }
}

void find_largestSubset(int testset[], int testsetLen, int* maxSubsetLen, int* largestSubset){
    int result[64];
    for(int i=0; i<64;i++){result[i]=WAITTOCHECK;}
    int subsetNumber = 0;
    int startIndex = 0;
    while(get_startIndex(testset, testsetLen, result, &startIndex)){
        result[testset[startIndex]]=subsetNumber;
        label_subset(testset, testsetLen, result, subsetNumber);
        subsetNumber++;
    }
    select_largest_subset(testset, testsetLen, result, subsetNumber, maxSubsetLen, largestSubset);
}

void get_largest_subset(int* largestSubset, int* maxSubsetLen){
    int testset[64]={0};
    int pixelCount=0;
    for(int i=0; i<64; i++){
        if (zscore[i] > zscore_threshold_high) {
            testset[pixelCount]=i;
            pixelCount++;
            zscoreWeight[i]=true;
        } else if (zscore[i] > zscore_threshold_low) {
            if (zscoreWeight[i] == true) {
                testset[pixelCount]=i;
                pixelCount++;
            }
        } else {
                zscoreWeight[i]=false;
        }
    }
    testset[pixelCount] = BOUNDARY;
    if(pixelCount == 0){
        roll = (float)(MAXRANGE/2);
        pitch = (float)(MAXRANGE/2);
        // return false;
    }
    // int* largestSubset;
    // int maxSubsetLen = 0;
    find_largestSubset(testset, pixelCount, maxSubsetLen, largestSubset);

    // If subset only has one pixel higher than zscore threshold, it will be consideredd as a noise.
    // This subset will be reset here.
    if (*maxSubsetLen == 1) {
      *maxSubsetLen = 0;
      largestSubset[0] = -1; // reset
    }
    // // calculate_height(maxSubsetLen);
    // // calculate_xy(maxSubsetLen, largestSubset);
    // count = (count+1) % 10;
    // return true;
}

// void rotateXY() {
//     int x_grid = (RIGHT_UP - LEFT_UP) / 7;
//     int y_grid = (LEFT_DOWN - LEFT_UP) / 7;
//     float r_pixelIndex = LEFT_UP + (x_grid * x_weight_coordinate) + (y_grid * y_weight_coordinate);
//     x_weight_coordinate = fmod(r_pixelIndex, 8);
//     y_weight_coordinate = r_pixelIndex / 8;
// }

void get_xy(int* largestSubset, int maxSubsetLen) {
    if (maxSubsetLen == 0) {
        x_weight_coordinate = -1.0;
        y_weight_coordinate = -1.0;
        return;
    }

    float x_weight_zscore_sum = 0.0f;
    float y_weight_zscore_sum = 0.0f;
    float zscore_sum = 0.0f;

    for(int i=0; i<maxSubsetLen; i++) {
        float _zscore = zscore[largestSubset[i]];
        zscore_sum += _zscore;
    }

    for(int i=0; i<maxSubsetLen; i++) {
        int _x = largestSubset[i] % 8;
        int _y = largestSubset[i] / 8;
        float _zscore = zscore[largestSubset[i]];
        x_weight_zscore_sum += _x * _zscore / zscore_sum;
        y_weight_zscore_sum += _y * _zscore / zscore_sum;
    }
    x_weight_coordinate = x_weight_zscore_sum;
    y_weight_coordinate = y_weight_zscore_sum;
}

void get_total_heat(int* largestSubset, int maxSubsetLen) {
    if (maxSubsetLen == 0) {
        z_total_heat = -1;
        return;
    }

    float zscore_sum = 0.0f;
    for(int i=0; i<maxSubsetLen; i++) {
        float _zscore = zscore[largestSubset[i]];
        zscore_sum += _zscore;
    }
    // z_total_heat = sqrtf(zscore_sum);
    z_total_heat = (int)(zscore_sum*100);
}

static const DeckDriver vl53l0x_deck = {
  .vid = 0xBC,
  .pid = 0x09,
  .name = "bcZRanger",
  .usedGpio = 0x0C,

  .init = vl53l0xInit,
  .test = vl53l0xTest,
};

DECK_DRIVER(vl53l0x_deck);

PARAM_GROUP_START(deck)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcZRanger, &isInit)
PARAM_GROUP_STOP(deck)

PARAM_GROUP_START(zscore)
PARAM_ADD(PARAM_FLOAT, thre_high, &zscore_threshold_high)
PARAM_ADD(PARAM_FLOAT, thre_low, &zscore_threshold_low)
PARAM_GROUP_STOP(zscore)

LOG_GROUP_START(range)
LOG_ADD(LOG_UINT16, zrange, &range_last)
LOG_ADD(LOG_UINT16, range_down, &range_last_down)
LOG_ADD(LOG_UINT16, range_front, &range_last_front)
// LOG_ADD(LOG_UINT16, range_top, &range_last_top)
LOG_GROUP_STOP(range)

LOG_GROUP_START(gridEye)
LOG_ADD(LOG_FLOAT, roomTemp, &roomTemp)
LOG_ADD(LOG_UINT32, yellowGroupH, &yellowGroupH)
LOG_ADD(LOG_UINT32, yellowGroupL, &yellowGroupL)
LOG_ADD(LOG_UINT32, orangeGroupH, &orangeGroupH)
LOG_ADD(LOG_UINT32, orangeGroupL, &orangeGroupL)
LOG_GROUP_STOP(gridEye)

LOG_GROUP_START(gridEyeXYZ)
LOG_ADD(LOG_FLOAT, xw, &x_weight_coordinate)
LOG_ADD(LOG_FLOAT, yw, &y_weight_coordinate)
LOG_ADD(LOG_INT16, zh, &z_total_heat)
LOG_ADD(LOG_INT16, zMax, &z_max)
LOG_ADD(LOG_INT16, zMin, &z_min)
LOG_GROUP_STOP(gridEyeXYZ)


// char groupName[10] = "gridEyeR";
// char varName[10] = "rawDataC";
// for(int i=0;i<1;i++)
// {
//   groupName[8] = (char)(i+48);
//   LOG_GROUP_START(groupName)
//   for(int j=0;j<8;j++)
//   {
//     varName[8] = (char)(j+48);
//     LOG_ADD(LOG_UINT16, varName, &rawData[i*8+j]);
//   }
//   LOG_GROUP_STOP(groupName)
// }

// LOG_GROUP_START(gridEyeR0)
// LOG_ADD(LOG_INT16, rawDataC0, &zscore_int16_t[0])
// LOG_ADD(LOG_INT16, rawDataC1, &zscore_int16_t[1])
// LOG_ADD(LOG_INT16, rawDataC2, &zscore_int16_t[2])
// LOG_ADD(LOG_INT16, rawDataC3, &zscore_int16_t[3])
// LOG_ADD(LOG_INT16, rawDataC4, &zscore_int16_t[4])
// LOG_ADD(LOG_INT16, rawDataC5, &zscore_int16_t[5])
// LOG_ADD(LOG_INT16, rawDataC6, &zscore_int16_t[6])
// LOG_ADD(LOG_INT16, rawDataC7, &zscore_int16_t[7])
// LOG_GROUP_STOP(gridEyeR0)

// LOG_GROUP_START(gridEyeR1)
// LOG_ADD(LOG_INT16, rawDataC0, &zscore_int16_t[8])
// LOG_ADD(LOG_INT16, rawDataC1, &zscore_int16_t[9])
// LOG_ADD(LOG_INT16, rawDataC2, &zscore_int16_t[10])
// LOG_ADD(LOG_INT16, rawDataC3, &zscore_int16_t[11])
// LOG_ADD(LOG_INT16, rawDataC4, &zscore_int16_t[12])
// LOG_ADD(LOG_INT16, rawDataC5, &zscore_int16_t[13])
// LOG_ADD(LOG_INT16, rawDataC6, &zscore_int16_t[14])
// LOG_ADD(LOG_INT16, rawDataC7, &zscore_int16_t[15])
// LOG_GROUP_STOP(gridEyeR1)

// LOG_GROUP_START(gridEyeR2)
// LOG_ADD(LOG_INT16, rawDataC0, &zscore_int16_t[16])
// LOG_ADD(LOG_INT16, rawDataC1, &zscore_int16_t[17])
// LOG_ADD(LOG_INT16, rawDataC2, &zscore_int16_t[18])
// LOG_ADD(LOG_INT16, rawDataC3, &zscore_int16_t[19])
// LOG_ADD(LOG_INT16, rawDataC4, &zscore_int16_t[20])
// LOG_ADD(LOG_INT16, rawDataC5, &zscore_int16_t[21])
// LOG_ADD(LOG_INT16, rawDataC6, &zscore_int16_t[22])
// LOG_ADD(LOG_INT16, rawDataC7, &zscore_int16_t[23])
// LOG_GROUP_STOP(gridEyeR2)

// LOG_GROUP_START(gridEyeR3)
// LOG_ADD(LOG_INT16, rawDataC0, &zscore_int16_t[24])
// LOG_ADD(LOG_INT16, rawDataC1, &zscore_int16_t[25])
// LOG_ADD(LOG_INT16, rawDataC2, &zscore_int16_t[26])
// LOG_ADD(LOG_INT16, rawDataC3, &zscore_int16_t[27])
// LOG_ADD(LOG_INT16, rawDataC4, &zscore_int16_t[28])
// LOG_ADD(LOG_INT16, rawDataC5, &zscore_int16_t[29])
// LOG_ADD(LOG_INT16, rawDataC6, &zscore_int16_t[30])
// LOG_ADD(LOG_INT16, rawDataC7, &zscore_int16_t[31])
// LOG_GROUP_STOP(gridEyeR3)

// LOG_GROUP_START(gridEyeR4)
// LOG_ADD(LOG_INT16, rawDataC0, &zscore_int16_t[32])
// LOG_ADD(LOG_INT16, rawDataC1, &zscore_int16_t[33])
// LOG_ADD(LOG_INT16, rawDataC2, &zscore_int16_t[34])
// LOG_ADD(LOG_INT16, rawDataC3, &zscore_int16_t[35])
// LOG_ADD(LOG_INT16, rawDataC4, &zscore_int16_t[36])
// LOG_ADD(LOG_INT16, rawDataC5, &zscore_int16_t[37])
// LOG_ADD(LOG_INT16, rawDataC6, &zscore_int16_t[38])
// LOG_ADD(LOG_INT16, rawDataC7, &zscore_int16_t[39])
// LOG_GROUP_STOP(gridEyeR4)

// LOG_GROUP_START(gridEyeR5)
// LOG_ADD(LOG_INT16, rawDataC0, &zscore_int16_t[40])
// LOG_ADD(LOG_INT16, rawDataC1, &zscore_int16_t[41])
// LOG_ADD(LOG_INT16, rawDataC2, &zscore_int16_t[42])
// LOG_ADD(LOG_INT16, rawDataC3, &zscore_int16_t[43])
// LOG_ADD(LOG_INT16, rawDataC4, &zscore_int16_t[44])
// LOG_ADD(LOG_INT16, rawDataC5, &zscore_int16_t[45])
// LOG_ADD(LOG_INT16, rawDataC6, &zscore_int16_t[46])
// LOG_ADD(LOG_INT16, rawDataC7, &zscore_int16_t[47])
// LOG_GROUP_STOP(gridEyeR5)

// LOG_GROUP_START(gridEyeR6)
// LOG_ADD(LOG_INT16, rawDataC0, &zscore_int16_t[48])
// LOG_ADD(LOG_INT16, rawDataC1, &zscore_int16_t[49])
// LOG_ADD(LOG_INT16, rawDataC2, &zscore_int16_t[50])
// LOG_ADD(LOG_INT16, rawDataC3, &zscore_int16_t[51])
// LOG_ADD(LOG_INT16, rawDataC4, &zscore_int16_t[52])
// LOG_ADD(LOG_INT16, rawDataC5, &zscore_int16_t[53])
// LOG_ADD(LOG_INT16, rawDataC6, &zscore_int16_t[54])
// LOG_ADD(LOG_INT16, rawDataC7, &zscore_int16_t[55])
// LOG_GROUP_STOP(gridEyeR6)

// LOG_GROUP_START(gridEyeR7)
// LOG_ADD(LOG_INT16, rawDataC0, &zscore_int16_t[56])
// LOG_ADD(LOG_INT16, rawDataC1, &zscore_int16_t[57])
// LOG_ADD(LOG_INT16, rawDataC2, &zscore_int16_t[58])
// LOG_ADD(LOG_INT16, rawDataC3, &zscore_int16_t[59])
// LOG_ADD(LOG_INT16, rawDataC4, &zscore_int16_t[60])
// LOG_ADD(LOG_INT16, rawDataC5, &zscore_int16_t[61])
// LOG_ADD(LOG_INT16, rawDataC6, &zscore_int16_t[62])
// LOG_ADD(LOG_INT16, rawDataC7, &zscore_int16_t[63])
// LOG_GROUP_STOP(gridEyeR7)
