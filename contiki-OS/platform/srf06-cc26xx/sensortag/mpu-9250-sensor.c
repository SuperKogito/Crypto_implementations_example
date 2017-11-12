/*
 * Copyright (c) 2014, Texas Instruments Incorporated - http://www.ti.com/
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup sensortag-cc26xx-mpu
 * @{
 *
 * \file
 *  Driver for the Sensortag Invensense MPU9250 motion processing unit
 */
/*---------------------------------------------------------------------------*/
#include "contiki-conf.h"
#include "lib/sensors.h"
#include "mpu-9250-sensor.h"
#include "sys/rtimer.h"
#include "sensor-common.h"
#include "board-i2c.h"

#include "ti-lib.h"

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
/*---------------------------------------------------------------------------*/
#define DEBUG 1
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define FIFO_CORRUPTION_CHECK
#ifdef FIFO_CORRUPTION_CHECK
#define QUAT_ERROR_THRESH       (1L<<24)
#define QUAT_MAG_SQ_NORMALIZED  (1L<<28)
#define QUAT_MAG_SQ_MIN         (QUAT_MAG_SQ_NORMALIZED - QUAT_ERROR_THRESH)
#define QUAT_MAG_SQ_MAX         (QUAT_MAG_SQ_NORMALIZED + QUAT_ERROR_THRESH)
#endif
/*---------------------------------------------------------------------------*/
/* Sensor I2C address */
#define SENSOR_I2C_ADDRESS            0x68
#define SENSOR_MAG_I2_ADDRESS         0x0C
/*---------------------------------------------------------------------------*/
/* Registers */
#define SELF_TEST_X_GYRO              0x00 /* R/W */
#define SELF_TEST_Y_GYRO              0x01 /* R/W */
#define SELF_TEST_Z_GYRO              0x02 /* R/W */
#define SELF_TEST_X_ACCEL             0x0D /* R/W */
#define SELF_TEST_Z_ACCEL             0x0E /* R/W */
#define SELF_TEST_Y_ACCEL             0x0F /* R/W */
/*---------------------------------------------------------------------------*/
#define XG_OFFSET_H                   0x13 /* R/W */
#define XG_OFFSET_L                   0x14 /* R/W */
#define YG_OFFSET_H                   0x15 /* R/W */
#define YG_OFFSET_L                   0x16 /* R/W */
#define ZG_OFFSET_H                   0x17 /* R/W */
#define ZG_OFFSET_L                   0x18 /* R/W */
/*---------------------------------------------------------------------------*/
#define SMPLRT_DIV                    0x19 /* R/W */
#define CONFIG                        0x1A /* R/W */
#define GYRO_CONFIG                   0x1B /* R/W */
#define ACCEL_CONFIG                  0x1C /* R/W */
#define ACCEL_CONFIG_2                0x1D /* R/W */
#define LP_ACCEL_ODR                  0x1E /* R/W */
#define WOM_THR                       0x1F /* R/W */
#define FIFO_EN                       0x23 /* R/W */
#define I2C_MST_CTRL                  0x24 /* R/W */
#define I2C_SLV0_ADDR                 0x25 /* R/W */
#define I2C_SLV0_REG                  0x26 /* R/W */
#define I2C_SLV0_CTRL                 0x27 /* R/W */
#define I2C_SLV1_ADDR                 0x28 /* R/W */
#define I2C_SLV1_REG                  0x29 /* R/W */
#define I2C_SLV1_CTRL                 0x2A /* R/W */
/*---------------------------------------------------------------------------*/
/*
 * Registers 0x24 - 0x36 are not applicable to the SensorTag HW configuration
 * (IC2 Master)
 */
#define I2C_SLV4_CTRL                 0x34 /* R/W */
#define INT_PIN_CFG                   0x37 /* R/W */
#define INT_ENABLE                    0x38 /* R/W */
#define INT_STATUS                    0x3A /* R */
#define ACCEL_XOUT_H                  0x3B /* R */
#define ACCEL_XOUT_L                  0x3C /* R */
#define ACCEL_YOUT_H                  0x3D /* R */
#define ACCEL_YOUT_L                  0x3E /* R */
#define ACCEL_ZOUT_H                  0x3F /* R */
#define ACCEL_ZOUT_L                  0x40 /* R */
#define TEMP_OUT_H                    0x41 /* R */
#define TEMP_OUT_L                    0x42 /* R */
#define GYRO_XOUT_H                   0x43 /* R */
#define GYRO_XOUT_L                   0x44 /* R */
#define GYRO_YOUT_H                   0x45 /* R */
#define GYRO_YOUT_L                   0x46 /* R */
#define GYRO_ZOUT_H                   0x47 /* R */
#define GYRO_ZOUT_L                   0x48 /* R */
#define RAW_COMPASS                   0x49 /* R */
/*---------------------------------------------------------------------------*/
/*
 * Registers 0x49 - 0x60 are not applicable to the SensorTag HW configuration
 * (external sensor data)
 *
 * Registers 0x63 - 0x67 are not applicable to the SensorTag HW configuration
 * (I2C master)
 */
#define I2C_SLV1_DO                   0x64 /* R/W */
#define I2C_MST_DELAY_CTRL            0x67 /* R/W */
#define SIGNAL_PATH_RESET             0x68 /* R/W */
#define ACCEL_INTEL_CTRL              0x69 /* R/W */
#define USER_CTRL                     0x6A /* R/W */
#define PWR_MGMT_1                    0x6B /* R/W */
#define PWR_MGMT_2                    0x6C /* R/W */
#define BANK_SEL                      0x6D /* R/W */
#define MEM_R_W                       0x6F /* R/W */
#define PRGM_START_H                  0x70 /* R/W */
#define FIFO_COUNT_H                  0x72 /* R/W */
#define FIFO_COUNT_L                  0x73 /* R/W */
#define FIFO_R_W                      0x74 /* R/W */
#define WHO_AM_I                      0x75 /* R/W */
/*---------------------------------------------------------------------------*/
/* Masks */
#define ACC_CONFIG_MASK               (0x38)
#define GYRO_CONFIG_MASK              (0x07)
#define BIT_DMP_EN                    (0x80)
#define BIT_FIFO_RST        (0x04)
#define BIT_FIFO_EN         (0x40)
#define BIT_DMP_RST         (0x08)

#define EN_XYZ_GYRO    (0x04)
#define EN_XYZ_ACCEL   (0x02)
#define EN_XYZ_COMPASS (0x01)
#define EN_ALL_SENSORS (EN_XYZ_GYRO | EN_XYZ_ACCEL | EN_XYZ_COMPASS)
#define FIFO_EN_GYRO   (0x70)
#define FIFO_EN_ACCEL  (0x08)
#define FIFO_EN_TMP    (0x80)
/*---------------------------------------------------------------------------*/
/* Values PWR_MGMT_1 */
#define MPU_SLEEP                     0x4F  /* Sleep + stop all clocks */
#define MPU_WAKE_UP                   0x09  /* Disable temp. + intern osc */
/*---------------------------------------------------------------------------*/
/* Values PWR_MGMT_2 */
#define EN_ALL_AXES                      0x00
#define DISABLE_ALL_AXES                 0x3F
#define DISABLE_GYRO_AXES                0x07
#define DISABLE_ACC_AXES                 0x38
/*---------------------------------------------------------------------------*/
/* Data sizes */
#define DATA_SIZE                     6
#define MPU_DATA_SIZE                 14
#define MAG_DATA_SIZE                 6
/*---------------------------------------------------------------------------*/
/* Output data rates */
#define INV_LPA_0_3125HZ              0
#define INV_LPA_0_625HZ               1
#define INV_LPA_1_25HZ                2
#define INV_LPA_2_5HZ                 3
#define INV_LPA_5HZ                   4
#define INV_LPA_10HZ                  5
#define INV_LPA_20HZ                  6
#define INV_LPA_40HZ                  7
#define INV_LPA_80HZ                  8
#define INV_LPA_160HZ                 9
#define INV_LPA_320HZ                 10
#define INV_LPA_640HZ                 11
#define INV_LPA_STOPPED               255
/*---------------------------------------------------------------------------*/
/* Bit values */
#define BIT_ANY_RD_CLR                0x10
#define BIT_RAW_RDY_EN                0x01
#define BIT_WOM_EN                    0x40
#define BIT_LPA_CYCLE                 0x20
#define BIT_STBY_XA                   0x20
#define BIT_STBY_YA                   0x10
#define BIT_STBY_ZA                   0x08
#define BIT_STBY_XG                   0x04
#define BIT_STBY_YG                   0x02
#define BIT_STBY_ZG                   0x01
#define BIT_STBY_XYZA                 (BIT_STBY_XA | BIT_STBY_YA | BIT_STBY_ZA)
#define BIT_STBY_XYZG                 (BIT_STBY_XG | BIT_STBY_YG | BIT_STBY_ZG)
#define BIT_FIFO_OVERFLOW            (0x10)
/*---------------------------------------------------------------------------*/
/* User control register */
#define BIT_ACTL                      0x80
#define BIT_LATCH_EN                  0x20
/*---------------------------------------------------------------------------*/
/* INT Pin / Bypass Enable Configuration */
#define BIT_AUX_IF_EN                 0x20 /* I2C_MST_EN */
#define BIT_BYPASS_EN                 0x02
/*---------------------------------------------------------------------------*/
#define ACC_RANGE_INVALID -1
#define MAX_COMPASS_SAMPLE_RATE  (100)
#define ACC_GYRO_INTERNAL_SMPLRT (1000)
//---------------- Mangetometer defines--------------------------------------
#define SUPPORTS_AK89xx_HIGH_SENS   (0x10)
#define AK89xx_FSR                  (4915)
#define AKM_REG_WHOAMI      (0x00)

#define AKM_REG_ST1         (0x02)
#define AKM_REG_HXL         (0x03)
#define AKM_REG_ST2         (0x09)

#define AKM_REG_CNTL        (0x0A)
#define AKM_REG_ASTC        (0x0C)
#define AKM_REG_ASAX        (0x10)
#define AKM_REG_ASAY        (0x11)
#define AKM_REG_ASAZ        (0x12)

#define AKM_DATA_READY      (0x01)
#define AKM_DATA_OVERRUN    (0x02)
#define AKM_OVERFLOW        (0x80)
#define AKM_DATA_ERROR      (0x40)

#define AKM_BIT_SELF_TEST   (0x40)

#define AKM_POWER_DOWN          (0x00 | SUPPORTS_AK89xx_HIGH_SENS)
#define AKM_SINGLE_MEASUREMENT  (0x01 | SUPPORTS_AK89xx_HIGH_SENS)
#define AKM_FUSE_ROM_ACCESS     (0x0F | SUPPORTS_AK89xx_HIGH_SENS)
#define AKM_MODE_SELF_TEST      (0x08 | SUPPORTS_AK89xx_HIGH_SENS)

#define AKM_WHOAMI      (0x48)
#define BIT_SLAVE_EN        (0x80)
#define BIT_I2C_READ        (0x80)
#define BIT_DMP_INT_EN      (0x02)
#define BIT_DATA_RDY_EN     (0x01)

#define MPU_MEM_BANK_SIZE (256)

enum MPU9250_ACCEL_RANGE {
  MPU9250_ACCEL_RANGE_2G = 0,
  MPU9250_ACCEL_RANGE_4G,
  MPU9250_ACCEL_RANGE_8G,
  MPU9250_ACCEL_RANGE_16G
};

#define MPU_AX_GYR_X      2
#define MPU_AX_GYR_Y      1
#define MPU_AX_GYR_Z      0
#define MPU_AX_GYR        0x07

#define MPU_AX_ACC_X      5
#define MPU_AX_ACC_Y      4
#define MPU_AX_ACC_Z      3
#define MPU_AX_ACC        0x38

enum MPU9250_ACCEL_DLPF {
    MPU9250_ACCEL_DLPF_460HZ = 0,
    MPU9250_ACCEL_DLPF_184HZ,
    MPU9250_ACCEL_DLPF_92HZ,
    MPU9250_ACCEL_DLPF_41HZ,
    MPU9250_ACCEL_DLPF_20HZ,
    MPU9250_ACCEL_DLPF_10HZ,
    MPU9250_ACCEL_DLPF_5HZ,
    MPU9250_ACCEL_DLPF_460HZ2,
    MPU9250_NUM_ACCEL_DLPF
};

enum MPU9250_GYRO_DLPF {
    MPU9250_GYRO_DLPF_250HZ = 0,
    MPU9250_GYRO_DLPF_184HZ,
    MPU9250_GYRO_DLPF_92HZ,
    MPU9250_GYRO_DLPF_41HZ,
    MPU9250_GYRO_DLPF_20HZ,
    MPU9250_GYRO_DLPF_10HZ,
    MPU9250_GYRO_DLPF_5HZ,
    MPU9250_GYRO_DLPF_3600HZ,
    NUM_GYRO_DLPF
};

enum MPU9250_GYRO_RANGE {
    MPU9250_RANGE_250DPS = 0,
    MPU9250_RANGE_500DPS,
    MPU9250_RANGE_1000DPS,
    MPU9250_RANGE_2000DPS,
    MPU9250_NUM_GYRO_RANGE
};

struct mpu_config_s {
  uint8_t gyro_range;
  uint8_t sensors;
  uint8_t acc_range;
  uint8_t lpf;
  uint8_t dmp_on;
  uint8_t bypass_mode;
  unsigned short sample_rate;
  unsigned short compass_sample_rate;
  short mag_sens_adj[3];
  uint8_t lp_accel_mode;
  uint8_t int_enable;
  uint8_t dmp_loaded;
  uint8_t dmp_sample_rate;
  uint8_t dmp_feature_mask;
  uint8_t dmp_packet_length;
  uint8_t dmp_fifo_rate;
};

//local copy of the MPU registers
struct mpu_reg_s {
  uint8_t REG_GYRO_CONFIG;
  uint8_t REG_ACCEL_CONFIG;
  uint8_t REG_ACCEL_CONFIG2;
  uint8_t REG_CONFIG;
  uint8_t REG_FIFO_EN;
  uint8_t REG_INT_PIN_CFG;
  uint8_t REG_INT_ENABLE;
  uint8_t REG_INT_STATUS;
  uint8_t REG_USER_CTRL;
  uint8_t REG_SMPLRT_DIV;
  uint8_t REG_PWR_MGMT1;
  uint8_t REG_PWR_MGMT2;
};

/*---------------------------------------------------------------------------*/
/*DMP Config*/
/* These defines are copied from dmpDefaultMPU6050.c in the general MPL
 * releases. These defines may change for each DMP image, so be sure to modify
 * these values when switching to a new image.
 */
#define CFG_LP_QUAT             (2712)
#define END_ORIENT_TEMP         (1866)
#define CFG_27                  (2742)
#define CFG_20                  (2224)
#define CFG_23                  (2745)
#define CFG_FIFO_ON_EVENT       (2690)
#define END_PREDICTION_UPDATE   (1761)
#define CGNOTICE_INTR           (2620)
#define X_GRT_Y_TMP             (1358)
#define CFG_DR_INT              (1029)
#define CFG_AUTH                (1035)
#define UPDATE_PROP_ROT         (1835)
#define END_COMPARE_Y_X_TMP2    (1455)
#define SKIP_X_GRT_Y_TMP        (1359)
#define SKIP_END_COMPARE        (1435)
#define FCFG_3                  (1088)
#define FCFG_2                  (1066)
#define FCFG_1                  (1062)
#define END_COMPARE_Y_X_TMP3    (1434)
#define FCFG_7                  (1073)
#define FCFG_6                  (1106)
#define FLAT_STATE_END          (1713)
#define SWING_END_4             (1616)
#define SWING_END_2             (1565)
#define SWING_END_3             (1587)
#define SWING_END_1             (1550)
#define CFG_8                   (2718)
#define CFG_15                  (2727)
#define CFG_16                  (2746)
#define CFG_EXT_GYRO_BIAS       (1189)
#define END_COMPARE_Y_X_TMP     (1407)
#define DO_NOT_UPDATE_PROP_ROT  (1839)
#define CFG_7                   (1205)
#define FLAT_STATE_END_TEMP     (1683)
#define END_COMPARE_Y_X         (1484)
#define SKIP_SWING_END_1        (1551)
#define SKIP_SWING_END_3        (1588)
#define SKIP_SWING_END_2        (1566)
#define TILTG75_START           (1672)
#define CFG_6                   (2753)
#define TILTL75_END             (1669)
#define END_ORIENT              (1884)
#define CFG_FLICK_IN            (2573)
#define TILTL75_START           (1643)
#define CFG_MOTION_BIAS         (1208)
#define X_GRT_Y                 (1408)
#define TEMPLABEL               (2324)
#define CFG_ANDROID_ORIENT_INT  (1853)
#define CFG_GYRO_RAW_DATA       (2722)
#define X_GRT_Y_TMP2            (1379)

#define D_0_22                  (22+512)
#define D_0_24                  (24+512)

#define D_0_36                  (36)
#define D_0_52                  (52)
#define D_0_96                  (96)
#define D_0_104                 (104)
#define D_0_108                 (108)
#define D_0_163                 (163)
#define D_0_188                 (188)
#define D_0_192                 (192)
#define D_0_224                 (224)
#define D_0_228                 (228)
#define D_0_232                 (232)
#define D_0_236                 (236)

#define D_1_2                   (256 + 2)
#define D_1_4                   (256 + 4)
#define D_1_8                   (256 + 8)
#define D_1_10                  (256 + 10)
#define D_1_24                  (256 + 24)
#define D_1_28                  (256 + 28)
#define D_1_36                  (256 + 36)
#define D_1_40                  (256 + 40)
#define D_1_44                  (256 + 44)
#define D_1_72                  (256 + 72)
#define D_1_74                  (256 + 74)
#define D_1_79                  (256 + 79)
#define D_1_88                  (256 + 88)
#define D_1_90                  (256 + 90)
#define D_1_92                  (256 + 92)
#define D_1_96                  (256 + 96)
#define D_1_98                  (256 + 98)
#define D_1_106                 (256 + 106)
#define D_1_108                 (256 + 108)
#define D_1_112                 (256 + 112)
#define D_1_128                 (256 + 144)
#define D_1_152                 (256 + 12)
#define D_1_160                 (256 + 160)
#define D_1_176                 (256 + 176)
#define D_1_178                 (256 + 178)
#define D_1_218                 (256 + 218)
#define D_1_232                 (256 + 232)
#define D_1_236                 (256 + 236)
#define D_1_240                 (256 + 240)
#define D_1_244                 (256 + 244)
#define D_1_250                 (256 + 250)
#define D_1_252                 (256 + 252)
#define D_2_12                  (512 + 12)
#define D_2_96                  (512 + 96)
#define D_2_108                 (512 + 108)
#define D_2_208                 (512 + 208)
#define D_2_224                 (512 + 224)
#define D_2_236                 (512 + 236)
#define D_2_244                 (512 + 244)
#define D_2_248                 (512 + 248)
#define D_2_252                 (512 + 252)

#define CPASS_BIAS_X            (35 * 16 + 4)
#define CPASS_BIAS_Y            (35 * 16 + 8)
#define CPASS_BIAS_Z            (35 * 16 + 12)
#define CPASS_MTX_00            (36 * 16)
#define CPASS_MTX_01            (36 * 16 + 4)
#define CPASS_MTX_02            (36 * 16 + 8)
#define CPASS_MTX_10            (36 * 16 + 12)
#define CPASS_MTX_11            (37 * 16)
#define CPASS_MTX_12            (37 * 16 + 4)
#define CPASS_MTX_20            (37 * 16 + 8)
#define CPASS_MTX_21            (37 * 16 + 12)
#define CPASS_MTX_22            (43 * 16 + 12)
#define D_EXT_GYRO_BIAS_X       (61 * 16)
#define D_EXT_GYRO_BIAS_Y       (61 * 16) + 4
#define D_EXT_GYRO_BIAS_Z       (61 * 16) + 8
#define D_ACT0                  (40 * 16)
#define D_ACSX                  (40 * 16 + 4)
#define D_ACSY                  (40 * 16 + 8)
#define D_ACSZ                  (40 * 16 + 12)

#define FLICK_MSG               (45 * 16 + 4)
#define FLICK_COUNTER           (45 * 16 + 8)
#define FLICK_LOWER             (45 * 16 + 12)
#define FLICK_UPPER             (46 * 16 + 12)

#define D_AUTH_OUT              (992)
#define D_AUTH_IN               (996)
#define D_AUTH_A                (1000)
#define D_AUTH_B                (1004)

#define D_PEDSTD_BP_B           (768 + 0x1C)
#define D_PEDSTD_HP_A           (768 + 0x78)
#define D_PEDSTD_HP_B           (768 + 0x7C)
#define D_PEDSTD_BP_A4          (768 + 0x40)
#define D_PEDSTD_BP_A3          (768 + 0x44)
#define D_PEDSTD_BP_A2          (768 + 0x48)
#define D_PEDSTD_BP_A1          (768 + 0x4C)
#define D_PEDSTD_INT_THRSH      (768 + 0x68)
#define D_PEDSTD_CLIP           (768 + 0x6C)
#define D_PEDSTD_SB             (768 + 0x28)
#define D_PEDSTD_SB_TIME        (768 + 0x2C)
#define D_PEDSTD_PEAKTHRSH      (768 + 0x98)
#define D_PEDSTD_TIML           (768 + 0x2A)
#define D_PEDSTD_TIMH           (768 + 0x2E)
#define D_PEDSTD_PEAK           (768 + 0X94)
#define D_PEDSTD_STEPCTR        (768 + 0x60)
#define D_PEDSTD_TIMECTR        (964)
#define D_PEDSTD_DECI           (768 + 0xA0)

#define D_HOST_NO_MOT           (976)
#define D_ACCEL_BIAS            (660)

#define D_ORIENT_GAP            (76)

#define D_TILT0_H               (48)
#define D_TILT0_L               (50)
#define D_TILT1_H               (52)
#define D_TILT1_L               (54)
#define D_TILT2_H               (56)
#define D_TILT2_L               (58)
#define D_TILT3_H               (60)
#define D_TILT3_L               (62)
#define DMP_CODE_SIZE           (3062)

static const uint8_t dmp_memory[DMP_CODE_SIZE] = {
    /* bank # 0 */
    0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x24, 0x00, 0x00, 0x00, 0x02, 0x00, 0x03, 0x00, 0x00,
    0x00, 0x65, 0x00, 0x54, 0xff, 0xef, 0x00, 0x00, 0xfa, 0x80, 0x00, 0x0b, 0x12, 0x82, 0x00, 0x01,
    0x03, 0x0c, 0x30, 0xc3, 0x0e, 0x8c, 0x8c, 0xe9, 0x14, 0xd5, 0x40, 0x02, 0x13, 0x71, 0x0f, 0x8e,
    0x38, 0x83, 0xf8, 0x83, 0x30, 0x00, 0xf8, 0x83, 0x25, 0x8e, 0xf8, 0x83, 0x30, 0x00, 0xf8, 0x83,
    0xff, 0xff, 0xff, 0xff, 0x0f, 0xfe, 0xa9, 0xd6, 0x24, 0x00, 0x04, 0x00, 0x1a, 0x82, 0x79, 0xa1,
    0x00, 0x00, 0x00, 0x3c, 0xff, 0xff, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x38, 0x83, 0x6f, 0xa2,
    0x00, 0x3e, 0x03, 0x30, 0x40, 0x00, 0x00, 0x00, 0x02, 0xca, 0xe3, 0x09, 0x3e, 0x80, 0x00, 0x00,
    0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00,
    0x00, 0x0c, 0x00, 0x00, 0x00, 0x0c, 0x18, 0x6e, 0x00, 0x00, 0x06, 0x92, 0x0a, 0x16, 0xc0, 0xdf,
    0xff, 0xff, 0x02, 0x56, 0xfd, 0x8c, 0xd3, 0x77, 0xff, 0xe1, 0xc4, 0x96, 0xe0, 0xc5, 0xbe, 0xaa,
    0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x0b, 0x2b, 0x00, 0x00, 0x16, 0x57, 0x00, 0x00, 0x03, 0x59,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1d, 0xfa, 0x00, 0x02, 0x6c, 0x1d, 0x00, 0x00, 0x00, 0x00,
    0x3f, 0xff, 0xdf, 0xeb, 0x00, 0x3e, 0xb3, 0xb6, 0x00, 0x0d, 0x22, 0x78, 0x00, 0x00, 0x2f, 0x3c,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x42, 0xb5, 0x00, 0x00, 0x39, 0xa2, 0x00, 0x00, 0xb3, 0x65,
    0xd9, 0x0e, 0x9f, 0xc9, 0x1d, 0xcf, 0x4c, 0x34, 0x30, 0x00, 0x00, 0x00, 0x50, 0x00, 0x00, 0x00,
    0x3b, 0xb6, 0x7a, 0xe8, 0x00, 0x64, 0x00, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    /* bank # 1 */
    0x10, 0x00, 0x00, 0x00, 0x10, 0x00, 0xfa, 0x92, 0x10, 0x00, 0x22, 0x5e, 0x00, 0x0d, 0x22, 0x9f,
    0x00, 0x01, 0x00, 0x00, 0x00, 0x32, 0x00, 0x00, 0xff, 0x46, 0x00, 0x00, 0x63, 0xd4, 0x00, 0x00,
    0x10, 0x00, 0x00, 0x00, 0x04, 0xd6, 0x00, 0x00, 0x04, 0xcc, 0x00, 0x00, 0x04, 0xcc, 0x00, 0x00,
    0x00, 0x00, 0x10, 0x72, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x06, 0x00, 0x02, 0x00, 0x05, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x05, 0x00, 0x64, 0x00, 0x20, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x03, 0x00,
    0x00, 0x00, 0x00, 0x32, 0xf8, 0x98, 0x00, 0x00, 0xff, 0x65, 0x00, 0x00, 0x83, 0x0f, 0x00, 0x00,
    0xff, 0x9b, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0xb2, 0x6a, 0x00, 0x02, 0x00, 0x00,
    0x00, 0x01, 0xfb, 0x83, 0x00, 0x68, 0x00, 0x00, 0x00, 0xd9, 0xfc, 0x00, 0x7c, 0xf1, 0xff, 0x83,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x00, 0x00, 0x64, 0x03, 0xe8, 0x00, 0x64, 0x00, 0x28,
    0x00, 0x00, 0x00, 0x25, 0x00, 0x00, 0x00, 0x00, 0x16, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00,
    0x00, 0x00, 0x10, 0x00, 0x00, 0x2f, 0x00, 0x00, 0x00, 0x00, 0x01, 0xf4, 0x00, 0x00, 0x10, 0x00,
    /* bank # 2 */
    0x00, 0x28, 0x00, 0x00, 0xff, 0xff, 0x45, 0x81, 0xff, 0xff, 0xfa, 0x72, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x44, 0x00, 0x05, 0x00, 0x05, 0xba, 0xc6, 0x00, 0x47, 0x78, 0xa2,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x14,
    0x00, 0x00, 0x25, 0x4d, 0x00, 0x2f, 0x70, 0x6d, 0x00, 0x00, 0x05, 0xae, 0x00, 0x0c, 0x02, 0xd0,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x64, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x0e,
    0x00, 0x00, 0x0a, 0xc7, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0xff, 0xff, 0xff, 0x9c,
    0x00, 0x00, 0x0b, 0x2b, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x64,
    0xff, 0xe5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    /* bank # 3 */
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x01, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x24, 0x26, 0xd3,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x10, 0x00, 0x96, 0x00, 0x3c,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x0c, 0x0a, 0x4e, 0x68, 0xcd, 0xcf, 0x77, 0x09, 0x50, 0x16, 0x67, 0x59, 0xc6, 0x19, 0xce, 0x82,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0xd7, 0x84, 0x00, 0x03, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc7, 0x93, 0x8f, 0x9d, 0x1e, 0x1b, 0x1c, 0x19,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x03, 0x18, 0x85, 0x00, 0x00, 0x40, 0x00,
    0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x67, 0x7d, 0xdf, 0x7e, 0x72, 0x90, 0x2e, 0x55, 0x4c, 0xf6, 0xe6, 0x88,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /* bank # 4 */
    0xd8, 0xdc, 0xb4, 0xb8, 0xb0, 0xd8, 0xb9, 0xab, 0xf3, 0xf8, 0xfa, 0xb3, 0xb7, 0xbb, 0x8e, 0x9e,
    0xae, 0xf1, 0x32, 0xf5, 0x1b, 0xf1, 0xb4, 0xb8, 0xb0, 0x80, 0x97, 0xf1, 0xa9, 0xdf, 0xdf, 0xdf,
    0xaa, 0xdf, 0xdf, 0xdf, 0xf2, 0xaa, 0xc5, 0xcd, 0xc7, 0xa9, 0x0c, 0xc9, 0x2c, 0x97, 0xf1, 0xa9,
    0x89, 0x26, 0x46, 0x66, 0xb2, 0x89, 0x99, 0xa9, 0x2d, 0x55, 0x7d, 0xb0, 0xb0, 0x8a, 0xa8, 0x96,
    0x36, 0x56, 0x76, 0xf1, 0xba, 0xa3, 0xb4, 0xb2, 0x80, 0xc0, 0xb8, 0xa8, 0x97, 0x11, 0xb2, 0x83,
    0x98, 0xba, 0xa3, 0xf0, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xb2, 0xb9, 0xb4, 0x98, 0x83, 0xf1,
    0xa3, 0x29, 0x55, 0x7d, 0xba, 0xb5, 0xb1, 0xa3, 0x83, 0x93, 0xf0, 0x00, 0x28, 0x50, 0xf5, 0xb2,
    0xb6, 0xaa, 0x83, 0x93, 0x28, 0x54, 0x7c, 0xf1, 0xb9, 0xa3, 0x82, 0x93, 0x61, 0xba, 0xa2, 0xda,
    0xde, 0xdf, 0xdb, 0x81, 0x9a, 0xb9, 0xae, 0xf5, 0x60, 0x68, 0x70, 0xf1, 0xda, 0xba, 0xa2, 0xdf,
    0xd9, 0xba, 0xa2, 0xfa, 0xb9, 0xa3, 0x82, 0x92, 0xdb, 0x31, 0xba, 0xa2, 0xd9, 0xba, 0xa2, 0xf8,
    0xdf, 0x85, 0xa4, 0xd0, 0xc1, 0xbb, 0xad, 0x83, 0xc2, 0xc5, 0xc7, 0xb8, 0xa2, 0xdf, 0xdf, 0xdf,
    0xba, 0xa0, 0xdf, 0xdf, 0xdf, 0xd8, 0xd8, 0xf1, 0xb8, 0xaa, 0xb3, 0x8d, 0xb4, 0x98, 0x0d, 0x35,
    0x5d, 0xb2, 0xb6, 0xba, 0xaf, 0x8c, 0x96, 0x19, 0x8f, 0x9f, 0xa7, 0x0e, 0x16, 0x1e, 0xb4, 0x9a,
    0xb8, 0xaa, 0x87, 0x2c, 0x54, 0x7c, 0xba, 0xa4, 0xb0, 0x8a, 0xb6, 0x91, 0x32, 0x56, 0x76, 0xb2,
    0x84, 0x94, 0xa4, 0xc8, 0x08, 0xcd, 0xd8, 0xb8, 0xb4, 0xb0, 0xf1, 0x99, 0x82, 0xa8, 0x2d, 0x55,
    0x7d, 0x98, 0xa8, 0x0e, 0x16, 0x1e, 0xa2, 0x2c, 0x54, 0x7c, 0x92, 0xa4, 0xf0, 0x2c, 0x50, 0x78,
    /* bank # 5 */
    0xf1, 0x84, 0xa8, 0x98, 0xc4, 0xcd, 0xfc, 0xd8, 0x0d, 0xdb, 0xa8, 0xfc, 0x2d, 0xf3, 0xd9, 0xba,
    0xa6, 0xf8, 0xda, 0xba, 0xa6, 0xde, 0xd8, 0xba, 0xb2, 0xb6, 0x86, 0x96, 0xa6, 0xd0, 0xf3, 0xc8,
    0x41, 0xda, 0xa6, 0xc8, 0xf8, 0xd8, 0xb0, 0xb4, 0xb8, 0x82, 0xa8, 0x92, 0xf5, 0x2c, 0x54, 0x88,
    0x98, 0xf1, 0x35, 0xd9, 0xf4, 0x18, 0xd8, 0xf1, 0xa2, 0xd0, 0xf8, 0xf9, 0xa8, 0x84, 0xd9, 0xc7,
    0xdf, 0xf8, 0xf8, 0x83, 0xc5, 0xda, 0xdf, 0x69, 0xdf, 0x83, 0xc1, 0xd8, 0xf4, 0x01, 0x14, 0xf1,
    0xa8, 0x82, 0x4e, 0xa8, 0x84, 0xf3, 0x11, 0xd1, 0x82, 0xf5, 0xd9, 0x92, 0x28, 0x97, 0x88, 0xf1,
    0x09, 0xf4, 0x1c, 0x1c, 0xd8, 0x84, 0xa8, 0xf3, 0xc0, 0xf9, 0xd1, 0xd9, 0x97, 0x82, 0xf1, 0x29,
    0xf4, 0x0d, 0xd8, 0xf3, 0xf9, 0xf9, 0xd1, 0xd9, 0x82, 0xf4, 0xc2, 0x03, 0xd8, 0xde, 0xdf, 0x1a,
    0xd8, 0xf1, 0xa2, 0xfa, 0xf9, 0xa8, 0x84, 0x98, 0xd9, 0xc7, 0xdf, 0xf8, 0xf8, 0xf8, 0x83, 0xc7,
    0xda, 0xdf, 0x69, 0xdf, 0xf8, 0x83, 0xc3, 0xd8, 0xf4, 0x01, 0x14, 0xf1, 0x98, 0xa8, 0x82, 0x2e,
    0xa8, 0x84, 0xf3, 0x11, 0xd1, 0x82, 0xf5, 0xd9, 0x92, 0x50, 0x97, 0x88, 0xf1, 0x09, 0xf4, 0x1c,
    0xd8, 0x84, 0xa8, 0xf3, 0xc0, 0xf8, 0xf9, 0xd1, 0xd9, 0x97, 0x82, 0xf1, 0x49, 0xf4, 0x0d, 0xd8,
    0xf3, 0xf9, 0xf9, 0xd1, 0xd9, 0x82, 0xf4, 0xc4, 0x03, 0xd8, 0xde, 0xdf, 0xd8, 0xf1, 0xad, 0x88,
    0x98, 0xcc, 0xa8, 0x09, 0xf9, 0xd9, 0x82, 0x92, 0xa8, 0xf5, 0x7c, 0xf1, 0x88, 0x3a, 0xcf, 0x94,
    0x4a, 0x6e, 0x98, 0xdb, 0x69, 0x31, 0xda, 0xad, 0xf2, 0xde, 0xf9, 0xd8, 0x87, 0x95, 0xa8, 0xf2,
    0x21, 0xd1, 0xda, 0xa5, 0xf9, 0xf4, 0x17, 0xd9, 0xf1, 0xae, 0x8e, 0xd0, 0xc0, 0xc3, 0xae, 0x82,
    /* bank # 6 */
    0xc6, 0x84, 0xc3, 0xa8, 0x85, 0x95, 0xc8, 0xa5, 0x88, 0xf2, 0xc0, 0xf1, 0xf4, 0x01, 0x0e, 0xf1,
    0x8e, 0x9e, 0xa8, 0xc6, 0x3e, 0x56, 0xf5, 0x54, 0xf1, 0x88, 0x72, 0xf4, 0x01, 0x15, 0xf1, 0x98,
    0x45, 0x85, 0x6e, 0xf5, 0x8e, 0x9e, 0x04, 0x88, 0xf1, 0x42, 0x98, 0x5a, 0x8e, 0x9e, 0x06, 0x88,
    0x69, 0xf4, 0x01, 0x1c, 0xf1, 0x98, 0x1e, 0x11, 0x08, 0xd0, 0xf5, 0x04, 0xf1, 0x1e, 0x97, 0x02,
    0x02, 0x98, 0x36, 0x25, 0xdb, 0xf9, 0xd9, 0x85, 0xa5, 0xf3, 0xc1, 0xda, 0x85, 0xa5, 0xf3, 0xdf,
    0xd8, 0x85, 0x95, 0xa8, 0xf3, 0x09, 0xda, 0xa5, 0xfa, 0xd8, 0x82, 0x92, 0xa8, 0xf5, 0x78, 0xf1,
    0x88, 0x1a, 0x84, 0x9f, 0x26, 0x88, 0x98, 0x21, 0xda, 0xf4, 0x1d, 0xf3, 0xd8, 0x87, 0x9f, 0x39,
    0xd1, 0xaf, 0xd9, 0xdf, 0xdf, 0xfb, 0xf9, 0xf4, 0x0c, 0xf3, 0xd8, 0xfa, 0xd0, 0xf8, 0xda, 0xf9,
    0xf9, 0xd0, 0xdf, 0xd9, 0xf9, 0xd8, 0xf4, 0x0b, 0xd8, 0xf3, 0x87, 0x9f, 0x39, 0xd1, 0xaf, 0xd9,
    0xdf, 0xdf, 0xf4, 0x1d, 0xf3, 0xd8, 0xfa, 0xfc, 0xa8, 0x69, 0xf9, 0xf9, 0xaf, 0xd0, 0xda, 0xde,
    0xfa, 0xd9, 0xf8, 0x8f, 0x9f, 0xa8, 0xf1, 0xcc, 0xf3, 0x98, 0xdb, 0x45, 0xd9, 0xaf, 0xdf, 0xd0,
    0xf8, 0xd8, 0xf1, 0x8f, 0x9f, 0xa8, 0xca, 0xf3, 0x88, 0x09, 0xda, 0xaf, 0x8f, 0xcb, 0xf8, 0xd8,
    0xf2, 0xad, 0x97, 0x8d, 0x0c, 0xd9, 0xa5, 0xdf, 0xf9, 0xba, 0xa6, 0xf3, 0xfa, 0xf4, 0x12, 0xf2,
    0xd8, 0x95, 0x0d, 0xd1, 0xd9, 0xba, 0xa6, 0xf3, 0xfa, 0xda, 0xa5, 0xf2, 0xc1, 0xba, 0xa6, 0xf3,
    0xdf, 0xd8, 0xf1, 0xba, 0xb2, 0xb6, 0x86, 0x96, 0xa6, 0xd0, 0xca, 0xf3, 0x49, 0xda, 0xa6, 0xcb,
    0xf8, 0xd8, 0xb0, 0xb4, 0xb8, 0xd8, 0xad, 0x84, 0xf2, 0xc0, 0xdf, 0xf1, 0x8f, 0xcb, 0xc3, 0xa8,
    /* bank # 7 */
    0xb2, 0xb6, 0x86, 0x96, 0xc8, 0xc1, 0xcb, 0xc3, 0xf3, 0xb0, 0xb4, 0x88, 0x98, 0xa8, 0x21, 0xdb,
    0x71, 0x8d, 0x9d, 0x71, 0x85, 0x95, 0x21, 0xd9, 0xad, 0xf2, 0xfa, 0xd8, 0x85, 0x97, 0xa8, 0x28,
    0xd9, 0xf4, 0x08, 0xd8, 0xf2, 0x8d, 0x29, 0xda, 0xf4, 0x05, 0xd9, 0xf2, 0x85, 0xa4, 0xc2, 0xf2,
    0xd8, 0xa8, 0x8d, 0x94, 0x01, 0xd1, 0xd9, 0xf4, 0x11, 0xf2, 0xd8, 0x87, 0x21, 0xd8, 0xf4, 0x0a,
    0xd8, 0xf2, 0x84, 0x98, 0xa8, 0xc8, 0x01, 0xd1, 0xd9, 0xf4, 0x11, 0xd8, 0xf3, 0xa4, 0xc8, 0xbb,
    0xaf, 0xd0, 0xf2, 0xde, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xd8, 0xf1, 0xb8, 0xf6,
    0xb5, 0xb9, 0xb0, 0x8a, 0x95, 0xa3, 0xde, 0x3c, 0xa3, 0xd9, 0xf8, 0xd8, 0x5c, 0xa3, 0xd9, 0xf8,
    0xd8, 0x7c, 0xa3, 0xd9, 0xf8, 0xd8, 0xf8, 0xf9, 0xd1, 0xa5, 0xd9, 0xdf, 0xda, 0xfa, 0xd8, 0xb1,
    0x85, 0x30, 0xf7, 0xd9, 0xde, 0xd8, 0xf8, 0x30, 0xad, 0xda, 0xde, 0xd8, 0xf2, 0xb4, 0x8c, 0x99,
    0xa3, 0x2d, 0x55, 0x7d, 0xa0, 0x83, 0xdf, 0xdf, 0xdf, 0xb5, 0x91, 0xa0, 0xf6, 0x29, 0xd9, 0xfb,
    0xd8, 0xa0, 0xfc, 0x29, 0xd9, 0xfa, 0xd8, 0xa0, 0xd0, 0x51, 0xd9, 0xf8, 0xd8, 0xfc, 0x51, 0xd9,
    0xf9, 0xd8, 0x79, 0xd9, 0xfb, 0xd8, 0xa0, 0xd0, 0xfc, 0x79, 0xd9, 0xfa, 0xd8, 0xa1, 0xf9, 0xf9,
    0xf9, 0xf9, 0xf9, 0xa0, 0xda, 0xdf, 0xdf, 0xdf, 0xd8, 0xa1, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xac,
    0xde, 0xf8, 0xad, 0xde, 0x83, 0x93, 0xac, 0x2c, 0x54, 0x7c, 0xf1, 0xa8, 0xdf, 0xdf, 0xdf, 0xf6,
    0x9d, 0x2c, 0xda, 0xa0, 0xdf, 0xd9, 0xfa, 0xdb, 0x2d, 0xf8, 0xd8, 0xa8, 0x50, 0xda, 0xa0, 0xd0,
    0xde, 0xd9, 0xd0, 0xf8, 0xf8, 0xf8, 0xdb, 0x55, 0xf8, 0xd8, 0xa8, 0x78, 0xda, 0xa0, 0xd0, 0xdf,
    /* bank # 8 */
    0xd9, 0xd0, 0xfa, 0xf8, 0xf8, 0xf8, 0xf8, 0xdb, 0x7d, 0xf8, 0xd8, 0x9c, 0xa8, 0x8c, 0xf5, 0x30,
    0xdb, 0x38, 0xd9, 0xd0, 0xde, 0xdf, 0xa0, 0xd0, 0xde, 0xdf, 0xd8, 0xa8, 0x48, 0xdb, 0x58, 0xd9,
    0xdf, 0xd0, 0xde, 0xa0, 0xdf, 0xd0, 0xde, 0xd8, 0xa8, 0x68, 0xdb, 0x70, 0xd9, 0xdf, 0xdf, 0xa0,
    0xdf, 0xdf, 0xd8, 0xf1, 0xa8, 0x88, 0x90, 0x2c, 0x54, 0x7c, 0x98, 0xa8, 0xd0, 0x5c, 0x38, 0xd1,
    0xda, 0xf2, 0xae, 0x8c, 0xdf, 0xf9, 0xd8, 0xb0, 0x87, 0xa8, 0xc1, 0xc1, 0xb1, 0x88, 0xa8, 0xc6,
    0xf9, 0xf9, 0xda, 0x36, 0xd8, 0xa8, 0xf9, 0xda, 0x36, 0xd8, 0xa8, 0xf9, 0xda, 0x36, 0xd8, 0xa8,
    0xf9, 0xda, 0x36, 0xd8, 0xa8, 0xf9, 0xda, 0x36, 0xd8, 0xf7, 0x8d, 0x9d, 0xad, 0xf8, 0x18, 0xda,
    0xf2, 0xae, 0xdf, 0xd8, 0xf7, 0xad, 0xfa, 0x30, 0xd9, 0xa4, 0xde, 0xf9, 0xd8, 0xf2, 0xae, 0xde,
    0xfa, 0xf9, 0x83, 0xa7, 0xd9, 0xc3, 0xc5, 0xc7, 0xf1, 0x88, 0x9b, 0xa7, 0x7a, 0xad, 0xf7, 0xde,
    0xdf, 0xa4, 0xf8, 0x84, 0x94, 0x08, 0xa7, 0x97, 0xf3, 0x00, 0xae, 0xf2, 0x98, 0x19, 0xa4, 0x88,
    0xc6, 0xa3, 0x94, 0x88, 0xf6, 0x32, 0xdf, 0xf2, 0x83, 0x93, 0xdb, 0x09, 0xd9, 0xf2, 0xaa, 0xdf,
    0xd8, 0xd8, 0xae, 0xf8, 0xf9, 0xd1, 0xda, 0xf3, 0xa4, 0xde, 0xa7, 0xf1, 0x88, 0x9b, 0x7a, 0xd8,
    0xf3, 0x84, 0x94, 0xae, 0x19, 0xf9, 0xda, 0xaa, 0xf1, 0xdf, 0xd8, 0xa8, 0x81, 0xc0, 0xc3, 0xc5,
    0xc7, 0xa3, 0x92, 0x83, 0xf6, 0x28, 0xad, 0xde, 0xd9, 0xf8, 0xd8, 0xa3, 0x50, 0xad, 0xd9, 0xf8,
    0xd8, 0xa3, 0x78, 0xad, 0xd9, 0xf8, 0xd8, 0xf8, 0xf9, 0xd1, 0xa1, 0xda, 0xde, 0xc3, 0xc5, 0xc7,
    0xd8, 0xa1, 0x81, 0x94, 0xf8, 0x18, 0xf2, 0xb0, 0x89, 0xac, 0xc3, 0xc5, 0xc7, 0xf1, 0xd8, 0xb8,
    /* bank # 9 */
    0xb4, 0xb0, 0x97, 0x86, 0xa8, 0x31, 0x9b, 0x06, 0x99, 0x07, 0xab, 0x97, 0x28, 0x88, 0x9b, 0xf0,
    0x0c, 0x20, 0x14, 0x40, 0xb0, 0xb4, 0xb8, 0xf0, 0xa8, 0x8a, 0x9a, 0x28, 0x50, 0x78, 0xb7, 0x9b,
    0xa8, 0x29, 0x51, 0x79, 0x24, 0x70, 0x59, 0x44, 0x69, 0x38, 0x64, 0x48, 0x31, 0xf1, 0xbb, 0xab,
    0x88, 0x00, 0x2c, 0x54, 0x7c, 0xf0, 0xb3, 0x8b, 0xb8, 0xa8, 0x04, 0x28, 0x50, 0x78, 0xf1, 0xb0,
    0x88, 0xb4, 0x97, 0x26, 0xa8, 0x59, 0x98, 0xbb, 0xab, 0xb3, 0x8b, 0x02, 0x26, 0x46, 0x66, 0xb0,
    0xb8, 0xf0, 0x8a, 0x9c, 0xa8, 0x29, 0x51, 0x79, 0x8b, 0x29, 0x51, 0x79, 0x8a, 0x24, 0x70, 0x59,
    0x8b, 0x20, 0x58, 0x71, 0x8a, 0x44, 0x69, 0x38, 0x8b, 0x39, 0x40, 0x68, 0x8a, 0x64, 0x48, 0x31,
    0x8b, 0x30, 0x49, 0x60, 0x88, 0xf1, 0xac, 0x00, 0x2c, 0x54, 0x7c, 0xf0, 0x8c, 0xa8, 0x04, 0x28,
    0x50, 0x78, 0xf1, 0x88, 0x97, 0x26, 0xa8, 0x59, 0x98, 0xac, 0x8c, 0x02, 0x26, 0x46, 0x66, 0xf0,
    0x89, 0x9c, 0xa8, 0x29, 0x51, 0x79, 0x24, 0x70, 0x59, 0x44, 0x69, 0x38, 0x64, 0x48, 0x31, 0xa9,
    0x88, 0x09, 0x20, 0x59, 0x70, 0xab, 0x11, 0x38, 0x40, 0x69, 0xa8, 0x19, 0x31, 0x48, 0x60, 0x8c,
    0xa8, 0x3c, 0x41, 0x5c, 0x20, 0x7c, 0x00, 0xf1, 0x87, 0x98, 0x19, 0x86, 0xa8, 0x6e, 0x76, 0x7e,
    0xa9, 0x99, 0x88, 0x2d, 0x55, 0x7d, 0xd8, 0xb1, 0xb5, 0xb9, 0xa3, 0xdf, 0xdf, 0xdf, 0xae, 0xd0,
    0xdf, 0xaa, 0xd0, 0xde, 0xf2, 0xab, 0xf8, 0xf9, 0xd9, 0xb0, 0x87, 0xc4, 0xaa, 0xf1, 0xdf, 0xdf,
    0xbb, 0xaf, 0xdf, 0xdf, 0xb9, 0xd8, 0xb1, 0xf1, 0xa3, 0x97, 0x8e, 0x60, 0xdf, 0xb0, 0x84, 0xf2,
    0xc8, 0xf8, 0xf9, 0xd9, 0xde, 0xd8, 0x93, 0x85, 0xf1, 0x4a, 0xb1, 0x83, 0xa3, 0x08, 0xb5, 0x83,
    /* bank # 10 */
    0x9a, 0x08, 0x10, 0xb7, 0x9f, 0x10, 0xd8, 0xf1, 0xb0, 0xba, 0xae, 0xb0, 0x8a, 0xc2, 0xb2, 0xb6,
    0x8e, 0x9e, 0xf1, 0xfb, 0xd9, 0xf4, 0x1d, 0xd8, 0xf9, 0xd9, 0x0c, 0xf1, 0xd8, 0xf8, 0xf8, 0xad,
    0x61, 0xd9, 0xae, 0xfb, 0xd8, 0xf4, 0x0c, 0xf1, 0xd8, 0xf8, 0xf8, 0xad, 0x19, 0xd9, 0xae, 0xfb,
    0xdf, 0xd8, 0xf4, 0x16, 0xf1, 0xd8, 0xf8, 0xad, 0x8d, 0x61, 0xd9, 0xf4, 0xf4, 0xac, 0xf5, 0x9c,
    0x9c, 0x8d, 0xdf, 0x2b, 0xba, 0xb6, 0xae, 0xfa, 0xf8, 0xf4, 0x0b, 0xd8, 0xf1, 0xae, 0xd0, 0xf8,
    0xad, 0x51, 0xda, 0xae, 0xfa, 0xf8, 0xf1, 0xd8, 0xb9, 0xb1, 0xb6, 0xa3, 0x83, 0x9c, 0x08, 0xb9,
    0xb1, 0x83, 0x9a, 0xb5, 0xaa, 0xc0, 0xfd, 0x30, 0x83, 0xb7, 0x9f, 0x10, 0xb5, 0x8b, 0x93, 0xf2,
    0x02, 0x02, 0xd1, 0xab, 0xda, 0xde, 0xd8, 0xf1, 0xb0, 0x80, 0xba, 0xab, 0xc0, 0xc3, 0xb2, 0x84,
    0xc1, 0xc3, 0xd8, 0xb1, 0xb9, 0xf3, 0x8b, 0xa3, 0x91, 0xb6, 0x09, 0xb4, 0xd9, 0xab, 0xde, 0xb0,
    0x87, 0x9c, 0xb9, 0xa3, 0xdd, 0xf1, 0xb3, 0x8b, 0x8b, 0x8b, 0x8b, 0x8b, 0xb0, 0x87, 0xa3, 0xa3,
    0xa3, 0xa3, 0xb2, 0x8b, 0xb6, 0x9b, 0xf2, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3,
    0xa3, 0xf1, 0xb0, 0x87, 0xb5, 0x9a, 0xa3, 0xf3, 0x9b, 0xa3, 0xa3, 0xdc, 0xba, 0xac, 0xdf, 0xb9,
    0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3,
    0xd8, 0xd8, 0xd8, 0xbb, 0xb3, 0xb7, 0xf1, 0xaa, 0xf9, 0xda, 0xff, 0xd9, 0x80, 0x9a, 0xaa, 0x28,
    0xb4, 0x80, 0x98, 0xa7, 0x20, 0xb7, 0x97, 0x87, 0xa8, 0x66, 0x88, 0xf0, 0x79, 0x51, 0xf1, 0x90,
    0x2c, 0x87, 0x0c, 0xa7, 0x81, 0x97, 0x62, 0x93, 0xf0, 0x71, 0x71, 0x60, 0x85, 0x94, 0x01, 0x29,
    /* bank # 11 */
    0x51, 0x79, 0x90, 0xa5, 0xf1, 0x28, 0x4c, 0x6c, 0x87, 0x0c, 0x95, 0x18, 0x85, 0x78, 0xa3, 0x83,
    0x90, 0x28, 0x4c, 0x6c, 0x88, 0x6c, 0xd8, 0xf3, 0xa2, 0x82, 0x00, 0xf2, 0x10, 0xa8, 0x92, 0x19,
    0x80, 0xa2, 0xf2, 0xd9, 0x26, 0xd8, 0xf1, 0x88, 0xa8, 0x4d, 0xd9, 0x48, 0xd8, 0x96, 0xa8, 0x39,
    0x80, 0xd9, 0x3c, 0xd8, 0x95, 0x80, 0xa8, 0x39, 0xa6, 0x86, 0x98, 0xd9, 0x2c, 0xda, 0x87, 0xa7,
    0x2c, 0xd8, 0xa8, 0x89, 0x95, 0x19, 0xa9, 0x80, 0xd9, 0x38, 0xd8, 0xa8, 0x89, 0x39, 0xa9, 0x80,
    0xda, 0x3c, 0xd8, 0xa8, 0x2e, 0xa8, 0x39, 0x90, 0xd9, 0x0c, 0xd8, 0xa8, 0x95, 0x31, 0x98, 0xd9,
    0x0c, 0xd8, 0xa8, 0x09, 0xd9, 0xff, 0xd8, 0x01, 0xda, 0xff, 0xd8, 0x95, 0x39, 0xa9, 0xda, 0x26,
    0xff, 0xd8, 0x90, 0xa8, 0x0d, 0x89, 0x99, 0xa8, 0x10, 0x80, 0x98, 0x21, 0xda, 0x2e, 0xd8, 0x89,
    0x99, 0xa8, 0x31, 0x80, 0xda, 0x2e, 0xd8, 0xa8, 0x86, 0x96, 0x31, 0x80, 0xda, 0x2e, 0xd8, 0xa8,
    0x87, 0x31, 0x80, 0xda, 0x2e, 0xd8, 0xa8, 0x82, 0x92, 0xf3, 0x41, 0x80, 0xf1, 0xd9, 0x2e, 0xd8,
    0xa8, 0x82, 0xf3, 0x19, 0x80, 0xf1, 0xd9, 0x2e, 0xd8, 0x82, 0xac, 0xf3, 0xc0, 0xa2, 0x80, 0x22,
    0xf1, 0xa6, 0x2e, 0xa7, 0x2e, 0xa9, 0x22, 0x98, 0xa8, 0x29, 0xda, 0xac, 0xde, 0xff, 0xd8, 0xa2,
    0xf2, 0x2a, 0xf1, 0xa9, 0x2e, 0x82, 0x92, 0xa8, 0xf2, 0x31, 0x80, 0xa6, 0x96, 0xf1, 0xd9, 0x00,
    0xac, 0x8c, 0x9c, 0x0c, 0x30, 0xac, 0xde, 0xd0, 0xde, 0xff, 0xd8, 0x8c, 0x9c, 0xac, 0xd0, 0x10,
    0xac, 0xde, 0x80, 0x92, 0xa2, 0xf2, 0x4c, 0x82, 0xa8, 0xf1, 0xca, 0xf2, 0x35, 0xf1, 0x96, 0x88,
    0xa6, 0xd9, 0x00, 0xd8, 0xf1, 0xff
};
static const unsigned short sStartAddress = 0x0400;
/* END OF SECTION COPIED FROM dmpDefaultMPU6050.c */
#define DMP_FEATURE_SEND_ANY_GYRO   (DMP_FEATURE_SEND_RAW_GYRO | \
                                     DMP_FEATURE_SEND_CAL_GYRO)
#define DMP_SMPL_RATE     (200)
#define GYRO_SF             (46850825LL * 200 / DMP_SMPL_RATE)
#define MAX_FIFO_LENGTH (32)
#define MAX_FIFO        (1024)

// DMP MASKS
#define DMP_FEATURE_TAP             (0x001)
#define DMP_FEATURE_ANDROID_ORIENT  (0x002)
#define DMP_FEATURE_LP_QUAT         (0x004)
#define DMP_FEATURE_PEDOMETER       (0x008)
#define DMP_FEATURE_6X_LP_QUAT      (0x010)
#define DMP_FEATURE_GYRO_CAL        (0x020)
#define DMP_FEATURE_SEND_RAW_ACCEL  (0x040)
#define DMP_FEATURE_SEND_RAW_GYRO   (0x080)
#define DMP_FEATURE_SEND_CAL_GYRO   (0x100)

//DMP Key
#define DINA0A 0x0a
#define DINA22 0x22
#define DINA42 0x42
#define DINA5A 0x5a

#define DINA06 0x06
#define DINA0E 0x0e
#define DINA16 0x16
#define DINA1E 0x1e
#define DINA26 0x26
#define DINA2E 0x2e
#define DINA36 0x36
#define DINA3E 0x3e
#define DINA46 0x46
#define DINA4E 0x4e
#define DINA56 0x56
#define DINA5E 0x5e
#define DINA66 0x66
#define DINA6E 0x6e
#define DINA76 0x76
#define DINA7E 0x7e

#define DINA00 0x00
#define DINA08 0x08
#define DINA10 0x10
#define DINA18 0x18
#define DINA20 0x20
#define DINA28 0x28
#define DINA30 0x30
#define DINA38 0x38
#define DINA40 0x40
#define DINA48 0x48
#define DINA50 0x50
#define DINA58 0x58
#define DINA60 0x60
#define DINA68 0x68
#define DINA70 0x70
#define DINA78 0x78

#define DINA04 0x04
#define DINA0C 0x0c
#define DINA14 0x14
#define DINA1C 0x1C
#define DINA24 0x24
#define DINA2C 0x2c
#define DINA34 0x34
#define DINA3C 0x3c
#define DINA44 0x44
#define DINA4C 0x4c
#define DINA54 0x54
#define DINA5C 0x5c
#define DINA64 0x64
#define DINA6C 0x6c
#define DINA74 0x74
#define DINA7C 0x7c

#define DINA01 0x01
#define DINA09 0x09
#define DINA11 0x11
#define DINA19 0x19
#define DINA21 0x21
#define DINA29 0x29
#define DINA31 0x31
#define DINA39 0x39
#define DINA41 0x41
#define DINA49 0x49
#define DINA51 0x51
#define DINA59 0x59
#define DINA61 0x61
#define DINA69 0x69
#define DINA71 0x71
#define DINA79 0x79

#define DINA25 0x25
#define DINA2D 0x2d
#define DINA35 0x35
#define DINA3D 0x3d
#define DINA4D 0x4d
#define DINA55 0x55
#define DINA5D 0x5D
#define DINA6D 0x6d
#define DINA75 0x75
#define DINA7D 0x7d

#define DINADC 0xdc
#define DINAF2 0xf2
#define DINAAB 0xab
#define DINAAA 0xaa
#define DINAF1 0xf1
#define DINADF 0xdf
#define DINADA 0xda
#define DINAB1 0xb1
#define DINAB9 0xb9
#define DINAF3 0xf3
#define DINA8B 0x8b
#define DINAA3 0xa3
#define DINA91 0x91
#define DINAB6 0xb6
#define DINAB4 0xb4


#define DINC00 0x00
#define DINC01 0x01
#define DINC02 0x02
#define DINC03 0x03
#define DINC08 0x08
#define DINC09 0x09
#define DINC0A 0x0a
#define DINC0B 0x0b
#define DINC10 0x10
#define DINC11 0x11
#define DINC12 0x12
#define DINC13 0x13
#define DINC18 0x18
#define DINC19 0x19
#define DINC1A 0x1a
#define DINC1B 0x1b

#define DINC20 0x20
#define DINC21 0x21
#define DINC22 0x22
#define DINC23 0x23
#define DINC28 0x28
#define DINC29 0x29
#define DINC2A 0x2a
#define DINC2B 0x2b
#define DINC30 0x30
#define DINC31 0x31
#define DINC32 0x32
#define DINC33 0x33
#define DINC38 0x38
#define DINC39 0x39
#define DINC3A 0x3a
#define DINC3B 0x3b

#define DINC40 0x40
#define DINC41 0x41
#define DINC42 0x42
#define DINC43 0x43
#define DINC48 0x48
#define DINC49 0x49
#define DINC4A 0x4a
#define DINC4B 0x4b
#define DINC50 0x50
#define DINC51 0x51
#define DINC52 0x52
#define DINC53 0x53
#define DINC58 0x58
#define DINC59 0x59
#define DINC5A 0x5a
#define DINC5B 0x5b

#define DINC60 0x60
#define DINC61 0x61
#define DINC62 0x62
#define DINC63 0x63
#define DINC68 0x68
#define DINC69 0x69
#define DINC6A 0x6a
#define DINC6B 0x6b
#define DINC70 0x70
#define DINC71 0x71
#define DINC72 0x72
#define DINC73 0x73
#define DINC78 0x78
#define DINC79 0x79
#define DINC7A 0x7a
#define DINC7B 0x7b

#define DIND40 0x40


#define DINA80 0x80
#define DINA90 0x90
#define DINAA0 0xa0
#define DINAC9 0xc9
#define DINACB 0xcb
#define DINACD 0xcd
#define DINACF 0xcf
#define DINAC8 0xc8
#define DINACA 0xca
#define DINACC 0xcc
#define DINACE 0xce
#define DINAD8 0xd8
#define DINADD 0xdd
#define DINAF8 0xf0
#define DINAFE 0xfe

#define DINBF8 0xf8
#define DINAC0 0xb0
#define DINAC1 0xb1
#define DINAC2 0xb4
#define DINAC3 0xb5
#define DINAC4 0xb8
#define DINAC5 0xb9
#define DINBC0 0xc0
#define DINBC2 0xc2
#define DINBC4 0xc4
#define DINBC6 0xc6

/*---------------------------------------------------------------------------*/
#define MPU_AX_MAG        6
/*---------------------------------------------------------------------------*/
#define MPU_DATA_READY    0x01
#define MPU_MOVEMENT      0x40
/*---------------------------------------------------------------------------*/
/* Sensor selection/deselection */
#define SENSOR_SELECT()     board_i2c_select(BOARD_I2C_INTERFACE_1, SENSOR_I2C_ADDRESS)
#define SENSOR_DESELECT()   board_i2c_deselect()

#define MAG_SENSOR_SELECT()   board_i2c_select(BOARD_I2C_INTERFACE_1, SENSOR_MAG_I2_ADDRESS)
#define MAG_SENSOR_DESELECT() SENSOR_DESELECT()
/*---------------------------------------------------------------------------*/
/* Delay */
#define delay_ms(i) (ti_lib_cpu_delay(8000 * (i)))
/*---------------------------------------------------------------------------*/
static uint8_t val;
static uint8_t interrupt_status;
static struct mpu_config_s mpu_config;
static struct mpu_reg_s mpu_reg;
/*---------------------------------------------------------------------------*/
#define SENSOR_STATE_DISABLED     0
#define SENSOR_STATE_BOOTING      1
#define SENSOR_STATE_ENABLED      2

static int state = SENSOR_STATE_DISABLED;
static int elements = MPU_9250_SENSOR_TYPE_NONE;
/*---------------------------------------------------------------------------*/
/* 3 16-byte words for all sensor readings */
#define SENSOR_DATA_BUF_SIZE       3
#define ALL_SENSOR_DATA_BUF_SIZE   8 // 3 byte accel, 2 dieTemp, 3 gyro

static uint16_t sensor_value[ALL_SENSOR_DATA_BUF_SIZE];
/*---------------------------------------------------------------------------*/
/*
 * Wait SENSOR_BOOT_DELAY ticks for the sensor to boot and
 * SENSOR_STARTUP_DELAY for readings to be ready
 * Gyro is a little slower than Acc
 */
#define SENSOR_BOOT_DELAY     40 // TODO: Change back to 4
#define SENSOR_STARTUP_DELAY  5

static struct ctimer startup_timer;
/*---------------------------------------------------------------------------*/
/* Wait for the MPU to have data ready */
rtimer_clock_t t0;

/*
 * Wait timeout in rtimer ticks. This is just a random low number, since the
 * first time we read the sensor status, it should be ready to return data
 */
#define READING_WAIT_TIMEOUT 10
/*---------------------------------------------------------------------------*/
/**
 * \brief Place the MPU in low power mode
 */

#define min(a,b) \
  ({ __typeof__ (a) _a = (a); \
      __typeof__ (b) _b = (b); \
    _a > _b ? _b : _a; })

static bool mpu_init();
static void
sensor_sleep(void)
{
  SENSOR_SELECT();

  val = DISABLE_ALL_AXES;
  sensor_common_write_reg(PWR_MGMT_2, &val, 1);

  val = MPU_SLEEP;
  sensor_common_write_reg(PWR_MGMT_1, &val, 1);
  SENSOR_DESELECT();
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Exit low power mode
 */
//static void
//sensor_wakeup(void)
//{
//  SENSOR_SELECT();
//  val = MPU_WAKE_UP;
//  sensor_common_write_reg(PWR_MGMT_1, &val, 1);
//
//  /* All axis initially disabled */
//  val = EN_ALL_AXES;
//  sensor_common_write_reg(PWR_MGMT_2, &val, 1);
//  mpu_config = 0;
//
//  /* Restore the range */
//  sensor_common_write_reg(ACCEL_CONFIG, &acc_range_reg, 1);
//
//  /* Clear interrupts */
//  sensor_common_read_reg(INT_STATUS, &val, 1);
//  SENSOR_DESELECT();
//}

/*---------------------------------------------------------------------------*/
static void
convert_to_le(uint8_t *data, uint8_t len)
{
  int i;
  for(i = 0; i < len; i += 2) {
    uint8_t tmp;
    tmp = data[i];
    data[i] = data[i + 1];
    data[i + 1] = tmp;
  }
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Set the range of the accelerometer
 * \param new_range: ACC_RANGE_2G, ACC_RANGE_4G, ACC_RANGE_8G, ACC_RANGE_16G
 * \return true if the write to the sensor succeeded
 */
static bool
acc_set_range(uint8_t new_range) {
  PRINTF("acc_set_range begin\r\n");
  bool success;

  if(new_range == mpu_config.acc_range) {
    //range has already been set
    return true;
  }

  //extracting everything except the range bits from register
  val = mpu_reg.REG_ACCEL_CONFIG & 0xE7;
  //adding new range to the register
  val |= (new_range << 3);

  /* Apply the range */
  SENSOR_SELECT();
  success = sensor_common_write_reg(ACCEL_CONFIG, &val, 1);
  SENSOR_DESELECT();

  if(success == false) {
    PRINTF("ERROR: Failed while writing to ACCEL_CONFIG register\r\n");
    return false;
  }

  //updating local copy of config and register
  mpu_config.acc_range = new_range;
  mpu_reg.REG_ACCEL_CONFIG = val;
  PRINTF("acc_set_range end\r\n");
  return success;
}

/**
 *  @brief      Set digital low pass filter.
 *  The following LPF settings are supported: 188, 98, 42, 20, 10, 5.
 *  @param[in]  lpf Desired LPF setting.
 *  @return     true if successful.
 */
static bool
set_gyro_lpf(uint8_t new_lpf)
{
  PRINTF("set_gyro_lpf\r\n");
  bool success;
  if(new_lpf == mpu_config.lpf) {
    //lpf has already been set
    return true;
  }

  //extracting everything except the CLPF_CFG bits from register
  val = mpu_reg.REG_CONFIG & 0xFC;
  //adding new range to the register
  val |= new_lpf;

  /* Apply the range */
  SENSOR_SELECT();
  success = sensor_common_write_reg(CONFIG, &val, 1);
  SENSOR_DESELECT();

  if(success  == false) {
    PRINTF("ERROR: Failed while writing to CONFIG register\r\n");
    return false;
  }

  //updating local copy of config and register
  mpu_config.lpf = new_lpf;
  mpu_reg.REG_CONFIG = val;
  return success;
}

/*---------------------------------------------------------------------------*/
/**
 * \brief Check whether a data or wake on motion interrupt has occurred
 * \return Return the interrupt status
 *
 * This driver does not use interrupts, however this function allows us to
 * determine whether a new sensor reading is available
 */
static uint8_t
int_status(void)
{
  SENSOR_SELECT();
  sensor_common_read_reg(INT_STATUS, &interrupt_status, 1);
  SENSOR_DESELECT();

  return interrupt_status;
}

/*---------------------------------------------------------------------------*/
/**
 * \brief Read data from the accelerometer - X, Y, Z - 3 words
 * \return True if a valid reading could be taken, false otherwise
 */
static bool
acc_read(uint16_t *data)
{
  bool success;
  if(interrupt_status & BIT_RAW_RDY_EN) {
    /* Burst read of all accelerometer values */
    SENSOR_SELECT();
    success = sensor_common_read_reg(ACCEL_XOUT_H, (uint8_t *)data, DATA_SIZE);
    SENSOR_DESELECT();

    if(success) {
      convert_to_le((uint8_t *)data, DATA_SIZE);
    } else {
      sensor_common_set_error_data((uint8_t *)data, DATA_SIZE);
    }
  } else {
    /* Data not ready */
    success = false;
  }

  return success;
}

/*---------------------------------------------------------------------------*/
/**
 * \brief Read data from the accel and gyro data - 3 words
 * \return True if a valid reading could be taken, false otherwise
 */
static bool
mpu_read(uint16_t *data)
{
  bool success;
  if(interrupt_status & BIT_RAW_RDY_EN) {
    /* Burst read of all accelerometer values */
    SENSOR_SELECT();
    success = sensor_common_read_reg(ACCEL_XOUT_H, (uint8_t *)data, MPU_DATA_SIZE);
    SENSOR_DESELECT();

    if(success) {
      convert_to_le((uint8_t *)data, MPU_DATA_SIZE);
    } else {
      PRINTF("MPU: Failed to read mpu data\r\n");
      sensor_common_set_error_data((uint8_t *)data, MPU_DATA_SIZE);
    }
  } else {
    /* Data not ready */
    PRINTF("MPU: MPU data not ready to be read\r\n");
    success = false;
  }

  return success;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Read data from the gyroscope - X, Y, Z - 3 words
 * \return True if a valid reading could be taken, false otherwise
 */
static bool
gyro_read(uint16_t *data)
{
  bool success;
  if(interrupt_status & BIT_RAW_RDY_EN) {
    /* Select this sensor */
    SENSOR_SELECT();

    /* Burst read of all gyroscope values */
    success = sensor_common_read_reg(GYRO_XOUT_H, (uint8_t *)data, DATA_SIZE);

    if(success) {
      convert_to_le((uint8_t *)data, DATA_SIZE);
    } else {
      sensor_common_set_error_data((uint8_t *)data, DATA_SIZE);
    }

    SENSOR_DESELECT();
  } else {
    success = false;
  }

  return success;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Convert accelerometer raw reading to a value in G
 * \param raw_data The raw accelerometer reading
 * \return The converted value
 */
static float
acc_convert(int16_t raw_data)
{
  float v = 0;

  switch(mpu_config.acc_range) {
  case MPU9250_ACCEL_RANGE_2G:
    /* Calculate acceleration, unit mG, range -2, +2 */
    v = (raw_data * 1000.0 / (32768 >> 1));
    break;
  case MPU9250_ACCEL_RANGE_4G:
    /* Calculate acceleration, unit mG, range -4, +4 */
    v = (raw_data * 1000.0) / (32768 >> 2);
    break;
  case MPU9250_ACCEL_RANGE_8G:
    /* Calculate acceleration, unit mG, range -8, +8 */
    v = (raw_data * 1000.0) / (32768 >> 3);
    break;
  case MPU9250_ACCEL_RANGE_16G:
    /* Calculate acceleration, unit mG, range -16, +16 */
    v = (raw_data * 1000.0) / (32768 >> 4);
    break;
  default:
    v = 0;
    break;
  }

  return v;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Convert gyro raw reading to a value in deg/sec
 * \param raw_data The raw accelerometer reading
 * \return The converted value
 */
static float
gyro_convert(int16_t raw_data)
{
  /* calculate rotation, unit mdeg/s, range -250, +250 */
  return (raw_data * 1000.0) / (65536 / 500);
}
/*---------------------------------------------------------------------------*/
static void
notify_ready(void *not_used)
{
  state = SENSOR_STATE_ENABLED;
  sensors_changed(&mpu_9250_sensor);
}
/*---------------------------------------------------------------------------*/
static void initialise(void *not_used) {
  bool success;

  success = mpu_init();
  if(success == false) {
    PRINTF("MPU: Failed to init mpu\r\n");
  }
  ctimer_set(&startup_timer, SENSOR_STARTUP_DELAY, notify_ready, NULL);
}
/*---------------------------------------------------------------------------*/
static void
power_up(void)
{
  ti_lib_gpio_set_dio(BOARD_IOID_MPU_POWER);
  state = SENSOR_STATE_BOOTING;

  ctimer_set(&startup_timer, SENSOR_BOOT_DELAY, initialise, NULL);
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Returns a reading from the sensor
 * \param type MPU_9250_SENSOR_TYPE_ACC_[XYZ] or MPU_9250_SENSOR_TYPE_GYRO_[XYZ]
 * \return centi-G (ACC) or centi-Deg/Sec (Gyro)
 */
static int
value(int type)
{
  int rv;
  float converted_val = 0;

  if(state == SENSOR_STATE_DISABLED) {
    PRINTF("MPU: Sensor Disabled\r\n");
    return CC26XX_SENSOR_READING_ERROR;
  }

  memset(sensor_value, 0, sizeof(sensor_value));

  if((type & MPU_9250_SENSOR_TYPE_ACC) != 0) {
    t0 = RTIMER_NOW();

    while(!int_status() &&
          (RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + READING_WAIT_TIMEOUT)));

    rv = acc_read(sensor_value);

    if(rv == 0) {
      return CC26XX_SENSOR_READING_ERROR;
    }

    PRINTF("MPU: ACC = 0x%04x 0x%04x 0x%04x = ",
           sensor_value[0], sensor_value[1], sensor_value[2]);

    /* Convert */
    if(type == MPU_9250_SENSOR_TYPE_ACC_X) {
      converted_val = acc_convert(sensor_value[0]);
    } else if(type == MPU_9250_SENSOR_TYPE_ACC_Y) {
      converted_val = acc_convert(sensor_value[1]);
    } else if(type == MPU_9250_SENSOR_TYPE_ACC_Z) {
      converted_val = acc_convert(sensor_value[2]);
    }
    rv = (int)(converted_val * 100);
  } else if((type & MPU_9250_SENSOR_TYPE_GYRO) != 0) {
    t0 = RTIMER_NOW();

    while(!int_status() &&
          (RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + READING_WAIT_TIMEOUT)));

    rv = gyro_read(sensor_value);

    if(rv == 0) {
      return CC26XX_SENSOR_READING_ERROR;
    }

    PRINTF("MPU: Gyro = 0x%04x 0x%04x 0x%04x = ",
           sensor_value[0], sensor_value[1], sensor_value[2]);

    if(type == MPU_9250_SENSOR_TYPE_GYRO_X) {
      converted_val = gyro_convert(sensor_value[0]);
    } else if(type == MPU_9250_SENSOR_TYPE_GYRO_Y) {
      converted_val = gyro_convert(sensor_value[1]);
    } else if(type == MPU_9250_SENSOR_TYPE_GYRO_Z) {
      converted_val = gyro_convert(sensor_value[2]);
    }
    rv = (int)(converted_val * 100);
  } else {
    PRINTF("MPU: Invalid type\n");
    rv = CC26XX_SENSOR_READING_ERROR;
  }

  PRINTF("%ld\n", (long int)(converted_val * 100));

  return rv;
}


/*---------------------------------------------------------------------------*/
/**
 * \brief Returns the status of the sensor
 * \param type SENSORS_ACTIVE or SENSORS_READY
 * \return 1 if the sensor is enabled
 */
static int
status(int type)
{
  switch(type) {
  case SENSORS_ACTIVE:
  case SENSORS_READY:
    return state;
    break;
  default:
    break;
  }
  return SENSOR_STATE_DISABLED;
}
/**
 *  @brief      Set the gyro full-scale range.
 *  @param[in]  fsr Desired full-scale range.
 *  @return     0 if successful.
 */
static bool gyro_set_range(uint8_t new_range) {
  PRINTF("gyro_set_range start\r\n");
  bool success;

  if(new_range == mpu_config.gyro_range) {
    //range has already been set
    PRINTF("Range already set\r\n");
    return true;
  }

  //extracting everything except the range bits from register
  val = mpu_reg.REG_GYRO_CONFIG & 0xE7;
  //adding new range to the register
  val |= (new_range << 3);

  // Apply the range
  SENSOR_SELECT();
  success = sensor_common_write_reg(GYRO_CONFIG, &val, 1);
  SENSOR_DESELECT();

  if(success == false) {
    PRINTF("ERROR: Failed to write to GYRO_CONFIG register\r\n");
    return false;
  }

  //updating local copy of config and register
  mpu_config.gyro_range = new_range;
  mpu_reg.REG_GYRO_CONFIG = val;
  PRINTF("gyro_set_range end\r\n");
  return success;
}

/**
 *  @brief      Set compass sampling rate.
 *  The compass on the auxiliary I2C bus is read by the MPU hardware at a
 *  maximum of 100Hz. The actual rate can be set to a fraction of the gyro
 *  sampling rate.
 *
 *  \n WARNING: The new rate may be different than what was requested. Call
 *  mpu_get_compass_sample_rate to check the actual setting.
 *  @param[in]  rate    Desired compass sampling rate (Hz).
 *  @return     true if successful.
 */
static bool set_compass_sample_rate(unsigned short rate) {
  bool success;
  unsigned short new_rate = rate;
  PRINTF("set_compass_sample_rate begin\r\n");

  if (rate == 0) {
    PRINTF("ERROR: Compass rate can't be = 0\r\n");
    return false;
  } else if (rate > mpu_config.sample_rate) {
    PRINTF("INFO: Compass rate > gypo/accel sample rate\r\n");
    new_rate = mpu_config.sample_rate;
  } else if (rate > MAX_COMPASS_SAMPLE_RATE) {
    PRINTF("ERROR: Compass rate > MAX_COMPASS_SAMPLE_RATE\r\n");
    return false;
  }
  SENSOR_SELECT();
  //TODO Not to sure what slv4 ctrl is connected to and is doing..
  val = mpu_config.sample_rate  / new_rate - 1;
  success = sensor_common_write_reg(I2C_SLV4_CTRL, &val, 1);
  SENSOR_DESELECT();
  if(success == false) {
    PRINTF("ERROR: Failed while writing I2C_SLV4_CTRL register\r\n");
    return false;
  }

  mpu_config.compass_sample_rate = mpu_config.sample_rate / (val + 1);
  PRINTF("set_compass_sample_rate end\r\n");
  return true;
}

/**
 *  @brief      Set device to bypass mode.
 *  Used to communicate with Magnetometer directly over the I2C bus)
 *  @param[in]  bypass_on   1 to enable bypass mode.
 *  @return     true if successful.
 */
bool set_bypass(uint8_t bypass_on) {
  bool success;

  PRINTF("set_bypass begin\r\n");

  if (mpu_config.bypass_mode == bypass_on) {
    PRINTF("Info: bypass mode already on. set_bypass end\r\n");
    return true;
  }
  SENSOR_SELECT();
  if (bypass_on) {
    // disabling the I2C master
    success = sensor_common_read_reg(USER_CTRL, &val, 1);
    if(success == false) {
      SENSOR_DESELECT();
      PRINTF("ERROR: Failed while reading USER_CTRL register\r\n");
      return false;
    }
    val &= ~BIT_AUX_IF_EN;
    success = sensor_common_write_reg(USER_CTRL, &val, 1);
    if(success == false) {
      SENSOR_DESELECT();
      PRINTF("ERROR: Failed while writing USER_CTRL register\r\n");
      return false;
    }
    delay_ms(3);

    //enabling bypass mode
    val = BIT_BYPASS_EN;
    success = sensor_common_write_reg(INT_PIN_CFG, &val, 1);
    if(success  == false) {
      SENSOR_DESELECT();
      PRINTF("ERROR: Failed while writing INT_PIN_CFG register\r\n");
      return false;
    }
  } else {
    /* Enable I2C master mode if compass is being used. */
    success = sensor_common_read_reg(USER_CTRL, &val, 1);
    if(success  == false) {
      SENSOR_DESELECT();
      PRINTF("ERROR: Failed while reading USER_CTRL register\r\n");
      return false;
    }

    if (mpu_config.sensors & EN_XYZ_COMPASS) {
      val |= BIT_AUX_IF_EN;
    }

    val &= ~BIT_AUX_IF_EN;
    success = sensor_common_write_reg(USER_CTRL, &val, 1);
    if(success == false) {
      SENSOR_DESELECT();
      PRINTF("ERROR: Failed while writing USER_CTRL register\r\n");
      return false;
    }
    delay_ms(3);

    // disabling bypass mode
    val = 0;
    success = sensor_common_write_reg(INT_PIN_CFG, &val, 1);
    if(success == false) {
      SENSOR_DESELECT();
      PRINTF("ERROR: Failed while writing INT_PIN_CFG register\r\n");
      return false;
    }
  }
  SENSOR_DESELECT();
  mpu_config.bypass_mode = bypass_on;
  PRINTF("set_bypass end\r\n");
  return true;
}

/**
 *  @brief      Write to the DMP memory.
 *  This function prevents I2C writes past the bank boundaries. The DMP memory
 *  is only accessible when the chip is awake.
 *  @param[in]  mem_addr    Memory location (bank << 8 | start address)
 *  @param[in]  length      Number of bytes to write.
 *  @param[in]  data        Bytes to write to memory.
 *  @return     true if successful.
 */
static bool mpu_write_mem(unsigned short mem_addr, unsigned short length,
        uint8_t *data) {
  uint8_t tmp[2];
  bool success;
  //PRINTF("mpu_write_mem begin\r\n");

  if (!data) {
    PRINTF("ERROR: data is null\r\n");
    return false;
  }
  if (!mpu_config.sensors) {
    PRINTF("ERROR: trying to write to mpu but no sensors are activated\r\n");
    return false;
  }
  if (length > 0xFF) {
    PRINTF("ERROR: mpu_write_mem length too large. Len=%u\r\n", length);
    return false;
  }

  tmp[0] = (uint8_t)(mem_addr >> 8);
  tmp[1] = (uint8_t)(mem_addr & 0xFF);

  /* Check bank boundaries. */
  if (tmp[1] + length > MPU_MEM_BANK_SIZE) {
    PRINTF("ERROR: length exceed memory bank size\r\n");
    return false;
  }

  SENSOR_SELECT();
  //selecting the memory bank to read from MPU
  success = sensor_common_write_reg(BANK_SEL, tmp, 2);
  if (success == false) {
    SENSOR_DESELECT();
    PRINTF("ERROR: Failed to write to BANK_SEL register\r\n");
    return false;
  }

  //writing to the MPU memory bank
  success = sensor_common_write_reg(MEM_R_W, data, (uint8_t)length);
  SENSOR_DESELECT();
  if (success == false) {
    PRINTF("ERROR: Failed to write to MEM_R_W register\r\n");
    return false;
  }
  //PRINTF("mpu_write_mem end\r\n");
  return true;
}

/**
 *  @brief      Read from the DMP memory.
 *  This function prevents I2C reads past the bank boundaries. The DMP memory
 *  is only accessible when the chip is awake.
 *  @param[in]  mem_addr    Memory location (bank << 8 | start address)
 *  @param[in]  length      Number of bytes to read.
 *  @param[out] data        Bytes read from memory.
 *  @return     false if successful.
 */
static bool mpu_read_mem(unsigned short mem_addr, unsigned short length,
        uint8_t *data) {
  PRINTF("mpu_read_mem begin\r\n");
  PRINTF("INFO: writing to BANK_SEL data=%d\r\n", mem_addr);
  delay_ms(1);
  uint8_t tmp[2];
  bool success;

  if (!data) {
    PRINTF("ERROR: data is null\r\n");
    return false;
  }
  if (!mpu_config.sensors) {
    PRINTF("ERROR: trying to write to mpu but no sensors are activated\r\n");
    return false;
  }
  if (length > 0xFF) {
    PRINTF("ERROR: mpu_write_mem length too large. Len=%u\r\n", length);
    return false;
  }

  tmp[0] = (uint8_t)(mem_addr >> 8);
  tmp[1] = (uint8_t)(mem_addr & 0xFF);

  /* Check bank boundaries. */
  if (tmp[1] + length > MPU_MEM_BANK_SIZE) {
    PRINTF("ERROR: length exceed memory bank size\r\n");
    return false;
  }

  SENSOR_SELECT();
  //selecting the memory bank to read from MPU
  success = sensor_common_write_reg(BANK_SEL, tmp, 2);
  if (success == false) {
    SENSOR_DESELECT();
    PRINTF("ERROR: Failed to write to BANK_SEL register\r\n");
    return false;
  }
  PRINTF("INFO: reading from MEM_R_W len=%d\r\n", length);
  //reading from the MPU memory bank
  success = sensor_common_read_reg(MEM_R_W, data, (uint8_t)length);
  SENSOR_DESELECT();
  if (success == false) {
    PRINTF("ERROR: Failed to read from MEM_R_W register\r\n");
    return false;
  }

  PRINTF("mpu_read_mem end\r\n");
  return true;
}

/**
 *  @brief      Load and verify DMP image.
 *  @param[in]  length      Length of DMP image.
 *  @param[in]  firmware    DMP code.
 *  @param[in]  start_addr  Starting address of DMP code memory.
 *  @param[in]  sample_rate Fixed sampling rate used when DMP is enabled.
 *  @return     true if successful.
 */
static bool mpu_load_firmware(unsigned short length, const uint8_t *firmware,
    unsigned short start_addr, unsigned short sample_rate) {
  PRINTF("mpu_load_firmware begin\r\n");
  unsigned short i;
  unsigned short this_write;
  /* Must divide evenly into st.hw->bank_size to avoid bank crossings. */
#define LOAD_CHUNK  (16)
  uint8_t cur[LOAD_CHUNK], tmp[2];
  bool success;

  if (mpu_config.dmp_loaded) {
    /* DMP should only be loaded once. */
    PRINTF("ERROR: DMP firmware already loaded\r\n");
    return false;
  }

  if (!firmware) {
    PRINTF("ERROR: DMP firmware is NULL pointer\r\n");
    return false;
  }

  PRINTF("INFO: Starting to write firmware\r\n");
  for (i = 0; i < length; i += this_write) {

    this_write = min(LOAD_CHUNK,length - i);
    //PRINTF("INFO: Writing this_write = %d i=%d\r\n",this_write, i);
    if (mpu_write_mem(i, this_write, (uint8_t*)&firmware[i]) == false) {
      PRINTF("ERROR: Failed to write to MPU memory\r\n");
      return false;
    }
//    if (mpu_read_mem(i, this_write, cur) == false) {
//      PRINTF("ERROR: Failed to read from MPU memory\r\n");
//      return false;
//    }
//    if (memcmp(firmware+i, cur, this_write)) {
//      PRINTF("ERROR: Failed to copy memory\r\n");
//      return false;
//    }
  }
  PRINTF("INFO: Finished writing firmware\r\n");

  /* Set program start address. */
  tmp[0] = start_addr >> 8;
  tmp[1] = start_addr & 0xFF;
  SENSOR_SELECT();
  success = sensor_common_write_reg(PRGM_START_H, tmp, 2);
  SENSOR_DESELECT();
  if (success == false) {
    PRINTF("ERROR: Failed to write to PRGM_START_H register\r\n");
    return false;
  }

  mpu_config.dmp_loaded = 1;
  mpu_config.dmp_sample_rate = sample_rate;
  PRINTF("mpu_load_firmware end\r\n");
  return true;
}
//mpu_load_firmware(DMP_CODE_SIZE, dmp_memory, sStartAddress,DMP_SAMPLE_RATE);

/**
 *  @brief      Set sampling rate.
 *  Sampling rate must be between 4Hz and 1kHz.
 *  @param[in]  rate    Desired sampling rate (Hz).
 *  @return     true if successful.
 */
static bool set_sample_rate(unsigned short new_rate) {
  PRINTF("set_sample_rate begin\r\n");
  bool success;
  unsigned short rate = new_rate;

  if(!(mpu_config.sensors)) {
    PRINTF("ERROR: Trying to set sample rate but no sensors are on\r\n");
    return false;
  }

  if (mpu_config.dmp_on) {
    PRINTF("ERROR: Trying to set sample rate while dmp is on\r\n");
    return false;
  }
  // Gryo and acc limited to 1000Hz
  if (rate < 4) {
    rate = 4;
  } else if (rate > ACC_GYRO_INTERNAL_SMPLRT) {
    rate = ACC_GYRO_INTERNAL_SMPLRT;
  }
  // calculating the SMPLRT_DIV (sample rate divider).
  // Sample rate = Internal_Sample_Rate / (1 + SMPLRT_DIV)
  // The internal Sample rate = 1000 for ACC and gyro if LPF is on
  val = ACC_GYRO_INTERNAL_SMPLRT / rate - 1;

  // Apply the range
  SENSOR_SELECT();
  success = sensor_common_write_reg(SMPLRT_DIV, &val, 1);
  SENSOR_DESELECT();
  if (success == false) {
    PRINTF("ERROR: Failed to write to SMPLRT_DIV register\r\n");
    return false;
  }
  mpu_config.sample_rate = ACC_GYRO_INTERNAL_SMPLRT / (1 + val);

  success = set_compass_sample_rate(min(mpu_config.compass_sample_rate, MAX_COMPASS_SAMPLE_RATE));
  if (success == false) {
    PRINTF("ERROR: Failed to set_compass_sample_rate to %u\r\n",
        mpu_config.compass_sample_rate);
    return false;
  }
  /* TODO Automatically set LPF to 1/2 sampling rate. */
  //set_gyro_lpf(st.chip_cfg.sample_rate >> 1);
  PRINTF("set_sample_rate end\r\n");
  return success;
}

/**
 *  @brief      Read raw compass data.
 *  @param[out] data        Raw data in hardware units.
 *  @param[out] timestamp   Timestamp in milliseconds. Null if not needed.
 *  @return     0 if successful.
 */
static bool get_compass_reg(int16_t *data, unsigned long *timestamp) {
  static uint8_t tmp[9];
  bool success;

  if (!(mpu_config.sensors & EN_XYZ_COMPASS)) {
    PRINTF("ERROR: Trying to read from compass but not enabled\r\n");
    return false;
  }
  SENSOR_SELECT();
  success = sensor_common_read_reg(RAW_COMPASS, tmp, 8);
  SENSOR_DESELECT();
  if(success  == false) {
    PRINTF("ERROR: Failed while reading RAW_COMPASS register\r\n");
    return false;
  }

  data[0] = (tmp[2] << 8) | tmp[1];
  data[1] = (tmp[4] << 8) | tmp[3];
  data[2] = (tmp[6] << 8) | tmp[5];

  data[0] = ((long)data[0] * mpu_config.mag_sens_adj[0]) >> 8;
  data[1] = ((long)data[1] * mpu_config.mag_sens_adj[1]) >> 8;
  data[2] = ((long)data[2] * mpu_config.mag_sens_adj[2]) >> 8;

  //TODO enable timestamp
//  if (timestamp)
//      get_ms(timestamp);
  return true;
}

/**
 *  @brief      Setup the compass.
 *  @param[in]
 *  @return     true if successful.
 */
static bool setup_compass(void) {
  uint8_t data[4];
  bool success;
  PRINTF("setup_compass begin\r\n");
  if (set_bypass(1) == false) {
    PRINTF("ERROR: Failed while setting bypass to 0 \r\n");
  }

  MAG_SENSOR_SELECT();
  // confirming the AKM sensor is selected
  success = sensor_common_read_reg(AKM_REG_WHOAMI, &val, 1);
  if(success == false) {
    MAG_SENSOR_DESELECT();
    PRINTF("ERROR: Failed while reading AKM_REG_WHOAMI register\r\n");
    return false;
  }

  if (val != AKM_WHOAMI) {
    MAG_SENSOR_DESELECT();
    PRINTF("ERROR: AKM WHOAMI does not match! Expected =%02X got=%02X\r\n",
        (uint8_t)AKM_WHOAMI, val);
    return false;
  }

  // Reading FUSE ROM calibration data
  val = AKM_POWER_DOWN;
  success = sensor_common_write_reg(AKM_REG_CNTL, &val, 1);
  if(success == false) {
    MAG_SENSOR_DESELECT();
    PRINTF("ERROR: Failed while writing AKM_REG_CNTL register\r\n");
    return false;
  }
  delay_ms(1);

  val = AKM_FUSE_ROM_ACCESS;
  success = sensor_common_write_reg(AKM_REG_CNTL, &val, 1);
  if(success == false) {
    MAG_SENSOR_DESELECT();
    PRINTF("ERROR: Failed while writing AKM_REG_CNTL register\r\n");
    return false;
  }
  delay_ms(1);

  /* Get sensitivity adjustment data from fuse ROM. */
  success = sensor_common_read_reg(AKM_REG_ASAX, data, 3);
  if(success == false) {
    MAG_SENSOR_DESELECT();
    PRINTF("ERROR: Failed while reading AKM_REG_ASAX register\r\n");
    return false;
  }

  mpu_config.mag_sens_adj[0] = (long)data[0] + 128;
  mpu_config.mag_sens_adj[1] = (long)data[1] + 128;
  mpu_config.mag_sens_adj[2] = (long)data[2] + 128;

  // Power down again
  val = AKM_POWER_DOWN;
  success = sensor_common_write_reg(AKM_REG_CNTL, &val, 1);
  if(success == false) {
    MAG_SENSOR_DESELECT();
    PRINTF("ERROR: Failed while writing AKM_REG_CNTL register\r\n");
    return false;
  }
  delay_ms(1);
  MAG_SENSOR_DESELECT();

  if(set_bypass(0) == false) {
    PRINTF("ERROR: Failed while setting bypass to 0 \r\n");
    return false;
  }


  /* Set up master mode, master clock, and ES bit. */
  SENSOR_SELECT();
  val = 0x40;
  success = sensor_common_write_reg(I2C_MST_CTRL, &val, 1);
  if(success == false) {
    SENSOR_DESELECT();
    PRINTF("ERROR: Failed while writing I2C_MST_CTRL register\r\n");
    return false;
  }

  /* Slave 0 reads from AKM data registers. */
  val = BIT_I2C_READ | SENSOR_MAG_I2_ADDRESS;
  success = sensor_common_write_reg(I2C_SLV0_ADDR, &val, 1);
  if(success == false) {
    SENSOR_DESELECT();
    PRINTF("ERROR: Failed while writing I2C_SLV0_ADDR register\r\n");
    return false;
  }

  /* Compass reads start at this register. */
  val = AKM_REG_ST1;
  success = sensor_common_write_reg(I2C_SLV0_REG, &val, 1);
  if(success == false) {
    SENSOR_DESELECT();
    PRINTF("ERROR: Failed while writing I2C_SLV0_REG register\r\n");
    return false;
  }

  /* Enable slave 0, 8-byte reads. */
  val = BIT_SLAVE_EN | 8;
  success = sensor_common_write_reg(I2C_SLV0_CTRL, &val, 1);
  if(success == false) {
    SENSOR_DESELECT();
    PRINTF("ERROR: Failed while writing I2C_SLV0_CTRL register\r\n");
    return false;
  }

  /* Slave 1 changes AKM measurement mode. */
  val = SENSOR_MAG_I2_ADDRESS;
  success = sensor_common_write_reg(I2C_SLV1_ADDR, &val, 1);
  if(success == false) {
    SENSOR_DESELECT();
    PRINTF("ERROR: Failed while writing I2C_SLV1_ADDR register\r\n");
    return false;
  }

  /* AKM measurement mode register. */
  val = AKM_REG_CNTL;
  success = sensor_common_write_reg(I2C_SLV1_REG, &val, 1);
  if(success == false) {
    SENSOR_DESELECT();
    PRINTF("ERROR: Failed while writing I2C_SLV1_REG register\r\n");
    return false;
  }

  /* Enable slave 1, 1-byte writes. */
  val = BIT_SLAVE_EN | 1;
  success = sensor_common_write_reg(I2C_SLV1_CTRL, &val, 1);
  if(success == false) {
    SENSOR_DESELECT();
    PRINTF("ERROR: Failed while writing I2C_SLV1_CTRL register\r\n");
    return false;
  }

  /* Set slave 1 data. */
  val = AKM_SINGLE_MEASUREMENT;
  success = sensor_common_write_reg(I2C_SLV1_DO, &val, 1);
  if(success == false) {
    SENSOR_DESELECT();
    PRINTF("ERROR: Failed while writing I2C_SLV1_DO register\r\n");
    return false;
  }

  /* Trigger slave 0 and slave 1 actions at each sample. */
  val = 0x03;
  success = sensor_common_write_reg(I2C_MST_DELAY_CTRL, &val, 1);
  if(success == false) {
    SENSOR_DESELECT();
    PRINTF("ERROR: Failed while writing I2C_MST_DELAY_CTRL register\r\n");
    return false;
  }

  SENSOR_DESELECT();
  PRINTF("setup_compass end\r\n");
  return true;
}

/**
 *  @brief  Reset FIFO read/write pointers.
 *  @return true if successful.
 */
static bool mpu_reset_fifo(void) {
  bool success;
  PRINTF("mpu_reset_fifo begin\r\n");

  if (!(mpu_config.sensors)) {
    PRINTF("ERROR: reseting fifo but no sensors are enabled\r\n");
    return false;
  }

  SENSOR_SELECT();
  val = 0;
  // disabling interrupts
  success = sensor_common_write_reg(INT_ENABLE, &val, 1);
  if(success == false) {
    SENSOR_DESELECT();
    PRINTF("ERROR: Failed to disable interrupts.");
    PRINTF(" Failed while writing INT_ENABLE register\r\n");
    return false;
  }

  // disabling FIFO writes from sensors
  success = sensor_common_write_reg(FIFO_EN, &val, 1);
  if(success == false) {
    SENSOR_DESELECT();
    PRINTF("ERROR: Failed to disable FIFO writes from sensors.");
    PRINTF(" Failed while writing FIFO_EN register\r\n");
    return false;
  }

  // disabling FIFO writes from serial interface
  success = sensor_common_write_reg(USER_CTRL, &val, 1);
  if(success == false) {
    SENSOR_DESELECT();
    PRINTF("ERROR: Failed to disable FIFO writes from serial interface.");
    PRINTF(" Failed while writing USER_CTRL register\r\n");
    return false;
  }

  if (mpu_config.dmp_on) {
    // reseting the FIFO
    val = BIT_FIFO_RST | BIT_DMP_RST;
    success = sensor_common_write_reg(USER_CTRL, &val, 1);
    if(success == false) {
      SENSOR_DESELECT();
      PRINTF("ERROR: Failed to reset the FIFO");
      PRINTF("ERROR: Failed while writing USER_CTRL register\r\n");
      return false;
    }
    delay_ms(50);

    // enabling FIFO operation mode
    val = BIT_DMP_EN | BIT_FIFO_EN;
    success = sensor_common_write_reg(USER_CTRL, &val, 1);
    if (mpu_config.sensors & EN_XYZ_COMPASS) {
      val |= BIT_AUX_IF_EN;
    }

    success = sensor_common_write_reg(USER_CTRL, &val, 1);
    if(success == false) {
      SENSOR_DESELECT();
      PRINTF("ERROR: Failed to enable FIFO operation mode.");
      PRINTF("Failed while writing USER_CTRL register\r\n");
      return false;
    }

    // enabling interrupts
    if (mpu_config.int_enable) {
      val = BIT_DMP_INT_EN;
    } else{
      val = 0;
    }
    success = sensor_common_write_reg(INT_ENABLE, &val, 1);
    if(success == false) {
      SENSOR_DESELECT();
      PRINTF("ERROR: Failed to turn on interrupts.");
      PRINTF("Failed while writing INT_ENABLE register\r\n");
      return false;
    }

    val = 0;
    success = sensor_common_write_reg(FIFO_EN, &val, 1);
    if(success == false) {
      SENSOR_DESELECT();
      PRINTF("ERROR: Failed to disable sensor writes to FIFO(DMP MODE).");
      PRINTF("Failed while writing FIFO_EN register\r\n");
      return false;
    }
  } else {
    // reseting the FIFO
    val = BIT_FIFO_RST;
    success = sensor_common_write_reg(USER_CTRL, &val, 1);
    if(success == false) {
      SENSOR_DESELECT();
      PRINTF("ERROR: Failed to reset the FIFO");
      PRINTF("ERROR: Failed while writing USER_CTRL register\r\n");
      return false;
    }

    // enabling FIFO operation mode
    if (mpu_config.bypass_mode || !(mpu_config.sensors & EN_XYZ_COMPASS)) {
      val = BIT_FIFO_EN;
    } else {
      val = BIT_FIFO_EN | BIT_AUX_IF_EN;
    }
    success = sensor_common_write_reg(USER_CTRL, &val, 1);
    if(success == false) {
      SENSOR_DESELECT();
      PRINTF("ERROR: Failed to enable FIFO operation mode.");
      PRINTF("Failed while writing USER_CTRL register\r\n");
      return false;
    }
    delay_ms(50);

    //turning on interrupts again
    if (mpu_config.int_enable) {
      val = BIT_DATA_RDY_EN;
    } else {
      val = 0;
    }
    success = sensor_common_write_reg(INT_ENABLE, &val, 1);
    if(success == false) {
      SENSOR_DESELECT();
      PRINTF("ERROR: Failed to turn on interrupts.");
      PRINTF("Failed while writing INT_ENABLE register\r\n");
      return false;
    }

    // enabling sensors to write to FIFO
    val = mpu_reg.REG_FIFO_EN;
    success = sensor_common_write_reg(FIFO_EN, &val, 1);
    if(success == false) {
      SENSOR_DESELECT();
      PRINTF("ERROR: Failed to enable sensor writes to FIFO.");
      PRINTF("Failed while writing FIFO_EN register\r\n");
      return false;
    }
  }
  SENSOR_DESELECT();
  PRINTF("mpu_reset_fifo end\r\n");
  return true;
}

/**
 *  @brief      Enable/disable data ready interrupt.
 *  If the DMP is on, the DMP interrupt is enabled. Otherwise, the data ready
 *  interrupt is used.
 *  @param[in]  enable      1 to enable interrupt.
 *  @return      true if successful.
 */
static bool set_int_enable(uint8_t enable){
  bool success;
  PRINTF("set_int_enable begin\r\n");

  if (mpu_config.dmp_on) {
    if (enable) {
      val = BIT_DMP_INT_EN;
    } else {
      val = 0x00;
    }
    SENSOR_SELECT();
    success = sensor_common_write_reg(INT_ENABLE, &val, 1);
    SENSOR_DESELECT();
    if (success == false) {
      PRINTF("ERROR: Failed to write val=%02X to INT_ENABLE register\r\n", val);
      return false;
    }
    mpu_config.int_enable = 1;
  } else {
    if (!mpu_config.sensors) {
      PRINTF("ERROR: Setting interrupt but there are no sensors activated\r\n");
      return false;
    }
    if (enable && mpu_config.int_enable) {
      return true;
    }
    if (enable) {
      val = BIT_DATA_RDY_EN;
    } else {
      val = 0x00;
    }
    SENSOR_SELECT();
    success = sensor_common_write_reg(INT_ENABLE, &val, 1);
    SENSOR_DESELECT();
    if (success == false) {
      PRINTF("ERROR: Failed to write val=%02X to INT_ENABLE register\r\n", val);
      return false;
    }
    mpu_config.int_enable = 1;
  }
  PRINTF("set_int_enable end\r\n");
  return true;
}

/**
 *  @brief      Select which sensors are pushed to FIFO.
 *  @e sensors can contain a combination of the following flags:
 *  \n FIFO_EN_GYRO
 *  \n FIFO_EN_ACCEL
 *  @param[in]  sensors Mask of sensors to push to FIFO.
 *  @return     true if successful.
 */
static bool configure_fifo(uint8_t sensors) {
  PRINTF("configure_fifo begin\r\n");
  bool success;
  uint8_t prev;

  if (mpu_config.dmp_on) {
   return true;
  } else {
    if (!(mpu_config.sensors)) {
      PRINTF("ERROR: Trying to configure the fifo but no sensors are activated\r\n");
      return false;
    }

    prev = mpu_reg.REG_FIFO_EN;
    // trying to enable FIFO for gyro
    if (sensors & FIFO_EN_GYRO) {
      if (!(mpu_config.sensors & EN_XYZ_GYRO)) {
        PRINTF("ERROR: Gyro sensor not activated but trying to use gyro\r\n");
        return false;
      }
    }

    // trying to enable FIFO for gyro
    if (sensors & FIFO_EN_ACCEL) {
      if (!(mpu_config.sensors & EN_XYZ_ACCEL)) {
        PRINTF("ERROR: Accel sensor not activated but trying to use accel\r\n");
        return false;
      }
    }

    mpu_reg.REG_FIFO_EN = sensors;

    if (sensors || mpu_config.lp_accel_mode) {
      success = set_int_enable(1);
      if (success == false) {
        PRINTF("ERROR: Failed to enable interrupts\r\n");
        return false;
      }
    } else {
      success = set_int_enable(0);
      if (success == false) {
        PRINTF("ERROR: Failed to disable interrupts\r\n");
        return false;
      }
    }

    if (sensors) {
      success = mpu_reset_fifo();
      if (success == false) {
        PRINTF("ERROR: Failed to reset the FIFO\r\n");
        mpu_reg.REG_FIFO_EN = prev;
        return false;
      }
    }
  }
  PRINTF("configure_fifo end\r\n");
  return true;
}

/**
 *  @brief      Turn specific sensors on/off.
 *  @e sensors can contain a combination of the following flags:
 *  \n EN_XYZ_GYRO
 *  \n EN_XYZ_ACCEL
 *  \n EN_XYZ_COMPASS
 *  @param[in]  sensors    Mask of sensors to wake.
 *  @return     0 if successful.
 */
int mpu_set_sensors(uint8_t sensors) {
  uint8_t user_ctrl;
  bool success;

  val = EN_ALL_AXES;

  if (!(sensors & EN_XYZ_GYRO)) {
    //disabling gyro sensor
    PRINTF("INFO: disabling GYRO\r\n");
    val |= DISABLE_GYRO_AXES;
  }
  if (!(sensors & EN_XYZ_ACCEL)) {
    val |= DISABLE_ACC_AXES;
    PRINTF("INFO: disabling ACCEL\r\n");
  }
  SENSOR_SELECT();
  success = sensor_common_write_reg(PWR_MGMT_2, &val, 1);

  if (success == false) {
    SENSOR_DESELECT();
    mpu_reg.REG_PWR_MGMT2 = 0x00;
    mpu_config.sensors = 0x00;
    PRINTF("ERROR: Failed to write to PWR_MGMT_2 register\r\n");
    return false;
  }

  mpu_config.sensors = sensors;
  mpu_reg.REG_PWR_MGMT2 = val;
// TODO need to handle LP accel mode
//  if (sensors && (sensors != INV_XYZ_ACCEL))
//      /* Latched interrupts only used in LP accel mode. */
//      mpu_set_int_latched(0);
  success = sensor_common_read_reg(USER_CTRL, &val, 1);
  if(success  == false) {
    SENSOR_DESELECT();
    PRINTF("ERROR: Failed while reading USER_CTRL register\r\n");
    return false;
  }
  user_ctrl = val;

  /* Handle AKM power management. */
  if (sensors & EN_XYZ_COMPASS) {
    val = AKM_SINGLE_MEASUREMENT;
    user_ctrl |= BIT_AUX_IF_EN;
  } else {
    val = AKM_POWER_DOWN;
    user_ctrl &= ~BIT_AUX_IF_EN;
  }

  success = sensor_common_write_reg(I2C_SLV1_DO, &val, 1);
  if(success == false) {
    SENSOR_DESELECT();
    PRINTF("ERROR: Failed while writing I2C_SLV1_DO register\r\n");
    return false;
  }

  if (mpu_config.dmp_on) {
    user_ctrl |= BIT_DMP_EN;
  } else {
    user_ctrl &= ~BIT_DMP_EN;
  }

  // Enable/disable I2C master mode. DMP need master mode
  val = user_ctrl;
  success = sensor_common_write_reg(USER_CTRL, &val, 1);
  if(success == false) {
    SENSOR_DESELECT();
    PRINTF("ERROR: Failed while writing USER_CTRL register\r\n");
    return false;
  }

  SENSOR_DESELECT();
  delay_ms(50);
  return true;
}

/**
 *  @brief       Generate 6-axis quaternions from the DMP.
 *  In this driver, the 3-axis and 6-axis DMP quaternion features are mutually
 *  exclusive.
 *  @param[in]   enable  1 to enable 6-axis quaternion.
 *  @return      true if successful.
 */
static bool dmp_enable_6x_lp_quat(uint8_t enable) {
  PRINTF("dmp_enable_6x_lp_quat begin\r\n");

  uint8_t regs[4];
  bool success;
  if (enable) {
    regs[0] = DINA20;
    regs[1] = DINA28;
    regs[2] = DINA30;
    regs[3] = DINA38;
  } else {
    memset(regs, 0xA3, 4);
  }

  SENSOR_SELECT();
  success = mpu_write_mem(CFG_8, 4, regs);
  SENSOR_DESELECT();
  if (success == false) {
    PRINTF("ERROR: Failed while writing to CFG_8 register\r\n");
    return false;
  }

  PRINTF("dmp_enable_6x_lp_quat end\r\n");
  return mpu_reset_fifo();
}
/**
 *  @brief      Calibrate the gyro data in the DMP.
 *  After eight seconds of no motion, the DMP will compute gyro biases and
 *  subtract them from the quaternion output. If @e dmp_enable_feature is
 *  called with @e DMP_FEATURE_SEND_CAL_GYRO, the biases will also be
 *  subtracted from the gyro output.
 *  @param[in]  enable  1 to enable gyro calibration.
 *  @return     true if successful.
 */
static bool dmp_enable_gyro_cal(uint8_t enable) {
  if (enable) {
    uint8_t regs[9] = {0xb8, 0xaa, 0xb3, 0x8d, 0xb4, 0x98, 0x0d, 0x35, 0x5d};
    return mpu_write_mem(CFG_MOTION_BIAS, 9, regs);
  } else {
    uint8_t regs[9] = {0xb8, 0xaa, 0xaa, 0xaa, 0xb0, 0x88, 0xc3, 0xc5, 0xc7};
    return mpu_write_mem(CFG_MOTION_BIAS, 9, regs);
  }
}

/**
 *  @brief      Generate 3-axis quaternions from the DMP.
 *  In this driver, the 3-axis and 6-axis DMP quaternion features are mutually
 *  exclusive.
 *  @param[in]  enable  1 to enable 3-axis quaternion.
 *  @return     true if successful.
 */
static bool dmp_enable_lp_quat(uint8_t enable) {
  PRINTF("dmp_enable_lp_quat begin\r\n");
  uint8_t regs[4];
  bool success;
  if (enable) {
    regs[0] = DINBC0;
    regs[1] = DINBC2;
    regs[2] = DINBC4;
    regs[3] = DINBC6;
  } else {
    memset(regs, 0x8B, 4);
  }

  success = mpu_write_mem(CFG_LP_QUAT, 4, regs);
  if (success == false) {
    PRINTF("ERROR: Failed to write to MPU mem CFG_LP_QUAT\r\n");
    return false;
  }

  PRINTF("dmp_enable_lp_quat end\r\n");
  return mpu_reset_fifo();
}

/**
 *  @brief      Enable DMP features.
 *  The following \#define's are used in the input mask:
 *  \n DMP_FEATURE_TAP
 *  \n DMP_FEATURE_ANDROID_ORIENT
 *  \n DMP_FEATURE_LP_QUAT
 *  \n DMP_FEATURE_6X_LP_QUAT
 *  \n DMP_FEATURE_GYRO_CAL
 *  \n DMP_FEATURE_SEND_RAW_ACCEL
 *  \n DMP_FEATURE_SEND_RAW_GYRO
 *  \n NOTE: DMP_FEATURE_LP_QUAT and DMP_FEATURE_6X_LP_QUAT are mutually
 *  exclusive.
 *  \n NOTE: DMP_FEATURE_SEND_RAW_GYRO and DMP_FEATURE_SEND_CAL_GYRO are also
 *  mutually exclusive.
 *  @param[in]  mask    Mask of features to enable.
 *  @return     true if successful.
 */
static bool dmp_enable_feature(unsigned short mask) {
  PRINTF("dmp_enable_feature begin\r\n");
  uint8_t tmp[10];
  bool success;

  /* TODO: All of these settings can probably be integrated into the default
   * DMP image.
   */
  /* Set integration scale factor. */
  tmp[0] = (uint8_t)((GYRO_SF >> 24) & 0xFF);
  tmp[1] = (uint8_t)((GYRO_SF >> 16) & 0xFF);
  tmp[2] = (uint8_t)((GYRO_SF >> 8) & 0xFF);
  tmp[3] = (uint8_t)(GYRO_SF & 0xFF);
  success = mpu_write_mem(D_0_104, 4, tmp);
  if (success == false) {
    PRINTF("ERROR: Failed to write to MPU mem D_0_104\r\n");
    return false;
  }

//  /* Send sensor data to the FIFO. */
  tmp[0] = 0xA3;
  if (mask & DMP_FEATURE_SEND_RAW_ACCEL) {
    tmp[1] = 0xC0;
    tmp[2] = 0xC8;
    tmp[3] = 0xC2;
  } else {
    tmp[1] = 0xA3;
    tmp[2] = 0xA3;
    tmp[3] = 0xA3;
  }

  if (mask & DMP_FEATURE_SEND_ANY_GYRO) {
    tmp[4] = 0xC4;
    tmp[5] = 0xCC;
    tmp[6] = 0xC6;
  } else {
    tmp[4] = 0xA3;
    tmp[5] = 0xA3;
    tmp[6] = 0xA3;
  }
  tmp[7] = 0xA3;
  tmp[8] = 0xA3;
  tmp[9] = 0xA3;
  success = mpu_write_mem(CFG_15,10,tmp);
  if (success == false) {
    PRINTF("ERROR: Failed to write to MPU mem CFG_15\r\n");
    return false;
  }

  /* Send gesture data to the FIFO. */
  if (mask & (DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT)) {
    tmp[0] = DINA20;
  } else {
    tmp[0] = 0xD8;
  }
  success = mpu_write_mem(CFG_27,1,tmp);
  if (success == false) {
    PRINTF("ERROR: Failed to write to MPU mem CFG_27\r\n");
    return false;
  }

  if (mask & DMP_FEATURE_GYRO_CAL) {
    success = dmp_enable_gyro_cal(1);
    if (success == false) {
      PRINTF("ERROR: Failed to enable DMP gyro cal\r\n");
      return false;
    }
  } else {
    success = dmp_enable_gyro_cal(0);
    if (success == false) {
      PRINTF("ERROR: Failed to disable DMP gyro cal\r\n");
      return false;
    }
  }

  if (mask & DMP_FEATURE_SEND_ANY_GYRO) {
    if (mask & DMP_FEATURE_SEND_CAL_GYRO) {
      tmp[0] = 0xB2;
      tmp[1] = 0x8B;
      tmp[2] = 0xB6;
      tmp[3] = 0x9B;
    } else {
      tmp[0] = DINAC0;
      tmp[1] = DINA80;
      tmp[2] = DINAC2;
      tmp[3] = DINA90;
    }
    success = mpu_write_mem(CFG_GYRO_RAW_DATA, 4, tmp);
    if (success == false) {
      PRINTF("ERROR: Failed to write to MPU mem CFG_GYRO_RAW_DATA\r\n");
      return false;
    }
  }

  if (mask & DMP_FEATURE_TAP) {
    //TODO
    /* Enable tap. */
//    tmp[0] = 0xF8;
//    mpu_write_mem(CFG_20, 1, tmp);
//    dmp_set_tap_thresh(TAP_XYZ, 250);
//    dmp_set_tap_axes(TAP_XYZ);
//    dmp_set_tap_count(1);
//    dmp_set_tap_time(100);
//    dmp_set_tap_time_multi(500);
//
//    dmp_set_shake_reject_thresh(GYRO_SF, 200);
//    dmp_set_shake_reject_time(40);
//    dmp_set_shake_reject_timeout(10);
    PRINTF("ERROR: DMP_FEATURE_TAP not implemented yet ");
    return false;
  } else {
    tmp[0] = 0xD8;
    success = mpu_write_mem(CFG_20, 1, tmp);
    if (success == false) {
      PRINTF("ERROR: Failed to write to MPU mem CFG_20\r\n");
      return false;
    }
  }

  if (mask & DMP_FEATURE_ANDROID_ORIENT) {
    tmp[0] = 0xD9;
  } else {
    tmp[0] = 0xD8;
  }
  success = mpu_write_mem(CFG_ANDROID_ORIENT_INT, 1, tmp);
  if (success == false) {
    PRINTF("ERROR: Failed to write to MPU mem CFG_ANDROID_ORIENT_INT\r\n");
    return false;
  }

  if (mask & DMP_FEATURE_LP_QUAT) {
    success = dmp_enable_lp_quat(1);
    if (success == false) {
      PRINTF("ERROR: Failed to enable DMP lp quat\r\n");
      return false;
    }
  } else {
    success = dmp_enable_lp_quat(0);
    if (success == false) {
      PRINTF("ERROR: Failed to disable DMP lp quat\r\n");
      return false;
    }
  }

  if (mask & DMP_FEATURE_6X_LP_QUAT) {
    success = dmp_enable_6x_lp_quat(1);
    if (success == false) {
      PRINTF("ERROR: Failed to enable DMP 6x lp quat\r\n");
      return false;
    }
  } else {
    success = dmp_enable_6x_lp_quat(0);
    if (success == false) {
      PRINTF("ERROR: Failed to disable DMP 6x lp quat\r\n");
      return false;
    }
  }

  /* Pedometer is always enabled. */
  mpu_config.dmp_feature_mask = mask | DMP_FEATURE_PEDOMETER;
  success = mpu_reset_fifo();
  if (success == false) {
    PRINTF("ERROR: Failed to reset FIFO\r\n");
    return false;
  }

  mpu_config.dmp_packet_length = 0;
  if (mask & DMP_FEATURE_SEND_RAW_ACCEL) {
    mpu_config.dmp_packet_length += 6;
  }
  if (mask & DMP_FEATURE_SEND_ANY_GYRO) {
    mpu_config.dmp_packet_length += 6;
  }
  if (mask & (DMP_FEATURE_LP_QUAT | DMP_FEATURE_6X_LP_QUAT)) {
    mpu_config.dmp_packet_length += 16;
  }
  if (mask & (DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT)) {
    mpu_config.dmp_packet_length += 4;
  }
  PRINTF("dmp_enable_feature end\r\n");
  return true;
}

static bool dmpEnableFeatures(unsigned short mask) {
  // TODO This work around might be needed
  //unsigned short enMask = 0;
  //enMask |= mask;
  // Combat known issue where fifo sample rate is incorrect
  // unless tap is enabled in the DMP.
  //enMask |= DMP_FEATURE_TAP;
  //return dmp_enable_feature(enmask);
  return dmp_enable_feature(mask);
}

/**
 *  @brief      Set DMP output rate.
 *  Only used when DMP is on.
 *  @param[in]  rate    Desired fifo rate (Hz).
 *  @return     true if successful.
 */
static bool dmp_set_fifo_rate(unsigned short rate){
  PRINTF("dmp_set_fifo_rate begin\r\n");
  const uint8_t regs_end[12] = {DINAFE, DINAF2, DINAAB,
      0xc4, DINAAA, DINAF1, DINADF, DINADF, 0xBB, 0xAF, DINADF, DINADF};
  unsigned short div;
  uint8_t tmp[8];
  bool success;

  if (rate > DMP_SMPL_RATE) {
    return false;
  }
  div = DMP_SMPL_RATE / rate - 1;
  tmp[0] = (uint8_t)((div >> 8) & 0xFF);
  tmp[1] = (uint8_t)(div & 0xFF);
  success = mpu_write_mem(D_0_22, 2, tmp);
  if (success == false) {
    PRINTF("ERROR: Failed to write to MPU mem D_0_22\r\n");
    return false;
  }

  success = mpu_write_mem(CFG_6, 12, (uint8_t*)regs_end);
  if (success == false) {
    PRINTF("ERROR: Failed to write to MPU mem CFG_6\r\n");
    return false;
  }

  mpu_config.dmp_fifo_rate = rate;
  PRINTF("dmp_set_fifo_rate end\r\n");
  return true;
}

/**
 *  @brief      Enable/disable DMP support.
 *  @param[in]  enable  1 to turn on the DMP.
 *  @return     true if successful.
 */
static bool set_dmp_state(uint8_t enable) {
  PRINTF("set_dmp_state begin\r\n");
  bool success;

  if (mpu_config.dmp_on == enable) {
    return true;
  }

  if (enable) {
    if (!mpu_config.dmp_loaded) {
      PRINTF("ERROR: Trying to enable DMP but DMP is not loaded \r\n");
      return false;
    }

    /* Disable data ready interrupt. */
    success = set_int_enable(0);
    if (success == false) {
      PRINTF("ERROR: Failed to disable interrupts \r\n");
      return false;
    }

    /* Disable bypass mode. */
    success = set_bypass(0);
    if (success == false) {
      PRINTF("ERROR: Failed to disable bypass mode \r\n");
      return false;
    }

    /* Keep constant sample rate, FIFO rate controlled by DMP. */
    success = set_sample_rate(mpu_config.dmp_sample_rate);
    if (success == false) {
      PRINTF("ERROR: Failed to set sample rate \r\n");
      return false;
    }

    /* Remove FIFO elements. */
    val = 0;
    SENSOR_SELECT();
    success = sensor_common_write_reg(FIFO_EN, &val, 1);
    SENSOR_DESELECT();
    if(success == false) {
      PRINTF("ERROR: Failed to write to FIFO_EN register\r\n");
      return false;
    }

    mpu_config.dmp_on = 1;

    /* Enable DMP interrupt. */
    success = set_int_enable(1);
    if (success == false) {
      PRINTF("ERROR: Failed to enable DMP interrupts \r\n");
      return false;
    }

    success = mpu_reset_fifo();
    if (success == false) {
      PRINTF("ERROR: Failed to reset FIFO \r\n");
      return false;
    }
  } else {
    /* Disable DMP interrupt. */
    success = set_int_enable(0);
    if (success == false) {
      PRINTF("ERROR: Failed to disable DMP interrupts \r\n");
      return false;
    }

    /* Restore FIFO settings. */
    val = mpu_reg.REG_FIFO_EN;
    SENSOR_SELECT();
    success = sensor_common_write_reg(FIFO_EN, &val, 1);
    SENSOR_DESELECT();
    if(success == false) {
      PRINTF("ERROR: Failed to write to FIFO_EN register\r\n");
      return false;
    }

    mpu_config.dmp_on = 0;
    success = mpu_reset_fifo();
    if (success == false) {
      PRINTF("ERROR: Failed to reset FIFO \r\n");
      return false;
    }
  }
  PRINTF("set_dmp_state end\r\n");
  return true;
}

/**
 *  @brief      Get one unparsed packet from the FIFO.
 *  This function should be used if the packet is to be parsed elsewhere.
 *  @param[in]  length  Length of one FIFO packet.
 *  @param[in]  data    FIFO packet.
 *  @param[in]  more    Number of remaining packets.
 *  @return     true if successful.
 */
static bool read_fifo_stream(uint16_t length, uint8_t *data, uint8_t *more) {
  uint8_t tmp[2];
  uint16_t fifo_count;
  bool success;

  if (!mpu_config.dmp_on) {
    PRINTF("ERROR: Trying to read from FIFO but DMP is not on\r\n");
    return false;
  }
  if (!mpu_config.sensors) {
    PRINTF("ERROR: Trying to read from FIFO but no sensors are enabled\r\n");
    return false;
  }

  // reading FIFO size
  SENSOR_SELECT();
  success = sensor_common_read_reg(FIFO_COUNT_H, tmp, 2);
  if(success  == false) {
    SENSOR_DESELECT();
    PRINTF("ERROR: Failed while reading FIFO_COUNT_H register\r\n");
    return false;
  }
  fifo_count = (tmp[0] << 8) | tmp[1];

  if (fifo_count < length) {
      *more = 0;
      PRINTF("ERROR: fifo_count less than requested length\r\n");
      return false;
  }

  if (fifo_count > (MAX_FIFO >> 1)) {
    /* FIFO is 50% full, better check overflow bit. */
    success = sensor_common_read_reg(INT_STATUS, &val, 1);
    if(success  == false) {
      SENSOR_DESELECT();
      PRINTF("ERROR: Failed while reading FIFO_COUNT_H register\r\n");
      return false;
    }

    if (val & BIT_FIFO_OVERFLOW) {
      SENSOR_DESELECT();
      PRINTF("ERROR: FIFO overflowed. Reseting FIFO\r\n");
      mpu_reset_fifo();
      return false;
    }
  }

  success = sensor_common_read_reg(FIFO_R_W, data, length);
  if(success  == false) {
    SENSOR_DESELECT();
    PRINTF("ERROR: Failed while reading FIFO_R_W register\r\n");
    return false;
  }
  *more = fifo_count / length - 1;
  return true;
}

/**
 *  @brief      Get one packet from the FIFO.
 *  If @e sensors does not contain a particular sensor, disregard the data
 *  returned to that pointer.
 *  \n @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  \n INV_WXYZ_QUAT
 *  \n If the FIFO has no new data, @e sensors will be zero.
 *  \n If the FIFO is disabled, @e sensors will be zero and this function will
 *  return a non-zero error code.
 *  @param[out] gyro        Gyro data in hardware units.
 *  @param[out] accel       Accel data in hardware units.
 *  @param[out] quat        3-axis quaternion data in hardware units.
 *  @param[out] timestamp   Timestamp in milliseconds.
 *  @param[out] sensors     Mask of sensors read from FIFO.
 *  @param[out] more        Number of remaining packets.
 *  @return     true if successful.
 */
static bool dmp_read_fifo(dmp_sensor* mpu_data) {
  uint8_t fifo_data[MAX_FIFO_LENGTH];
  uint8_t i = 0;
  uint8_t more;
  bool success;

  /* Get a packet. */
  success = read_fifo_stream(mpu_config.dmp_packet_length, fifo_data, &more);
  if (success == false) {
    PRINTF("ERROR: Failed to read from FIFO stream\r\n");
    return false;
  }

  /* Parse DMP packet. */
  if (mpu_config.dmp_feature_mask & (DMP_FEATURE_LP_QUAT | DMP_FEATURE_6X_LP_QUAT)) {
#ifdef FIFO_CORRUPTION_CHECK
      long quat_q14[4], quat_mag_sq;
#endif
    mpu_data->quat[0] = ((long)fifo_data[0] << 24) | ((long)fifo_data[1] << 16) |
      ((long)fifo_data[2] << 8) | fifo_data[3];
    mpu_data->quat[1] = ((long)fifo_data[4] << 24) | ((long)fifo_data[5] << 16) |
      ((long)fifo_data[6] << 8) | fifo_data[7];
    mpu_data->quat[2] = ((long)fifo_data[8] << 24) | ((long)fifo_data[9] << 16) |
      ((long)fifo_data[10] << 8) | fifo_data[11];
    mpu_data-> quat[3] = ((long)fifo_data[12] << 24) | ((long)fifo_data[13] << 16) |
      ((long)fifo_data[14] << 8) | fifo_data[15];
    i += 16;
#ifdef FIFO_CORRUPTION_CHECK
    /* We can detect a corrupted FIFO by monitoring the quaternion data and
     * ensuring that the magnitude is always normalized to one. This
     * shouldn't happen in normal operation, but if an I2C error occurs,
     * the FIFO reads might become misaligned.
     *
     * Let's start by scaling down the quaternion data to avoid long long
     * math.
     */
    quat_q14[0] = mpu_data->quat[0] >> 16;
    quat_q14[1] = mpu_data->quat[1] >> 16;
    quat_q14[2] = mpu_data->quat[2] >> 16;
    quat_q14[3] = mpu_data->quat[3] >> 16;
    quat_mag_sq = quat_q14[0] * quat_q14[0] + quat_q14[1] * quat_q14[1] +
        quat_q14[2] * quat_q14[2] + quat_q14[3] * quat_q14[3];
    if ((quat_mag_sq < QUAT_MAG_SQ_MIN) ||
      (quat_mag_sq > QUAT_MAG_SQ_MAX)) {
      /* Quaternion is outside of the acceptable threshold. */
      PRINTF("ERROR: quat corrupt, resetting FIFO\r\n");
      mpu_reset_fifo();
      return false;
    }
#endif
  }

  if (mpu_config.dmp_feature_mask & DMP_FEATURE_SEND_RAW_ACCEL) {
    mpu_data->acc[0] = ((short)fifo_data[i+0] << 8) | fifo_data[i+1];
    mpu_data->acc[1] = ((short)fifo_data[i+2] << 8) | fifo_data[i+3];
    mpu_data->acc[2] = ((short)fifo_data[i+4] << 8) | fifo_data[i+5];
    i += 6;
  }

  if (mpu_config.dmp_feature_mask & DMP_FEATURE_SEND_ANY_GYRO) {
    mpu_data->gyr[0] = ((short)fifo_data[i+0] << 8) | fifo_data[i+1];
    mpu_data->gyr[1] = ((short)fifo_data[i+2] << 8) | fifo_data[i+3];
    mpu_data->gyr[2] = ((short)fifo_data[i+4] << 8) | fifo_data[i+5];
    i += 6;
  }

  /* Gesture data is at the end of the DMP packet. Parse it and call
   * the gesture callbacks (if registered).
   */
  if (mpu_config.dmp_feature_mask & (DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT)) {
    //TODO
    //decode_gesture(fifo_data + ii);
  }

  //get_ms(timestamp);
  return true;
}

/**
 *  @brief      Initialize hardware.
 *  Initial configuration:\n
 *  Gyro FSR: +/- 2000DPS\n
 *  Accel FSR +/- 2G\n
 *  DLPF: 42Hz\n
 *  FIFO rate: 50Hz\n
 *  Clock source: Gyro PLL\n
 *  FIFO: Disabled.\n
 *  Data ready interrupt: Disabled, active low, unlatched.
 *  @param[in]  int_param   Platform-specific parameters to interrupt API.
 *  @return     true if successful.
 */
static bool mpu_init() {
  bool success;
  SENSOR_SELECT();
  /* Reset device. */

  val = 0x00;
  success = sensor_common_write_reg(PWR_MGMT_1, &val, 1);

  if (success == false) {
    SENSOR_DESELECT();
    PRINTF("ERROR: Failed to write reset to PWR_MGMT_1 register\r\n");
    return false;
  }
  delay_ms(100);

  // Waking up chip
  val = 0x00;
  success = sensor_common_write_reg(PWR_MGMT_1, &val, 1);
  if (success == false) {
    SENSOR_DESELECT();
    PRINTF("ERROR: Failed to write to PWR_MGMT_1 register\r\n");
    return false;
  }

  /* Clear interrupts */
  success = sensor_common_read_reg(INT_STATUS, &val, 1);
  if (success == false) {
    SENSOR_DESELECT();
    PRINTF("ERROR: Failed to write to INT_STATUS register\r\n");
    return false;
  }

  SENSOR_DESELECT();

  PRINTF("Power up successful\r\n");

  /* Set to invalid values to ensure no I2C writes are skipped. */
  mpu_reg.REG_ACCEL_CONFIG = 0x00;
  mpu_reg.REG_GYRO_CONFIG = 0x00;
  mpu_reg.REG_CONFIG = 0x00;
  mpu_reg.REG_SMPLRT_DIV = 0x00;

  mpu_config.sensors = EN_ALL_SENSORS;
  mpu_config.gyro_range = 0xFF;
  mpu_config.acc_range = 0xFF;
  mpu_config.dmp_on = 0;
  mpu_config.sample_rate = 0;
  mpu_config.compass_sample_rate = MAX_COMPASS_SAMPLE_RATE;
  mpu_config.bypass_mode = 0xFF;
  mpu_config.lp_accel_mode = 0;
  mpu_config.int_enable = 0;
  mpu_config.dmp_loaded = 0;
  mpu_config.dmp_sample_rate = 0;
  mpu_config.dmp_feature_mask = 0;
  mpu_config.dmp_packet_length = 0;
  mpu_config.dmp_fifo_rate = 0;

  if (gyro_set_range(MPU9250_RANGE_2000DPS) == false) {
    PRINTF("ERROR: Failed to set gyro range\r\n");
    return false;
  } else {
    PRINTF("gyro set range success\r\n");
  }

  if (acc_set_range(MPU9250_ACCEL_RANGE_2G) == false) {
    PRINTF("ERROR: Failed to set accel range\r\n");
    return false;
  } else {
    PRINTF("accel set range success\r\n");
  }

  if (set_gyro_lpf(MPU9250_GYRO_DLPF_41HZ) == false) {
    PRINTF("ERROR: Failed to set gyro DLPF\r\n");
    return false;
  }

  if (set_sample_rate(50) == false) {
    PRINTF("ERROR: Failed to set sample rate\r\n");
    return false;
  }

  if (configure_fifo(0) == false) {
    PRINTF("ERROR: Failed to configure fifo\r\n");
    return false;
  }

  if(setup_compass() == false) {
    PRINTF("ERROR: Failed to setup compass\r\n");
    return false;
  }
  if (set_compass_sample_rate(10) == false) {
    PRINTF("ERROR: Failed to setup compass sample rate to 10\r\n");
    return false;
  }

  mpu_set_sensors(EN_ALL_SENSORS);
  delay_ms(10);
  return true;
}
/**
 *  @brief      Check to see if data is availabe in FIFO.
 *  @param[in]  int_param   Platform-specific parameters to interrupt API.
 *  @return     true if successful.
 */
static uint16_t fifo_available(void) {
  uint8_t tmp[2];
  bool success;

  SENSOR_SELECT();
  success = sensor_common_read_reg(FIFO_COUNT_H, tmp, 2);
  if(success  == false) {
    SENSOR_DESELECT();
    PRINTF("ERROR: Failed while reading FIFO_COUNT_H register\r\n");
    return 0;
  }

  return (tmp[0] << 8) | tmp[1];;
}

/*---------------------------------------------------------------------------*/
/**
 * \brief Configuration function for the MPU9250 sensor.
 *
 * \param type Activate, enable or disable the sensor. See below
 * \param enable
 *
 * When type == SENSORS_HW_INIT we turn on the hardware
 * When type == SENSORS_ACTIVE and enable==1 we enable the sensor
 * When type == SENSORS_ACTIVE and enable==0 we disable the sensor
 * When type == MPU_DMP_ACTIVATE and enable >0 we set the FIFO rate
 * When type == MPU_DMP_ACTIVATE and enable==0 we disable the DMP
 */
static int configure(int type, int enable) {
  bool success;
  unsigned short feat = DMP_FEATURE_6X_LP_QUAT;
  unsigned short rate = (unsigned short)enable;

  switch(type) {
  case SENSORS_HW_INIT:
    ti_lib_rom_ioc_pin_type_gpio_input(BOARD_IOID_MPU_INT);
    ti_lib_ioc_io_port_pull_set(BOARD_IOID_MPU_INT, IOC_IOPULL_DOWN);
    ti_lib_ioc_io_hyst_set(BOARD_IOID_MPU_INT, IOC_HYST_ENABLE);

    ti_lib_rom_ioc_pin_type_gpio_output(BOARD_IOID_MPU_POWER);
    ti_lib_ioc_io_drv_strength_set(BOARD_IOID_MPU_POWER, IOC_CURRENT_4MA,
                                   IOC_STRENGTH_MAX);
    ti_lib_gpio_clear_dio(BOARD_IOID_MPU_POWER);
    elements = MPU_9250_SENSOR_TYPE_NONE;
    break;
  case SENSORS_ACTIVE:
    if(((enable & MPU_9250_SENSOR_TYPE_ACC) != 0) ||
       ((enable & MPU_9250_SENSOR_TYPE_GYRO) != 0)) {
      PRINTF("MPU: Enabling\n");
      elements = enable & MPU_9250_SENSOR_TYPE_ALL;

      power_up();

      state = SENSOR_STATE_BOOTING;
    } else {
      PRINTF("MPU: Disabling\n");
      if(HWREG(GPIO_BASE + GPIO_O_DOUT31_0) & BOARD_MPU_POWER) {
        /* Then check our state */
        elements = MPU_9250_SENSOR_TYPE_NONE;
        ctimer_stop(&startup_timer);
        sensor_sleep();
        while(ti_lib_i2c_master_busy(I2C0_BASE));
        state = SENSOR_STATE_DISABLED;
        ti_lib_gpio_clear_dio(BOARD_IOID_MPU_POWER);
      }
    }
    break;
  case DMP_START:
    success = mpu_load_firmware(DMP_CODE_SIZE, dmp_memory, sStartAddress,DMP_SMPL_RATE);
    if (success == false) {
      PRINTF("ERROR: Failed to load the DMP firmware\n");
    }

    // 3-axis and 6-axis LP quat are mutually exclusive.
    // If both are selected, default to 3-axis
    if (feat & DMP_FEATURE_LP_QUAT) {
      feat &= ~(DMP_FEATURE_6X_LP_QUAT);
      //dmp_enable_lp_quat(1);
    } else if (feat & DMP_FEATURE_6X_LP_QUAT) {
      success = dmp_enable_6x_lp_quat(1);
      if (success == false) {
        PRINTF("ERROR: Failed to enable 6x lp quat\n");
      }
    }

    if (feat & DMP_FEATURE_GYRO_CAL) {
      success = dmp_enable_gyro_cal(1);
      if (success == false) {
        PRINTF("ERROR: Failed to enable DMP gyro cal");
      }
    }

    if (dmpEnableFeatures(feat) == false) {
      PRINTF("ERROR: Failed to enable DMP features");
    }

    if (rate < 1 && rate > 200) {
      PRINTF("ERROR: The DMP FIFO rate out of range(1-200)");
    }
    if (dmp_set_fifo_rate(rate) == false) {
      PRINTF("ERROR: Failed to set DMP fifo rate");
    }

    success = set_dmp_state(1);
    if (success == false) {
      PRINTF("ERROR: Failed to set DMP state to 1");
    }
    break;
  default:
    break;
  }
  return state;
}

static int mpu_value(mpu_sensor* mpu_data) {
  int ret = 0;
  int rv;
  static int16_t compass[3];

  if(state == SENSOR_STATE_DISABLED) {
    PRINTF("MPU: Sensor Disabled\n");
    return CC26XX_SENSOR_READING_ERROR;
  }

  memset(sensor_value, 0, sizeof(sensor_value));
  t0 = RTIMER_NOW();

  while(!int_status() &&
        (RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + READING_WAIT_TIMEOUT)));

  rv = mpu_read(sensor_value);

  if(rv == 0) {
    return CC26XX_SENSOR_READING_ERROR;
  }

  // Convert accel to mG
  mpu_data->acc[0] = 10 * acc_convert(sensor_value[0]);
  mpu_data->acc[1] = 10 * acc_convert(sensor_value[1]);
  mpu_data->acc[2] = 10 * acc_convert(sensor_value[2]);

  // Convert gyro to mDegrees/s
  mpu_data->gyr[0] = gyro_convert(sensor_value[4]);
  mpu_data->gyr[1] = gyro_convert(sensor_value[5]);
  mpu_data->gyr[2] = gyro_convert(sensor_value[6]);

  // getting compass values
  rv = get_compass_reg(compass,NULL);
  mpu_data->mag[0] = compass[0];
  mpu_data->mag[1] = compass[1];
  mpu_data->mag[2] = compass[2];

  return ret;
}

static int dmp_value(dmp_sensor* mpu_data) {
  int ret = 0;
  bool success;

  success = dmp_read_fifo(mpu_data);
  if (success == false) {
    PRINTF("ERROR: Failed to read fifo\n");
    return CC26XX_SENSOR_READING_ERROR;
  }

  return ret;
}
/*---------------------------------------------------------------------------*/
MPU_SENSOR(mpu_9250_sensor, "MPU9250", value, configure, status, mpu_value, \
    fifo_available, dmp_value);
/*---------------------------------------------------------------------------*/
/** @} */
