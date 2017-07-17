
#ifndef IMU_MAG_H
#define IMU_MAG_H


#include <iostream>
#include <string>
#include <sstream>
#include <myi2c.h>
#include <cmath>
#include <cstdlib>

#define PI 3.14159
#define DECLINATION 12

/// I2C slave address
#define I2C_SLAVE_ADDR_RTC      0x6f        /// TBD
#define I2C_SLAVE_ADDR_IMU      0x6b        /// Pololu IMU
#define I2C_SLAVE_ADDR_MAG      0x1d        /// Pololu mag LM303
#define I2C_SLAVE_ADDR_RHTEMP   0x5f        /// MEMS HTS221
#define I2C_SLAVE_ADDR_PRESSURE 0x5c        /// MEMS LP25H

#define READ_BUF_G         0x00
#define READ_BUF_XL        0x08


/// polo Magnetic Registers
#define CTRL_REG1         0x20
#define CTRL_REG2         0x21
#define CTRL_REG3         0x22
#define CTRL_REG4         0x23
#define CTRL_REG5         0x24
#define STATUS_REG_M        0x27
#define OUT_X_L_M           0x28
#define OUT_X_H_M           0x29
#define OUT_Y_L_M           0x2a
#define OUT_Y_H_M           0x2b
#define OUT_Z_L_M           0x2c
#define OUT_Z_H_M           0x2d
#define OUT_TEMP_M_L          0x2E
#define OUT_TEMP_M_H          0x2F

#define LSM303D_ID              0x49

///  LSM303D Registers
#define LSM303D_TEMP_OUT_L      0x05
#define LSM303D_TEMP_OUT_H      0x06
#define LSM303D_STATUS_M        0x07
#define LSM303D_OUT_X_L_M       0x08
#define LSM303D_OUT_X_H_M       0x09
#define LSM303D_OUT_Y_L_M       0x0a
#define LSM303D_OUT_Y_H_M       0x0b
#define LSM303D_OUT_Z_L_M       0x0c
#define LSM303D_OUT_Z_H_M       0x0d
#define LSM303D_WHO_AM_I        0x0f
#define LSM303D_INT_CTRL_M      0x12
#define LSM303D_INT_SRC_M       0x13
#define LSM303D_INT_THS_L_M     0x14
#define LSM303D_INT_THS_H_M     0x15
#define LSM303D_OFFSET_X_L_M    0x16
#define LSM303D_OFFSET_X_H_M    0x17
#define LSM303D_OFFSET_Y_L_M    0x18
#define LSM303D_OFFSET_Y_H_M    0x19
#define LSM303D_OFFSET_Z_L_M    0x1a
#define LSM303D_OFFSET_Z_H_M    0x1b
#define LSM303D_REFERENCE_X     0x1c
#define LSM303D_REFERENCE_Y     0x1d
#define LSM303D_REFERENCE_Z     0x1e
#define LSM303D_CTRL0           0x1f
#define LSM303D_CTRL1           0x20
#define LSM303D_CTRL2           0x21
#define LSM303D_CTRL3           0x22
#define LSM303D_CTRL4           0x23
#define LSM303D_CTRL5           0x24
#define LSM303D_CTRL6           0x25
#define LSM303D_CTRL7           0x26
#define LSM303D_STATUS_A        0x27
#define LSM303D_OUT_X_L_A       0x28
#define LSM303D_OUT_X_H_A       0x29
#define LSM303D_OUT_Y_L_A       0x2a
#define LSM303D_OUT_Y_H_A       0x2b
#define LSM303D_OUT_Z_L_A       0x2c
#define LSM303D_OUT_Z_H_A       0x2d
#define LSM303D_FIFO_CTRL       0x2e
#define LSM303D_FIFO_SRC        0x2f
#define LSM303D_IG_CFG1         0x30
#define LSM303D_IG_SRC1         0x31
#define LSM303D_IG_THS1         0x32
#define LSM303D_IG_DUR1         0x33
#define LSM303D_IG_CFG2         0x34
#define LSM303D_IG_SRC2         0x35
#define LSM303D_IG_THS2         0x36
#define LSM303D_IG_DUR2         0x37
#define LSM303D_CLICK_CFG       0x38
#define LSM303D_CLICK_SRC       0x39
#define LSM303D_CLICK_THS       0x3a
#define LSM303D_TIME_LIMIT      0x3b
#define LSM303D_TIME_LATENCY    0x3c
#define LSM303D_TIME_WINDOW     0x3d
#define LSM303D_ACT_THIS        0x3e
#define LSM303D_ACT_DUR         0x3f

///  Accel sample rate defines

#define LSM303D_ACCEL_SAMPLERATE_3_125 1
#define LSM303D_ACCEL_SAMPLERATE_6_25 2
#define LSM303D_ACCEL_SAMPLERATE_12_5 3
#define LSM303D_ACCEL_SAMPLERATE_25   4
#define LSM303D_ACCEL_SAMPLERATE_50   5
#define LSM303D_ACCEL_SAMPLERATE_100  6
#define LSM303D_ACCEL_SAMPLERATE_200  7
#define LSM303D_ACCEL_SAMPLERATE_400  8
#define LSM303D_ACCEL_SAMPLERATE_800  9
#define LSM303D_ACCEL_SAMPLERATE_1600 10


#define LSM303D_ACCEL_FSR_2     0
#define LSM303D_ACCEL_FSR_4     1
#define LSM303D_ACCEL_FSR_6     2
#define LSM303D_ACCEL_FSR_8     3
#define LSM303D_ACCEL_FSR_16    4


#define LSM303D_ACCEL_LPF_773   0
#define LSM303D_ACCEL_LPF_194   1
#define LSM303D_ACCEL_LPF_362   2
#define LSM303D_ACCEL_LPF_50    3


#define LSM303D_COMPASS_SAMPLERATE_3_125    0
#define LSM303D_COMPASS_SAMPLERATE_6_25     1
#define LSM303D_COMPASS_SAMPLERATE_12_5     2
#define LSM303D_COMPASS_SAMPLERATE_25       3
#define LSM303D_COMPASS_SAMPLERATE_50       4
#define LSM303D_COMPASS_SAMPLERATE_100      5


#define LSM303D_COMPASS_FSR_2   0
#define LSM303D_COMPASS_FSR_4   1
#define LSM303D_COMPASS_FSR_8   2
#define LSM303D_COMPASS_FSR_12  3

typedef struct tag_DOF_data
{
    float x;    /// x component
    float y;    /// y component
    float z;    /// z component
} DOF_data;


/// can i do this?
template <typename T> struct vector
    {
      T x, y, z;
    };

const float M_GN[] = {0.00014, 0.00029, 0.00043, 0.00058 };

extern int I2C_FP[5];
enum I2C_bus {IMU, MAG, TEMP, HUMID, RTC};  /// add others for future

template<typename T>
T vector_maginitude( const vector<T> my_vec)
{
    T magnitude;
    magnitude = pow((my_vec.x*my_vec.x + my_vec.y *my_vec.y + my_vec.z*my_vec.z), 0.5);
    return magnitude;
}

class IMU_303 : public I2CBus
{
public:
    IMU_303();
    ~IMU_303();
    void Init_303();
    void set_defaults();


    template <typename T> struct vector
    {
      T x, y, z;
    };
    template<typename T>
    T vector_maginitude( const vector<T> my_vec)
    {
        T magnitude;
        magnitude = pow((my_vec.x*my_vec.x + my_vec.y *my_vec.y + my_vec.z*my_vec.z), 0.5);
        return magnitude;
    }


private:
    vector<int16_t> acc_data; // accelerometer readings
    vector<int16_t> mag_data; // magnetometer readings
    vector<int16_t> mag_max; // maximum magnetometer values, used for calibration
    vector<int16_t> mag_min; // minimum magnetometer values, used for calibration

};







#endif // IMU_MAG_H


