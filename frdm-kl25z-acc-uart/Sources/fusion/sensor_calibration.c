#include <assert.h>

#include "compiler.h"
#include "fixmatrix.h"
#include "fixarray.h"

#define EXTERN_INLINE_SENSOR INLINE
#include "fusion/sensor_calibration.h"

#if !defined(FIXMATRIX_MAX_SIZE) || (FIXMATRIX_MAX_SIZE < 4)
#error FIXMATRIX_MAX_SIZE must be defined to value greater or equal 4.
#endif

/*!
* \brief Affine transformation matrix for HMC5883L sensor data calibration
*
* Data is retrieved via MATLAB script and only valid for a specific board configuration.
* Be sure to provide your own values here or YMMV.
*/
__attribute__((section("ROM")))
const static mf16 hmc5883l_calibration_data = {
    3, 4, 0,
    {
        { F16(0.98308),     F16(0.0025144),     F16(0.02777),       F16(0.0064502) },
        { F16(0.0025144),   F16(0.92661),       F16(-0.043022),     F16(0.10543) },
        { F16(0.02777),     F16(-0.043022),     F16(1.1128),        F16(-0.020258) }
    }};

/*!
* \brief Affine transformation matrix for MPU6050 accelerometer sensor data calibration
*
* Data is retrieved via MATLAB script and only valid for a specific board configuration.
* Be sure to provide your own values here or YMMV.
*/
__attribute__((section("ROM")))
const static mf16 mpu6050_accelerometer_calibration_data = {
    3, 4, 0,
    {
        { F16(1.0062),      F16(0.0034341),     F16(-0.0027532),    F16(-0.0164) },
        { F16(0.0034341),   F16(1.0008),        F16(-0.0056162),    F16(-0.016173) },
        { F16(-0.0027532),  F16(-0.0056162),    F16(0.9931),        F16(0.02059) }
    } };

/*!
* \brief Affine transformation matrix for MPU6050 gyroscope sensor data calibration
*
* Data is retrieved via MATLAB script and only valid for a specific board configuration.
* Be sure to provide your own values here or YMMV.
*/
__attribute__((section("ROM")))
const static mf16 mpu6050_gyroscope_calibration_data = {
    3, 4, 0,
    {
        { F16(1),           0,                  0,                  F16(-4.5446) },
        { 0,                F16(1),             0,                  F16(-0.048858) },
        { 0,                0,                  F16(1),             F16(-1.1197) }
    } };

/*!
* \brief Sensor variances for the MPU6050 accelerometer.
*
* Data is retrieved via MATLAB script and only valid for a specific board configuration.
* Be sure to provide your own values here or YMMV.
*/
const static fix16_t var_mpu6050_accelerometer[3]   = { F16(9.8036e-06),    F16(9.6462e-06),    F16(2.4831e-05) };

/*!
* \brief Sensor variances for the MPU6050 gyroscope.
*
* Data is retrieved via MATLAB script and only valid for a specific board configuration.
* Be sure to provide your own values here or YMMV.
*/
const static fix16_t var_mpu6050_gyroscope[3]       = { F16(0.016307),      F16(0.0084706),     F16(0.0129) };

/*!
* \brief Sensor variances for the HMC5883L magnetometer.
*
* Data is retrieved via MATLAB script and only valid for a specific board configuration.
* Be sure to provide your own values here or YMMV.
*/
const static fix16_t var_hmc5883l[3]                = { F16(2.0347e-06),    F16(1.9233e-06),    F16(2.3021e-06) };

/*!
* \brief Retrieves the variances of the MPU6050 accelerometer
* \param[out] x The x axis variances
* \param[out] y The y axis variances
* \param[out] z The z axis variances
*/
void mpu6050_var_accelerometer(register fix16_t *const RESTRICT x, register fix16_t *const RESTRICT y, register fix16_t *const RESTRICT z)
{
    *x = var_mpu6050_accelerometer[0];
    *y = var_mpu6050_accelerometer[1];
    *z = var_mpu6050_accelerometer[2];

    // these should really be compile-time checks
    assert(*x > 0);
    assert(*y > 0);
    assert(*z > 0);
}

/*!
* \brief Retrieves the variances of the MPU6050 gyroscope
* \param[out] x The x axis variances
* \param[out] y The y axis variances
* \param[out] z The z axis variances
*/
void mpu6050_var_gyroscope(register fix16_t *const RESTRICT x, register fix16_t *const RESTRICT y, register fix16_t *const RESTRICT z)
{
    *x = var_mpu6050_gyroscope[0];
    *y = var_mpu6050_gyroscope[1];
    *z = var_mpu6050_gyroscope[2];

    // these should really be compile-time checks
    assert(*x > 0);
    assert(*y > 0);
    assert(*z > 0);
}

/*!
* \brief Retrieves the variances of the HMC5883L magnetometer
* \param[out] x The x axis variances
* \param[out] y The y axis variances
* \param[out] z The z axis variances
*/
void hmc5883l_var(register fix16_t *const RESTRICT x, register fix16_t *const RESTRICT y, register fix16_t *const RESTRICT z)
{
    *x = var_hmc5883l[0];
    *y = var_hmc5883l[1];
    *z = var_hmc5883l[2];

    // these should really be compile-time checks
    assert(*x > 0);
    assert(*y > 0);
    assert(*z > 0);
}

/*!
* \brief Calibrates a given sensor using a 3x4 affine transformation
* \param[inout] x The x data (will be overwritten with the calibrated version)
* \param[inout] y The y data (will be overwritten with the calibrated version)
* \param[inout] z The z data (will be overwritten with the calibrated version)
*/
STATIC_INLINE HOT NONNULL
void sensor_calibrate(register const mf16 *const RESTRICT calibration_data, register fix16_t *const RESTRICT x, register fix16_t *const RESTRICT y, register fix16_t *const RESTRICT z)
{
    // cached input
    const fix16_t input[4] = { *x, *y, *z, fix16_one };

    assert(calibration_data->rows == 3);
    assert(calibration_data->columns == 4);

    // calculate x
    *x = fa16_dot(
        &input[0], 1,
        &calibration_data->data[0][0], 1,
        4);

    // calculate y
    *y = fa16_dot(
        &input[0], 1,
        &calibration_data->data[1][0], 1,
        4);

    // calculate z
    *z = fa16_dot(
        &input[0], 1,
        &calibration_data->data[2][0], 1,
        4);
}

/*!
* \brief Calibrates MPU6050 accelerometer sensor data
* \param[inout] x The x data (will be overwritten with the calibrated version)
* \param[inout] y The y data (will be overwritten with the calibrated version)
* \param[inout] z The z data (will be overwritten with the calibrated version)
*/
HOT NONNULL
void mpu6050_calibrate_accelerometer(register fix16_t *const RESTRICT x, register fix16_t *const RESTRICT y, register fix16_t *const RESTRICT z)
{
    sensor_calibrate(&mpu6050_accelerometer_calibration_data, x, y, z);
}

/*!
* \brief Calibrates MPU6050 gyroscope sensor data
* \param[inout] x The x data (will be overwritten with the calibrated version)
* \param[inout] y The y data (will be overwritten with the calibrated version)
* \param[inout] z The z data (will be overwritten with the calibrated version)
*/
HOT NONNULL
void mpu6050_calibrate_gyroscope(register fix16_t *x, register fix16_t *y, register fix16_t *z)
{
    sensor_calibrate(&mpu6050_gyroscope_calibration_data, x, y, z);
}

/*!
* \brief Calibrates HMC5883L magnetometer sensor data
* \param[inout] x The x data (will be overwritten with the calibrated version)
* \param[inout] y The y data (will be overwritten with the calibrated version)
* \param[inout] z The z data (will be overwritten with the calibrated version)
*/
HOT NONNULL
void hmc5883l_calibrate(register fix16_t *x, register fix16_t *y, register fix16_t *z)
{
    sensor_calibrate(&hmc5883l_calibration_data, x, y, z);
}