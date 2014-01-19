#include "fixmath.h"
#include "fusion/sensor_calibration.h"
#include "fusion/sensor_prepare.h"

/*!
* \brief Converts a raw MPU6050 2.12 (signed 1.12) sensor value to fix16_t
* \param[in] raw The sensor value
* \return The converted value
*/
HOT CONST
STATIC_INLINE fix16_t fix16_from_raw_mpu6050(register uint16_t raw)
{
    // MPU6050 sensor data is in 14bit 2.12 format including sign.
    // In order to convert to Q16 format, we simply copy over (keeping the sign)
    // and then shift left by two.

    return (fix16_t)(((uint32_t)raw) << 2);
}

/*!
* \brief Prepares MPU6050 accelerometer sensor data for fusion by converting and calibrating them.
* \param[out] out The prepared sensor data
* \param[in] raw_x The sensor x value
* \param[in] raw_y The sensor y value
* \param[in] raw_z The sensor z value
* \param[in] scaling The scaling factor, e.g. F16(8192) for 4g mode, F16(16384) for 4g mode, ...
*/
void sensor_prepare_mpu6050_accelerometer_data(v3d *const out, uint16_t rawx, uint16_t rawy, uint16_t rawz, const fix16_t scaling)
{
    // fetch value
    v3d value = {
        fix16_from_raw_mpu6050(rawx),
        fix16_from_raw_mpu6050(rawy),
        fix16_from_raw_mpu6050(rawz)
    };

    // scale
    v3d_div_s(out, &value, scaling);

    // calibrate!
    mpu6050_calibrate_accelerometer_v3d(out);
}

/*!
* \brief Prepares MPU6050 gyroscope sensor data for fusion by converting and calibrating them.
* \param[out] out The prepared sensor data
* \param[in] raw_x The sensor x value
* \param[in] raw_y The sensor y value
* \param[in] raw_z The sensor z value
* \param[in] scaling The scaling factor, e.g. F16(131) for 250°/s mode ...
*/
void sensor_prepare_mpu6050_gyroscope_data(v3d *const out, uint16_t rawx, uint16_t rawy, uint16_t rawz, const fix16_t scaling)
{
    // fetch value
    v3d value = {
        fix16_from_raw_mpu6050(rawx),
        fix16_from_raw_mpu6050(rawy),
        fix16_from_raw_mpu6050(rawz)
    };

    // scale
    v3d_div_s(out, &value, scaling);

    // calibrate!
    mpu6050_calibrate_gyroscope_v3d(out);
}

/*!
* \brief Prepares HMC5883L magnetometer sensor data for fusion by converting and calibrating them.
* \param[out] out The prepared sensor data
* \param[in] raw_x The sensor x value
* \param[in] raw_y The sensor y value
* \param[in] raw_z The sensor z value
* \param[in] scaling The scaling factor, e.g. F16(1090) for 1.3 gauss mode ...
*/
void sensor_prepare_hmc5883l_data(v3d *const out, uint16_t rawx, uint16_t rawy, uint16_t rawz, const fix16_t scaling)
{
    // fetch value
    const register fix16_t invScaling = fix16_div(F16(1), scaling);
    out->x = fix16_mul(fix16_from_int(rawx), invScaling);
    out->y = fix16_mul(fix16_from_int(rawx), invScaling);
    out->z = fix16_mul(fix16_from_int(rawx), invScaling);

    // calibrate!
    hmc5883l_calibrate_v3d(out);
}