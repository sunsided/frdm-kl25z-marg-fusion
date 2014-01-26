#include "fixmath.h"
#include "fusion/sensor_calibration.h"
#include "fusion/sensor_prepare.h"

/*!
* \brief Prepares MPU6050 accelerometer sensor data for fusion by converting and calibrating them.
* \param[out] out The prepared sensor data
* \param[in] raw_x The sensor x value
* \param[in] raw_y The sensor y value
* \param[in] raw_z The sensor z value
* \param[in] scaling The scaling factor, e.g. F16(8192) for 4g mode, F16(16384) for 4g mode, ...
*/
void sensor_prepare_mpu6050_accelerometer_data(v3d *const out, int16_t rawx, int16_t rawy, int16_t rawz, const fix16_t scaling)
{
    // fetch value
    v3d value = {
        // note that we are swapping X and Y axis here and flipping X and Z signs
        // in order to convert from AHRS to regular coordinate system
        -fix16_from_int(rawy),
         fix16_from_int(rawx),
        -fix16_from_int(rawz)
    };

    // scale
    v3d_div_s(out, &value, scaling);

    // calibrate!
    mpu6050_calibrate_accelerometer_v3d(out);

    value = *out;
    out->x =  value.x;
    out->y = -value.y;
    out->z =  value.z;
}

/*!
* \brief Prepares MPU6050 gyroscope sensor data for fusion by converting and calibrating them.
* \param[out] out The prepared sensor data
* \param[in] raw_x The sensor x value
* \param[in] raw_y The sensor y value
* \param[in] raw_z The sensor z value
* \param[in] scaling The scaling factor, e.g. F16(131) for 250°/s mode ...
*/
void sensor_prepare_mpu6050_gyroscope_data(v3d *const out, int16_t rawx, int16_t rawy, int16_t rawz, const fix16_t scaling)
{
#if 1
    // fetch value
    v3d value = {
        // note that we are swapping X and Y axis here and flipping X sign
        // in order to convert from AHRS to regular coordinate system
        -fix16_from_int(rawy),
         fix16_from_int(rawx),
         fix16_from_int(rawz)
    };
#else
    // fetch value
    v3d value = {
        fix16_from_int(rawx),
        fix16_from_int(rawy),
        fix16_from_int(rawz)
    };
#endif

#if 0
    float tx = fix16_to_float(value.x);
    float ty = fix16_to_float(value.y);
    float tz = fix16_to_float(value.z);
#endif

    // scale
    v3d_div_s(out, &value, scaling);

    // calibrate!
    mpu6050_calibrate_gyroscope_v3d(out);

    // data in degree, so convert to radians

#if 1
    out->x = fix16_deg_to_rad( out->x);
    out->y = fix16_deg_to_rad(-out->y);
    out->z = fix16_deg_to_rad(-out->z);
#else
    out->x = fix16_deg_to_rad(-out->x);
    out->y = fix16_deg_to_rad(-out->y);
    out->z = fix16_deg_to_rad( out->z);
#endif
}

/*!
* \brief Prepares HMC5883L magnetometer sensor data for fusion by converting and calibrating them.
* \param[out] out The prepared sensor data
* \param[in] raw_x The sensor x value
* \param[in] raw_y The sensor y value
* \param[in] raw_z The sensor z value
* \param[in] scaling The scaling factor, e.g. F16(1090) for 1.3 gauss mode ...
*/
void sensor_prepare_hmc5883l_data(v3d *const out, int16_t rawx, int16_t rawy, int16_t rawz, const fix16_t scaling)
{
    // fetch value
    const register fix16_t invScaling = fix16_div(F16(1), scaling);
    out->x = fix16_mul(fix16_from_int(rawx), invScaling);
    out->y = fix16_mul(fix16_from_int(rawy), invScaling);
    out->z = fix16_mul(fix16_from_int(rawz), invScaling);

    // calibrate!
    hmc5883l_calibrate_v3d(out);

    v3d value = *out;
    out->x = value.x;
    out->y = value.y;
    out->z = value.z;
}