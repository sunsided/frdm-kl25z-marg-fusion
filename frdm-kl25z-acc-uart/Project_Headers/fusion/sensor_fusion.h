/*
* sensor_fusion.h
*
*  Created on: Jan 18, 2014
*      Author: Markus
*/

#ifndef SENSOR_FUNCTION_H_
#define SENSOR_FUNCTION_H_

#include "compiler.h"
#include "fixmath.h"
#include "fixmatrix.h"
#include "fixquat.h"

/*!
* \brief Initializes the sensor fusion mechanism.
*/
COLD
void fusion_initialize();

/*!
* \brief Fetches the values without any modification
* \param[out] roll The roll angle in degree.
* \param[out] pitch The pitch (elevation) angle in degree.
* \param[out] yaw The yaw (heading, azimuth) angle in degree.
*/
HOT NONNULL LEAF
void fusion_fetch_angles(register fix16_t *RESTRICT const roll, register fix16_t *RESTRICT const pitch, register fix16_t *RESTRICT const yaw);

/*!
* \brief Fetches the orientation quaternion.
* \param[out] quat The orientation quaternion
*/
HOT NONNULL LEAF
void fusion_fetch_quaternion(register qf16 *RESTRICT const quat);

/*!
* \brief Performs a prediction of the current Euler angles based on the time difference to the prediction or observation update call.
* \param[in] deltaT The time difference in seconds to the last prediction or observation update call.
*/
void fusion_predict(register const fix16_t deltaT) HOT;

/*!
* \brief Registers accelerometer measurements for the next update
* \param[in] ax The x-axis accelerometer value.
* \param[in] ay The y-axis accelerometer value.
* \param[in] az The z-axis accelerometer value.
*/
void fusion_set_accelerometer(register const fix16_t *const ax, register const fix16_t *const ay, register const fix16_t *const az) LEAF HOT NONNULL;

/*!
* \brief Registers accelerometer measurements for the next update
* \param[in] data The accelerometer data.
*/
STATIC_INLINE void fusion_set_accelerometer_v3d(register const v3d *const data)
{
    fusion_set_accelerometer(&data->x, &data->y, &data->z);
}

/*!
* \brief Registers gyroscope measurements for the next update
* \param[in] gx The x-axis gyroscope value.
* \param[in] gy The y-axis gyroscope value.
* \param[in] gz The z-axis gyroscope value.
*/
void fusion_set_gyroscope(register const fix16_t *const gx, register const fix16_t *const gy, register const fix16_t *const gz) LEAF HOT NONNULL;

/*!
* \brief Registers gyroscope measurements for the next update
* \param[in] data The gyroscope data.
*/
STATIC_INLINE void fusion_set_gyroscope_v3d(register const v3d *const data)
{
    fusion_set_gyroscope(&data->x, &data->y, &data->z);
}

/*!
* \brief Registers magnetometer measurements for the next update
* \param[in] mx The x-axis magnetometer value.
* \param[in] my The y-axis magnetometer value.
* \param[in] mz The z-axis magnetometer value.
*/
void fusion_set_magnetometer(register const fix16_t *const mx, register const fix16_t *const my, register const fix16_t *const mz) LEAF HOT NONNULL;

/*!
* \brief Registers magnetometer measurements for the next update
* \param[in] data The magnetometer data.
*/
STATIC_INLINE void fusion_set_magnetometer_v3d(register const v3d *const data)
{
    fusion_set_magnetometer(&data->x, &data->y, &data->z);
}

/*!
* \brief Updates the current prediction with the set measurements.
* \param[in] deltaT The time difference in seconds to the last prediction or observation update call.
*/
void fusion_update(register const fix16_t deltaT) HOT;

#endif // SENSOR_FUNCTION_H_
