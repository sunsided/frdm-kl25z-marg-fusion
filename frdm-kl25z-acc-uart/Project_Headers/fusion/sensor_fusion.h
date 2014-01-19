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

/*!
* \brief Initializes the sensor fusion mechanism.
*/
void fusion_initialize() COLD;

/*!
* \brief Fetches the values without any modification
* \param[out] roll The roll angle in degree.
* \param[out] pitch The pitch (elevation) angle in degree.
* \param[out] yaw The yaw (heading, azimuth) angle in degree.
*/
void fetch_values(register fix16_t *const roll, register fix16_t *const pitch, register fix16_t *const yaw) HOT PURE NONNULL;

/*!
* \brief Performs a prediction of the current Euler angles based on the time difference to the prediction or observation update call.
* \param[in] deltaT The time difference in seconds to the last prediction or observation update call.
*/
void fusion_predict(register const fix16_t deltaT) HOT;

/*!
* \brief Updates the current prediction with accelerometer measurements
* \param[in] ax The x-axis accelerometer value.
* \param[in] ay The y-axis accelerometer value.
* \param[in] az The z-axis accelerometer value.
*/
void fusion_set_accelerometer(register const fix16_t *const ax, register const fix16_t *const ay, register const fix16_t *const az) LEAF HOT NONNULL;

/*!
* \brief Updates the current prediction with gyroscope measurements
* \param[in] gx The x-axis gyroscope value.
* \param[in] gy The y-axis gyroscope value.
* \param[in] gz The z-axis gyroscope value.
*/
void fusion_set_gyroscope(register const fix16_t *const gx, register const fix16_t *const gy, register const fix16_t *const gz) LEAF HOT NONNULL;

/*!
* \brief Updates the current prediction with magnetometer measurements
* \param[in] mx The x-axis magnetometer value.
* \param[in] my The y-axis magnetometer value.
* \param[in] mz The z-axis magnetometer value.
*/
void fusion_set_magnetometer(register const fix16_t *const mx, register const fix16_t *const my, register const fix16_t *const mz) LEAF HOT NONNULL;

/*!
* \brief Updates the current prediction with the set measurements.
* \param[in] deltaT The time difference in seconds to the last prediction or observation update call.
*/
void fusion_update(register const fix16_t deltaT) HOT;

#endif // SENSOR_FUNCTION_H_