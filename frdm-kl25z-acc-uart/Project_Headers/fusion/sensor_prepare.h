/*
* sensor_prepare.h
*
*  Created on: Jan 19, 2014
*      Author: Markus
*/

#ifndef SENSOR_PREPARE_H_
#define SENSOR_PREPARE_H_

#include "fixvector3d.h"
#include "compiler.h"

/*!
* \brief Prepares MPU6050 accelerometer sensor data for fusion by converting and calibrating them.
* \param[out] out The prepared sensor data
* \param[in] raw_x The sensor x value
* \param[in] raw_y The sensor y value
* \param[in] raw_z The sensor z value
* \param[in] scaling The scaling factor, e.g. 8192 for 4g mode, 16384 for 4g mode, ...
*/
void sensor_prepare_mpu6050_accelerometer_data(v3d *const out, uint16_t rawx, uint16_t rawy, uint16_t rawz, const uint_fast16_t scaling) HOT NONNULL;

/*!
* \brief Prepares MPU6050 gyroscope sensor data for fusion by converting and calibrating them.
* \param[out] out The prepared sensor data
* \param[in] raw_x The sensor x value
* \param[in] raw_y The sensor y value
* \param[in] raw_z The sensor z value
* \param[in] scaling The scaling factor, e.g. 131 for 250°/s mode ...
*/
void sensor_prepare_mpu6050_gyroscope_data(v3d *const out, uint16_t rawx, uint16_t rawy, uint16_t rawz, const uint_fast16_t scaling) HOT NONNULL;

/*!
* \brief Prepares HMC5883L magnetometer sensor data for fusion by converting and calibrating them.
* \param[out] out The prepared sensor data
* \param[in] raw_x The sensor x value
* \param[in] raw_y The sensor y value
* \param[in] raw_z The sensor z value
* \param[in] scaling The scaling factor, e.g. 1090 for 1.3 gauss mode ...
*/
void sensor_prepare_hmc5883l_data(v3d *const out, uint16_t rawx, uint16_t rawy, uint16_t rawz, const uint_fast16_t scaling) HOT NONNULL;

#endif