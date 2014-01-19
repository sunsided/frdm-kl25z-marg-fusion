/*
* sensor_ddcm.h
*
*  Created on: Jan 19, 2014
*      Author: Markus
*/

#ifndef SENSOR_DDCM_H_
#define SENSOR_DDCM_H_

#include "compiler.h"
#include "fixmatrix.h"
#include "fixvector3d.h"

/*!
* \brief Builds a DCM from the given calibrated sensor values
* \param[out] dcm The DCM matrix to write to
* \param[in] a The accelerometer vector.
* \param[in] m The magnetometer vector
*/
void sensor_dcm(mf16 *const dcm, 
    register const v3d *RESTRICT const a, register const v3d *RESTRICT const m) HOT NONNULL;

/*!
* \brief Gets roll, pitch, yaw from DCM.
* \param[out] roll The roll angle in radians.
* \param[out] pitch The pitch(elevation) angle in radians.
* \param[out] yaw The yaw(heading, azimuth) angle in radians.
*/
void sensor_dcm2rpy(mf16 *const dcm, fix16_t *RESTRICT const roll, fix16_t *RESTRICT const pitch, fix16_t *RESTRICT const yaw) HOT NONNULL;

#endif // SENSOR_DDCM_H_