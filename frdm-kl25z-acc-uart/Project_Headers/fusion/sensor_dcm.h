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
void sensor_dcm(mf16 *const dcm, const v3d *RESTRICT const a, const v3d *RESTRICT const m) HOT NONNULL;

/*!
* \brief Gets roll, pitch, yaw from DCM.
* \param[in] dcm The DCM matrix
* \param[out] roll The roll angle in radians.
* \param[out] pitch The pitch(elevation) angle in radians.
* \param[out] yaw The yaw(heading, azimuth) angle in radians.
*/
void sensor_dcm2rpy(const mf16 *RESTRICT const dcm, fix16_t *RESTRICT const roll, fix16_t *RESTRICT const pitch, fix16_t *RESTRICT const yaw) HOT NONNULL;

/*!
* \brief Gets roll, pitch, yaw angular velocities from difference DCM.
* \param[in] current_dcm The DCM matrix
* \param[in] previous_dcm The previous time step DCM matrix
* \param[out] omega_roll The roll anglular velocity in radians/s.
* \param[out] omega_pitch The pitch(elevation) anglular velocity in radians/s.
* \param[out] omega_yaw The yaw(heading, azimuth) anglular velocity in radians/s.
*/
void sensor_ddcm(const mf16 *RESTRICT const current_dcm, const mf16 *RESTRICT const previous_dcm, fix16_t *RESTRICT const omega_roll, fix16_t *RESTRICT const omega_pitch, fix16_t *RESTRICT const omega_yaw) HOT NONNULL;

/*!
* \brief Gets the last calculated coordinate system
* \param[out] x The X axis
* \param[out] y The Y axis
* \param[out] z The Z axis
*/
void sensor_get_csys(v3d *x, v3d *y, v3d *z);

#endif // SENSOR_DDCM_H_