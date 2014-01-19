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

/*!
* \brief Builds a DCM from the given calibrated sensor values
* \param[out] ddcm The DCM matrix to write to
* \param[in] ax The x-axis accelerometer value.
* \param[in] ay The y-axis accelerometer value.
* \param[in] az The z-axis accelerometer value.
* \param[in] gx The x-axis gyroscope value.
* \param[in] gy The y-axis gyroscope value.
* \param[in] gz The z-axis gyroscope value.
*/
void sensor_dcm(mf16 *const ddcm, 
    register const fix16_t *const ax, register const fix16_t *const ay, register const fix16_t *const az,
    register const fix16_t *const mx, register const fix16_t *const my, register const fix16_t *const mz) HOT NONNULL;

#endif // SENSOR_DDCM_H_