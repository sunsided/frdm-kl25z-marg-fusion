#include "fixmath.h"
#include "fusion/sensor_dcm.h"

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
    register const fix16_t *const mx, register const fix16_t *const my, register const fix16_t *const mz)
{


    /*
        The direction cosine matrix is defined as the matrix of dot products
        between all vector components.

        DCM = [ ...
            dot(X, x),  dot(Y, x),  dot(Z, x);
            dot(X, y),  dot(Y, y),  dot(Z, y);
            dot(X, z),  dot(Y, z),  dot(Z, z);
            ];

        The interesting part for us is that the reference system is
        the world coordinate system in unit vectors, so the dot products
        essentially only select components.
    */

    fix16_t x[3], y[3], z[3];

    ddcm->data[0][0] = x[0];
    ddcm->data[0][1] = y[0];
    ddcm->data[0][2] = z[0];

    ddcm->data[1][0] = x[1];
    ddcm->data[1][1] = y[1];
    ddcm->data[1][2] = z[1];

    ddcm->data[2][0] = x[2];
    ddcm->data[2][1] = y[2];
    ddcm->data[2][2] = z[2];
}