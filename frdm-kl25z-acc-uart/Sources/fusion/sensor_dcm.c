#include "fixmath.h"
#include "fusion/sensor_dcm.h"

static v3d coordinate_system[3] = { { F16(1), 0, 0 }, { 0, F16(1), 0 }, { 0, 0, F16(1) } };

/*!
* \brief Gets the last calculated coordinate system
* \param[out] x The X axis
* \param[out] y The Y axis
* \param[out] z The Z axis
*/
void sensor_get_csys(v3d *x, v3d *y, v3d *z) 
{
    *x = coordinate_system[0];
    *y = coordinate_system[2];
    *z = coordinate_system[3];
}

/*!
* \brief Builds a DCM from the given calibrated sensor values
* \param[out] dcm The DCM matrix to write to
* \param[in] a The accelerometer vector.
* \param[in] m The magnetometer vector
*/
void sensor_dcm(mf16 *const dcm,
    const v3d *RESTRICT const a, const v3d *RESTRICT const m)
{

    v3d an, mn;
    v3d_normalize(&an, a);
    v3d_normalize(&mn, m);

    // define coordinate system
    v3d X, Y, Z;

    // after normalization, a (positive up) is the Z axis
    Z = an;

    // after normalization, m lies along the X axis
    // note that X and Z are most probably not orthogonal due to the
    // fact that m follows the magnetic field.
    // we will correct for that later
    X = mn;

    // now we calculate Y by crossing X and Z and renormalize
    v3d_cross(&Y, &X, &Z);
    v3d_normalize(&Y, &Y);

    // finally we recreate X from Z and Y
    v3d_cross(&X, &Z, &Y);
    v3d_normalize(&X, &X);

    // and now, Z from X and Y
    v3d_cross(&Z, &X, &Y);
    v3d_normalize(&Z, &Z);

    coordinate_system[0] = X;
    coordinate_system[1] = Y;
    coordinate_system[2] = Z;

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

#if 1
    dcm->data[0][0] = X.x;
    dcm->data[0][1] = Y.x;
    dcm->data[0][2] = Z.x;

    dcm->data[1][0] = X.y;
    dcm->data[1][1] = Y.y;
    dcm->data[1][2] = Z.y;

    dcm->data[2][0] = X.z;
    dcm->data[2][1] = Y.z;
    dcm->data[2][2] = Z.z;
#else
    dcm->data[0][0] = X.x;
    dcm->data[1][0] = Y.x;
    dcm->data[2][0] = Z.x;

    dcm->data[0][1] = X.y;
    dcm->data[1][1] = Y.y;
    dcm->data[2][1] = Z.y;

    dcm->data[0][2] = X.z;
    dcm->data[1][2] = Y.z;
    dcm->data[2][2] = Z.z;
#endif

    dcm->rows = dcm->columns = 3;
}

/*!
* \brief Gets roll, pitch, yaw from DCM.
* \param[out] roll The roll angle in radians.
* \param[out] pitch The pitch(elevation) angle in radians.
* \param[out] yaw The yaw(heading, azimuth) angle in radians.
*/
void sensor_dcm2rpy(const mf16 *RESTRICT const dcm, fix16_t *RESTRICT const roll, fix16_t *RESTRICT const pitch, fix16_t *RESTRICT const yaw)
{
#if 1
    // extract angles
    // see: William Premerlani, "Computing Euler Angles from Direction Cosines"
    *pitch = -fix16_asin(dcm->data[0][2]);
    *roll = fix16_atan2(dcm->data[1][2], dcm->data[2][2]);
    *yaw = fix16_atan2(dcm->data[0][1], dcm->data[0][0]);
#else
    *pitch = -fix16_asin(dcm->data[2][0]);
    *roll = fix16_atan2(dcm->data[2][1], dcm->data[2][2]);
    *yaw = fix16_atan2(dcm->data[1][0], dcm->data[0][0]);
#endif
}

/*!
* \brief Gets roll, pitch, yaw angular velocities from difference DCM.
* \param[in] current_dcm The DCM matrix
* \param[in] previous_dcm The previous time step DCM matrix
* \param[out] omega_roll The roll anglular velocity in radians/s.
* \param[out] omega_pitch The pitch(elevation) anglular velocity in radians/s.
* \param[out] omega_yaw The yaw(heading, azimuth) anglular velocity in radians/s.
*/
void sensor_ddcm(const mf16 *RESTRICT const current_dcm, const mf16 *RESTRICT const previous_dcm, fix16_t *RESTRICT const omega_roll, fix16_t *RESTRICT const omega_pitch, fix16_t *RESTRICT const omega_yaw)
{
    mf16 ddcm;
    mf16_mul_bt(&ddcm, current_dcm, previous_dcm);
    sensor_dcm2rpy(&ddcm, omega_roll, omega_pitch, omega_yaw);
}