#include "fixmath.h"
#include "fixarray.h"
#include "fusion/sensor_dcm.h"

/*!
* \brief Builds a DCM from the given calibrated sensor values
* \param[out] dcm The DCM matrix to write to
* \param[in] a The accelerometer vector.
* \param[in] m The magnetometer vector
*/
void sensor_dcm(mf16 *const dcm,
    register const v3d *RESTRICT const a, register const v3d *RESTRICT const m)
{
    // TODO: check pointer to array dereferencing

#if 0

    // calculate the vector norms for normalization
    register const fix16_t norm_a = fa16_norm(&(*a)[0], 1, 3);
    register const fix16_t norm_m = fa16_norm(&(*m)[0], 1, 3);

    // invert norm to speed up following calculations
    register const fix16_t inv_norm_a = fix16_div(F16(1), norm_a);
    register const fix16_t inv_norm_m = fix16_div(F16(1), norm_m);

    // normalize vectors
    const fix16_t an[3] = {
        fix16_mul((*a)[0], inv_norm_a),
        fix16_mul((*a)[1], inv_norm_a),
        fix16_mul((*a)[2], inv_norm_a)
    };

    const fix16_t mn[3] = {
        fix16_mul((*m)[0], inv_norm_m),
        fix16_mul((*m)[1], inv_norm_m),
        fix16_mul((*m)[2], inv_norm_m)
    };

#else

    v3d an, mn;
    v3d_normalize(&an, a);
    v3d_normalize(&mn, m);

#endif

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

    dcm->data[0][0] = X.x;
    dcm->data[0][1] = Y.x;
    dcm->data[0][2] = Z.x;

    dcm->data[1][0] = X.y;
    dcm->data[1][1] = Y.y;
    dcm->data[1][2] = Z.y;

    dcm->data[2][0] = X.z;
    dcm->data[2][1] = Y.z;
    dcm->data[2][2] = Z.z;
}

/*!
* \brief Gets roll, pitch, yaw from DCM.
* \param[out] roll The roll angle in radians.
* \param[out] pitch The pitch(elevation) angle in radians.
* \param[out] yaw The yaw(heading, azimuth) angle in radians.
*/
void sensor_dcm2rpy(mf16 *const dcm, fix16_t *RESTRICT const roll, fix16_t *RESTRICT const pitch, fix16_t *RESTRICT const yaw)
{
    // extract angles
    // see: William Premerlani, "Computing Euler Angles from Direction Cosines"
    *pitch = -fix16_asin(dcm->data[0][2]);
    *roll = fix16_atan2(dcm->data[1][2], dcm->data[2][2]);
    *yaw = fix16_atan2(dcm->data[0][1], dcm->data[0][0]);
}