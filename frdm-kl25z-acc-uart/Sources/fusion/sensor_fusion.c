#include <assert.h>
#include <stdbool.h>

#include "fixmath.h"
#include "fixkalman.h"
#include "fixvector3d.h"
#include "fixmatrix.h"

#if !defined(FIXMATRIX_MAX_SIZE) || (FIXMATRIX_MAX_SIZE < 6)
#error FIXMATRIX_MAX_SIZE must be defined to value greater or equal 6.
#endif

#include "fusion/sensor_dcm.h"
#include "fusion/sensor_fusion.h"

/************************************************************************/
/* Measurement covariance definitions                                   */
/************************************************************************/

static const fix16_t initial_r_axis = F16(0.02);
static const fix16_t initial_r_gyro = F16(0.001);

static const fix16_t q_axis = F16(0.4);
static const fix16_t q_gyro = F16(1);

static const fix16_t alpha1 = F16(10);
static const fix16_t alpha2 = F16(0.4);

/************************************************************************/
/* Kalman filter structure definition                                   */
/************************************************************************/

/*!
* \brief The Kalman filter instance used to predict the orientation.
*/
static kalman16_uc_t kf_attitude;

/*!
* \def KF_STATES Number of states
*/
#define KF_ATTITUDE_STATES 6

/*!
* \brief The Kalman filter instance used to predict the orientation.
*/
static kalman16_uc_t kf_orientation;

/*!
* \def KF_STATES Number of states
*/
#define KF_ORIENTATION_STATES 6

/*!
* \brief The Kalman filter observation instance used to update the prediction with accelerometer data
*/
static kalman16_observation_t kfm_accel;

/*!
* \def KFM_ACCEL Number of observation variables for accelerometer updates
*/
#define KFM_ACCEL 6

/*!
* \brief The Kalman filter observation instance used to update the prediction with magnetometer data
*/
static kalman16_observation_t kfm_magneto;

/*!
* \def KFM_MAGNETO Number of observation variables for magnetometer updates
*/
#define KFM_MAGNETO 6

/*!
* \brief The Kalman filter observation instance used to update the prediction with magnetometer data
*/
static kalman16_observation_t kfm_gyro;

/*!
* \def KFM_GYRO Number of observation variables for gyroscope-only updates
*/
#define KFM_GYRO 3

/*!
* \brief Lambda parameter for certainty tuning
*/
static fix16_t lambda = F16(1);

/************************************************************************/
/* Sensor data buffers                                                  */
/************************************************************************/

/*!
* \brief The current accelerometer measurements
*/
static v3d m_accelerometer = { 0, 0, 0 };

/*!
* \brief The current gyroscope measurements
*/
static v3d m_gyroscope = { 0, 0, 0 };

/*!
* \brief The current magnetometer measurements
*/
static v3d m_magnetometer = { 0, 0, 0 };

/*!
* \brief Determines if an accelerometer measurement is available
*/
static bool m_have_accelerometer = false;

/*!
* \brief Determines if an gyroscope measurement is available
*/
static bool m_have_gyroscope = false;

/*!
* \brief Determines if an magnetometer measurement is available
*/
static bool m_have_magnetometer = false;

/************************************************************************/
/* Filter bootstrapping                                                 */
/************************************************************************/

/*!
* \brief Determines if the attitude filter was already bootstrapped
*/
static bool m_attitude_bootstrapped = false;

/*!
* \brief Determines if the orientation filter was already bootstrapped
*/
static bool m_orientation_bootstrapped = false;

/************************************************************************/
/* Helper macros                                                        */
/************************************************************************/

/*!
* \def matrix_set Helper macro to set a value in a specific matrix field
*/
#define matrix_set(matrix, row, column, value) \
    assert(row <= FIXMATRIX_MAX_SIZE); \
    assert(column <= FIXMATRIX_MAX_SIZE); \
    assert(row < matrix->rows); \
    assert(column < matrix->columns); \
    matrix->data[row][column] = value

/*!
* \def matrix_set_symmetric Helper macro to set a value symmetrically in a matrix
*/
#define matrix_set_symmetric(matrix, row, column, value) \
    assert(row <= FIXMATRIX_MAX_SIZE); \
    assert(column <= FIXMATRIX_MAX_SIZE); \
    assert(row <= matrix->rows); \
    assert(column <= matrix->columns); \
    assert(row <= matrix->columns); \
    assert(column <= matrix->rows); \
    matrix->data[row][column] = value; \
    matrix->data[column][row] = value


/*!
* \def F16_ONE The value 1 in Q16
*/
#define F16_ONE             (F16(1))

/*!
* \def F16_ONE_HALF The value 0.5 in Q16
*/
#define F16_ONE_HALF        (F16(0.5))

/*!
* \def T Helper macro to fetch the time delta. Value is required to be fix16_t.
*
* This macro has no functionality by itself but makes the intention clear.
*/
#define dT(dt)    (dt)

/*!
* \def T Helper macro to fetch 0.5*dT^2.  Value is required to be fix16_t.
*/
#define half_dT_square(dt)    (fix16_mul(F16_ONE_HALF, fix16_sq(dt)))

/*!
* \def T Helper macro for the initial value of dT
*/
#define initial_dT          (F16_ONE)

/************************************************************************/
/* Helper functions                                                     */
/************************************************************************/

// Calculates dest = A + B * s
HOT NONNULL
STATIC_INLINE void mf16_add_scaled(mf16 *dest, const mf16 *RESTRICT a, const mf16 *RESTRICT b, const fix16_t s)
{
    int row, column;

    if (dest->columns != a->columns || dest->rows != a->rows)
        dest->errors |= FIXMATRIX_DIMERR;

    if (a->columns != b->columns || a->rows != b->rows)
        dest->errors |= FIXMATRIX_DIMERR;

    for (row = 0; row < dest->rows; row++)
    {
        for (column = 0; column < dest->columns; column++)
        {
            register const fix16_t scaled = fix16_mul(b->data[row][column], s);
            register fix16_t sum = fix16_add(a->data[row][column], scaled);

#ifndef FIXMATH_NO_OVERFLOW
            if (sum == fix16_overflow)
                dest->errors |= FIXMATRIX_OVERFLOW;
#endif

            dest->data[row][column] = sum;
        }
    }
}

/************************************************************************/
/* System initialization                                                */
/************************************************************************/

/*!
* \brief Initializes the state matrix of a specific filter based on its state
*/
HOT NONNULL LEAF
STATIC_INLINE void update_state_matrix_from_state(kalman16_uc_t *const kf)
{
    mf16 *const A = &kf->A;
    const mf16 *const x = &kf->x;

    fix16_t c1 = x->data[0][0];
    fix16_t c2 = x->data[1][0];
    fix16_t c3 = x->data[2][0];

    //matrix_set(A, 0, 3,   0);
    matrix_set(A, 0, 4, -c3);
    matrix_set(A, 0, 5,  c2);

    matrix_set(A, 1, 3,  c3);
    //matrix_set(A, 1, 4,   0);
    matrix_set(A, 1, 5, -c1);

    matrix_set(A, 2, 3, -c2);
    matrix_set(A, 2, 4,  c1);
    //matrix_set(A, 2, 5,   0);
}

/*!
* \brief Initialization of a specific filter
*/
COLD NONNULL
static void initialize_system_filter(kalman16_uc_t *const kf, const uint_fast8_t states)
{
    kalman_filter_initialize_uc(kf, states);

    /************************************************************************/
    /* Prepare initial state estimation                                     */
    /************************************************************************/
    // intentionally left blank, because the initialize function already set the vector to zero

    /************************************************************************/
    /* Set state transition model                                           */
    /************************************************************************/
    update_state_matrix_from_state(kf);

    /************************************************************************/
    /* Set state variances                                                  */
    /************************************************************************/
    {
        mf16 *const P = &kf->P;

        // initial axis (accelerometer/magnetometer) variances
        matrix_set(P, 0, 0, F16(5));
        matrix_set(P, 1, 1, F16(5));
        matrix_set(P, 2, 2, F16(5));

        // initial gyro variances
        matrix_set(P, 3, 3, F16(1));
        matrix_set(P, 4, 4, F16(1));
        matrix_set(P, 5, 5, F16(1));
    }
    
    /************************************************************************/
    /* Set system process noise                                             */
    /************************************************************************/
    {
        mf16 *const Q = &kf->Q;

        // axis process noise
        matrix_set(Q, 0, 0, q_axis);
        matrix_set(Q, 1, 1, q_axis);
        matrix_set(Q, 2, 2, q_axis);

        // gyro process noise
        matrix_set(Q, 3, 3, q_gyro);
        matrix_set(Q, 4, 4, q_gyro);
        matrix_set(Q, 5, 5, q_gyro);
    }
}

/*!
* \brief Initialization of the filter
*/
COLD
static void initialize_system()
{
    initialize_system_filter(&kf_orientation, KF_ORIENTATION_STATES);
    initialize_system_filter(&kf_attitude, KF_ATTITUDE_STATES);

    // set intial state estimate
    kf_attitude.x.data[0][0] = 0;
    kf_attitude.x.data[1][0] = 0;
    kf_attitude.x.data[2][0] = 1;

    kf_orientation.x.data[0][0] = 0;
    kf_orientation.x.data[1][0] = 1;
    kf_orientation.x.data[2][0] = 0;
}

/*!
* \brief Initializes the state matrix of a specific filter based on its state
*/
HOT NONNULL
STATIC_INLINE void update_measurement_noise(kalman16_observation_t *const kfm, register const fix16_t axisXYZ, register const fix16_t gyroXYZ)
{
    mf16 *const R = &kfm->R;

    matrix_set(R, 0, 0, axisXYZ);
    matrix_set(R, 1, 1, axisXYZ);
    matrix_set(R, 2, 2, axisXYZ);

    matrix_set(R, 3, 3, gyroXYZ);
    matrix_set(R, 4, 4, gyroXYZ);
    matrix_set(R, 5, 5, gyroXYZ);
}

/*!
* \brief Detects accelerations
* \return 1 if an external acceleration was detected
*/
HOT
STATIC_INLINE uint_fast8_t acceleration_detected()
{
    register fix16_t alpha = fix16_abs(fix16_sub(fix16_add(fix16_sq(m_accelerometer.x), fix16_add(fix16_sq(m_accelerometer.y), fix16_sq(m_accelerometer.z))), F16(1)));
    const fix16_t threshold = F16(0.14);
    if (alpha < threshold)
    {
        return 0;
    }

    return 1;
}

/*!
* \brief Dynamic measurement noise updating
*/
HOT NONNULL
STATIC_INLINE void tune_measurement_noise(kalman16_observation_t *const kfm)
{
    mf16 *const R = &kfm->R;
  

    matrix_set(R, 0, 0, fix16_mul(initial_r_axis, alpha1));
    matrix_set(R, 1, 1, fix16_mul(initial_r_axis, alpha1));
    matrix_set(R, 2, 2, fix16_mul(initial_r_axis, alpha1));

    matrix_set(R, 3, 3, fix16_mul(initial_r_gyro, alpha2));
    matrix_set(R, 4, 4, fix16_mul(initial_r_gyro, alpha2));
    matrix_set(R, 5, 5, fix16_mul(initial_r_gyro, alpha2));
}

/*!
* \brief Initialization of a specific measurement
*/
COLD
static void initialize_observation(kalman16_observation_t *const kfm, const uint_fast8_t states, const uint_fast8_t observations)
{
    kalman_observation_initialize(kfm, states, observations);

    /************************************************************************/
    /* Set observation model                                                */
    /************************************************************************/
    {
        mf16 *const H = &kfm->H;

        // axes
        matrix_set(H, 0, 0, F16_ONE);
        matrix_set(H, 1, 1, F16_ONE);
        matrix_set(H, 2, 2, F16_ONE);

        // gyro
        matrix_set(H, 3, 3, F16_ONE);
        matrix_set(H, 4, 4, F16_ONE);
        matrix_set(H, 5, 5, F16_ONE);
    }

    /************************************************************************/
    /* Set observation process noise covariance                             */
    /************************************************************************/
    update_measurement_noise(kfm, initial_r_axis, initial_r_gyro);
}

/*!
* \brief Initialization of accelerometer updates
*/
COLD
static void initialize_observation_accel()
{
    kalman16_observation_t *const kfm = &kfm_accel;
    initialize_observation(kfm, KF_ATTITUDE_STATES, KFM_ACCEL);
}

/*!
* \brief Initialization of magnetometer updates
*/
COLD
static void initialize_observation_magneto()
{
    kalman16_observation_t *const kfm = &kfm_magneto;
    initialize_observation(kfm, KF_ORIENTATION_STATES, KFM_MAGNETO);
}

/*!
* \brief Initialization of a specific measurement
*/
COLD
static void initialize_observation_gyro()
{
    kalman_observation_initialize(&kfm_gyro, KF_ORIENTATION_STATES, KFM_GYRO);

    /************************************************************************/
    /* Set observation model                                                */
    /************************************************************************/
    {
        mf16 *const H = &kfm_gyro.H;

        // gyro
        matrix_set(H, 0, 3, F16_ONE);
        matrix_set(H, 1, 4, F16_ONE);
        matrix_set(H, 2, 5, F16_ONE);
    }

    /************************************************************************/
    /* Set observation process noise covariance                             */
    /************************************************************************/
    {
        mf16 *const R = &kfm_gyro.R;

        matrix_set(R, 0, 0, initial_r_gyro);
        matrix_set(R, 1, 1, initial_r_gyro);
        matrix_set(R, 2, 2, initial_r_gyro);
    }

}

/*!
* \brief Initializes the sensor fusion mechanism.
*/
void fusion_initialize()
{
    initialize_system();
    initialize_observation_gyro();
    initialize_observation_accel();
    initialize_observation_magneto();
}

/************************************************************************/
/* State calculation helpers                                            */
/************************************************************************/

/*!
* \brief Sanitizes the state variables
*/
HOT NONNULL
STATIC_INLINE void fusion_sanitize_state(kalman16_uc_t *const kf)
{
    mf16 *const A = &kf->A;
    mf16 *const x = &kf->x;

    // fetch axes
    fix16_t c1 = x->data[0][0];
    fix16_t c2 = x->data[1][0];
    fix16_t c3 = x->data[2][0];

    // calculate vector norm
    fix16_t norm = fix16_sqrt(
                        fix16_add(
                            fix16_add(
                                fix16_sq(c1), 
                                fix16_sq(c2)),
                            fix16_sq(c3))
                    );

    // normalize vectors
    c1 = fix16_div(c1, norm);
    c2 = fix16_div(c2, norm);
    c3 = fix16_div(c3, norm);

    // re-set to state and state matrix
    x->data[0][0] = c1;
    x->data[1][0] = c2;
    x->data[2][0] = c3;
    update_state_matrix_from_state(kf);
}

/*!
* \brief Fetches the sign of a variable
* \return -1 if value is negative, +1 otherwise
*/
HOT CONST
STATIC_INLINE int32_t fix16_sign(const fix16_t value)
{
    return value >= 0 ? 1 : -1;
}

/*!
* \brief Fetches the sign of a variable, 
* \return -1 if value is negative, +1 if value is positive, 0 otherwise
*/
HOT CONST
STATIC_INLINE int32_t fix16_sign_ex(const fix16_t value)
{
    return (value > 0) ? 1 : (value < 0) ? -1 : 0;
}

/*!
* \brief Fetches the values without any modification
* \param[out] roll The roll angle in radians.
* \param[out] pitch The pitch (elevation) angle in radians.
*/
HOT NONNULL LEAF
STATIC_INLINE void calculate_roll_pitch(register fix16_t *RESTRICT const roll, register fix16_t *RESTRICT const pitch)
{
#if 0

    const mf16 *const x = kalman_get_state_vector_uc(&kf_attitude);
    
    // fetch axes
    const fix16_t c31 = x->data[0][0];
    const fix16_t c32 = x->data[1][0];
    const fix16_t c33 = x->data[2][0];
    
    // calculate pitch
    *pitch = -fix16_asin(c31);

    // calculate roll
    const fix16_t c1sq = fix16_sq(c31);
    const fix16_t c3sq = fix16_sq(c33);
    const fix16_t c1c3sq = fix16_add(c1sq, c3sq);
    const fix16_t c1c3norm = fix16_sqrt(c1c3sq);
    *roll = fix16_atan2(c32, fix16_sign(c33)*c1c3norm);
    //*roll = fix16_atan2(c32, c33);
#else

    const mf16 *const x = kalman_get_state_vector_uc(&kf_attitude);

    // fetch axes
    const fix16_t c31 =  x->data[0][0];
    const fix16_t c32 =  x->data[1][0];
    const fix16_t c33 =  x->data[2][0];

    // calculate pitch
    *pitch = -fix16_asin(c31);

    // calculate roll
    const fix16_t c1sq = fix16_sq(c31);
    const fix16_t c3sq = fix16_sq(c33);
    const fix16_t c1c3sq = fix16_add(c1sq, c3sq);
    const fix16_t c1c3norm = fix16_sqrt(c1c3sq);
    *roll = fix16_atan2(c32, fix16_sign(c33)*c1c3norm);
    //*roll = fix16_atan2(c32, c33);

#endif
}

/*!
* \brief Fetches the values without any modification
* \param[in] roll The roll angle in radians.
* \param[in] pitch The pitch (elevation) angle in radians.
* \param[out] yaw The yaw angle in radians
*/
HOT NONNULL LEAF
STATIC_INLINE void calculate_yaw(register const fix16_t roll, register const fix16_t pitch, register fix16_t *RESTRICT const yaw)
{
    const mf16 *const x2 = kalman_get_state_vector_uc(&kf_orientation);
    const mf16 *const x3 = kalman_get_state_vector_uc(&kf_attitude);

    // fetch axes
    const fix16_t c21 = x2->data[0][0];
    const fix16_t c22 = x2->data[1][0];
    const fix16_t c23 = x2->data[2][0];

    const fix16_t c31 = x3->data[0][0];
    const fix16_t c32 = x3->data[1][0];
    const fix16_t c33 = x3->data[2][0];

    // calculate partial cross product for C11
    // C1  = cross([C21 C22 C23], [C31 C32 C33])
    // -->
    //      C11 = C22*C33 - C23*C32
    //      C12 = C23*C31 - C21*C33
    //      C13 = C21*C32 - C22*C31
    fix16_t c11 = fix16_sub(fix16_mul(c22, c33), fix16_mul(c23, c32));

    // calculate pitch
    *yaw = -fix16_atan2(c21, c11);
}

/************************************************************************/
/* Convenience functions                                                */
/************************************************************************/

/*!
* \brief Fetches the values without any modification
* \param[out] roll The roll angle in degree.
* \param[out] pitch The pitch (elevation) angle in degree.
* \param[out] yaw The yaw (heading, azimuth) angle in degree.
*/
HOT NONNULL LEAF
void fusion_fetch_angles(register fix16_t *RESTRICT const roll, register fix16_t *RESTRICT const pitch, register fix16_t *RESTRICT const yaw)
{
    // calculate roll and pitch
    calculate_roll_pitch(roll, pitch);

    // calculate yaw
    calculate_yaw(*roll, *pitch, yaw);
}

/*!
* \brief  Returns the maximum value of two 
*/
HOT CONST LEAF
STATIC_INLINE const fix16_t zero_or_value(register const fix16_t value)
{
    return value >= 0 ? value : 0;
}

/*!
* \brief Fetches the orientation quaternion.
* \param[out] quat The orientation quaternion
*/
HOT NONNULL LEAF
void fusion_fetch_quaternion(register qf16 *RESTRICT const quat)
{   
    const register mf16 *const x2 = kalman_get_state_vector_uc(&kf_orientation);
    const register mf16 *const x3 = kalman_get_state_vector_uc(&kf_attitude);

    // m00 = R(1, 1);    m01 = R(1, 2);    m02 = R(1, 3);
    // m10 = R(2, 1);    m11 = R(2, 2);    m12 = R(2, 3);
    // m20 = R(3, 1);    m21 = R(3, 2);    m22 = R(3, 3);

    const register fix16_t m10 = x2->data[0][0];
    const register fix16_t m11 = x2->data[1][0];
    const register fix16_t m12 = x2->data[2][0];

    const register fix16_t m20 = x3->data[0][0];
    const register fix16_t m21 = x3->data[1][0];
    const register fix16_t m22 = x3->data[2][0];

    // calculate cross product for C1
    // m0 = cross([m10 m11 m12], [m20 m21 m22])
    // -->
    //      m00 = m11*m22 - m12*m21
    //      m01 = m12*m20 - m10*m22
    //      m02 = m10*m21 - m11*m20
    register fix16_t m00 = fix16_sub(fix16_mul(m11, m22), fix16_mul(m12, m21));
    register fix16_t m01 = fix16_sub(fix16_mul(m12, m20), fix16_mul(m10, m22));
    register fix16_t m02 = fix16_sub(fix16_mul(m10, m21), fix16_mul(m11, m20));

    // normalize C1 
    const register fix16_t norm = fix16_sqrt(fix16_add(fix16_sq(m00), fix16_add(fix16_sq(m01), fix16_sq(m02))));
    m00 = fix16_div(m00, norm);
    m01 = fix16_div(m01, norm);
    m02 = fix16_div(m02, norm);

    /*
    From MATLAB code:

    qw = sqrt(max(0, 1 + m00 + m11 + m22)) / 2;
    qx = sqrt(max(0, 1 + m00 - m11 - m22)) / 2;
    qy = sqrt(max(0, 1 - m00 + m11 - m22)) / 2;
    qz = sqrt(max(0, 1 - m00 - m11 + m22)) / 2;

    % patch the signs
    qx = copysign(qx, m21 - m12);
    qy = copysign(qy, m02 - m20);
    qz = copysign(qz, m10 - m01);

    q = [qw qx qy qz];
    */

    // qw = sqrt(max(0, 1 + m00 + m11 + m22)) / 2;
    fix16_t qw = fix16_mul(F16(0.5), fix16_sqrt(zero_or_value(fix16_add(F16(1), fix16_add(m00, fix16_add(m11, m22))))));

    // qx = sqrt(max(0, 1 + m00 - m11 - m22)) / 2;
    //                  1 + m00 - m11 - m22 
    //               =  1 + m00 - (m11 + m22)
    fix16_t qx = fix16_mul(F16(0.5), fix16_sqrt(zero_or_value(fix16_add(F16(1), fix16_sub(m00, fix16_add(m11, m22))))));

    // qy = sqrt(max(0, 1 - m00 + m11 - m22)) / 2;
    //                  1 - m00 + m11 - m22
    //               =  1 - (m00 - m11 + m22)
    //               =  1 - (m00 - (m11 - m22))
    fix16_t qy = fix16_mul(F16(0.5), fix16_sqrt(zero_or_value(fix16_sub(F16(1), fix16_sub(m00, fix16_sub(m11, m22))))));

    // qz = sqrt(max(0, 1 - m00 - m11 + m22)) / 2;
    //                  1 - m00 - m11 + m22 
    //               =  1 - (m00 + m11 - m22)
    //               =  1 - (m00 + (m11 - m22))
    fix16_t qz = fix16_mul(F16(0.5), fix16_sqrt(zero_or_value(fix16_sub(F16(1), fix16_add(m00, fix16_sub(m11, m22))))));
    
#if 1

    // qx = copysign(qx, m21 - m12);
    qx *= fix16_sign_ex(fix16_sub(m21, m12));

    // qy = copysign(qy, m02 - m20);
    qy *= fix16_sign_ex(fix16_sub(m02, m20));

    //  qz = copysign(qz, m10 - m01);
    qz *= fix16_sign_ex(fix16_sub(m10, m01));

#else

    // qx = copysign(qx, m21 - m12);
    qx *= fix16_sign_ex(fix16_sub(m12, m21));

    // qy = copysign(qy, m02 - m20);
    qy *= fix16_sign_ex(fix16_sub(m20, m02));

    //  qz = copysign(qz, m10 - m01);
    qz *= fix16_sign_ex(fix16_sub(m01, m10));

#endif
    
    // compose quaternion
    quat->a = qw;
    quat->b = qx;
    quat->c = qy;
    quat->d = qz;

    // normalizify
    qf16_normalize(quat, quat);
}

/************************************************************************/
/* State prediction                                                     */
/************************************************************************/

/*!
* \brief Performs a fast state update by using knowledge about the matrix structure
* \param[in] kf The filter whose state to update
* \param[in] deltaT The time differential
*/
HOT NONNULL
STATIC_INLINE void fusion_fastpredict_X(kalman16_uc_t *const kf, const register fix16_t deltaT)
{
    mf16 *const x = kalman_get_state_vector_uc(kf);

    /*
        Transition matrix layout:

        A_rp = [0 0 0,     0 -Cn3  Cn2;
                0 0 0,   Cn3    0 -Cn1;
                0 0 0,  -Cn2  Cn1    0;

                0 0 0,     0 0 0;
                0 0 0,     0 0 0;
                0 0 0,     0 0 0];
    */

    // fetch estimated DCM components
    register const fix16_t c1 = x->data[0][0];
    register const fix16_t c2 = x->data[1][0];
    register const fix16_t c3 = x->data[2][0];

    // fetch estimated angular velocities
    register const fix16_t gx = x->data[3][0];
    register const fix16_t gy = x->data[4][0];
    register const fix16_t gz = x->data[5][0];
    
    // solve differential equations
    register const fix16_t d_c1 = fix16_sub(fix16_mul(c2, gz), fix16_mul(c3, gy)); //    0*gx  + (-c3*gy) +   c2*gz  = c2*gz - c3*gy
    register const fix16_t d_c2 = fix16_sub(fix16_mul(c3, gx), fix16_mul(c1, gz)); //   c3*gx  +    0*gy  + (-c1*gz) = c3*gx - c1*gz
    register const fix16_t d_c3 = fix16_sub(fix16_mul(c1, gy), fix16_mul(c2, gx)); // (-c2*gx) +  (c1*gy) +    0*gz  = c1*gy - c2*gx

    // integrate
    x->data[0][0] = fix16_add(c1, fix16_mul(d_c1, deltaT));
    x->data[1][0] = fix16_add(c2, fix16_mul(d_c2, deltaT));
    x->data[2][0] = fix16_add(c3, fix16_mul(d_c3, deltaT));

    // keep constant.
    x->data[3][0] = gx;
    x->data[4][0] = gy;
    x->data[5][0] = gz;
}

/*!
* \brief Performs a prediction of the current Euler angles based on the time difference to the previous prediction/update iteration.
* \param[in] deltaT The time difference in seconds to the last prediction or observation update call.
*/
HOT
void fusion_predict(register const fix16_t deltaT)
{
    mf16 *const x2 = kalman_get_state_vector_uc(&kf_orientation);
    mf16 *const x3 = kalman_get_state_vector_uc(&kf_attitude);
    
    mf16 *const P2 = kalman_get_system_covariance_uc(&kf_orientation);
    mf16 *const P3 = kalman_get_system_covariance_uc(&kf_attitude);
    
    // predict state
    fusion_fastpredict_X(&kf_attitude, deltaT);
    fusion_fastpredict_X(&kf_orientation, deltaT);

    // predict covariance
    kalman_cpredict_P_uc(&kf_attitude, deltaT);
    kalman_cpredict_P_uc(&kf_orientation, deltaT);

    // re-orthogonalize and update state matrix
    fusion_sanitize_state(&kf_attitude);
    fusion_sanitize_state(&kf_orientation);
}

/************************************************************************/
/* Setters for sensor data                                              */
/************************************************************************/

/*!
* \brief Registers accelerometer measurements for the next update
* \param[in] ax The x-axis accelerometer value.
* \param[in] ay The y-axis accelerometer value.
* \param[in] az The z-axis accelerometer value.
*/
void fusion_set_accelerometer(register const fix16_t *const ax, register const fix16_t *const ay, register const fix16_t *const az)
{
    // invert to show up instead of down
    m_accelerometer.x = -*ax;
    m_accelerometer.y = -*ay;
    m_accelerometer.z = -*az;

#if 0
    // check accelerometer norm and discard non-still accelerations
    // 1 - (x^2 + y^2 + z^2) == 0
    fix16_t norm = fix16_sub(F16(1), fix16_sqrt(fix16_add(fix16_sq(m_accelerometer.x), fix16_add(fix16_sq(m_accelerometer.y), fix16_sq(m_accelerometer.z)))));
    if (norm > F16(0.1) || norm < F16(-0.1))
    {
        m_have_accelerometer = false;
        return;
    }
#endif

    m_have_accelerometer = true;
}

/*!
* \brief Registers gyroscope measurements for the next update
* \param[in] gx The x-axis gyroscope value.
* \param[in] gy The y-axis gyroscope value.
* \param[in] gz The z-axis gyroscope value.
*/
void fusion_set_gyroscope(register const fix16_t *const gx, register const fix16_t *const gy, register const fix16_t *const gz)
{
    m_gyroscope.x = *gx;
    m_gyroscope.y = *gy;
    m_gyroscope.z = *gz;
    m_have_gyroscope = true;
}

/*!
* \brief Registers magnetometer measurements for the next update
* \param[in] mx The x-axis magnetometer value.
* \param[in] my The y-axis magnetometer value.
* \param[in] mz The z-axis magnetometer value.
*/
void fusion_set_magnetometer(register const fix16_t *const mx, register const fix16_t *const my, register const fix16_t *const mz)
{
    m_magnetometer.x = *mx;
    m_magnetometer.y = *my;
    m_magnetometer.z = *mz;
    m_have_magnetometer = true;
}

/************************************************************************/
/* State udpate                                                         */
/************************************************************************/

/*!
* \brief Updates the current prediction with gyroscope data
*/
HOT
void fusion_update_attitude_gyro(register const fix16_t deltaT)
{
    /************************************************************************/
    /* Prepare measurement                                                  */
    /************************************************************************/
    {
        mf16 *const z = &kfm_gyro.z;

        matrix_set(z, 0, 0, m_gyroscope.x);
        matrix_set(z, 1, 0, m_gyroscope.y);
        matrix_set(z, 2, 0, m_gyroscope.z);
    }

    /************************************************************************/
    /* Perform Kalman update                                                */
    /************************************************************************/

    kalman_correct_uc(&kf_attitude, &kfm_gyro);

    /************************************************************************/
    /* Re-orthogonalize and update state matrix                             */
    /************************************************************************/

    fusion_sanitize_state(&kf_attitude);
}

/*!
* \brief Updates the current prediction with accelerometer data
*/
HOT
void fusion_update_attitude(register const fix16_t deltaT)
{
    /************************************************************************/
    /* Perform acceleration detection                                       */
    /************************************************************************/

    if (acceleration_detected())
    {
        fusion_update_attitude_gyro(deltaT);
        return;
    }

    /************************************************************************/
    /* Prepare measurement                                                  */
    /************************************************************************/
    {
        mf16 *const z = &kfm_accel.z;

        fix16_t norm = v3d_norm(&m_accelerometer);
        
        matrix_set(z, 0, 0, fix16_div(m_accelerometer.x, norm));
        matrix_set(z, 1, 0, fix16_div(m_accelerometer.y, norm));
        matrix_set(z, 2, 0, fix16_div(m_accelerometer.z, norm));

        matrix_set(z, 3, 0, m_gyroscope.x);
        matrix_set(z, 4, 0, m_gyroscope.y);
        matrix_set(z, 5, 0, m_gyroscope.z);
    }

    /************************************************************************/
    /* Prepare noise                                                        */
    /************************************************************************/

    tune_measurement_noise(&kfm_accel);

    /************************************************************************/
    /* Perform Kalman update                                                */
    /************************************************************************/

    kalman_correct_uc(&kf_attitude, &kfm_accel);

    /************************************************************************/
    /* Re-orthogonalize and update state matrix                             */
    /************************************************************************/

    fusion_sanitize_state(&kf_attitude);
}

/*!
* \brief Projects the current magnetometer readings in m_magnetometer into the X/Y plane
* \param[in] roll The roll angle
* \param[in] pitch The pitch angle
* \param[out] mx The projected x component
* \param[out] my The projected y component
* \param[out] mz The projected z component
* \return Cosine of pitch. Used to detect singularity.
*/
HOT LEAF NONNULL
STATIC_INLINE fix16_t magnetometer_project_ex(const fix16_t roll, const fix16_t pitch, fix16_t *RESTRICT const mx, fix16_t *RESTRICT const my, fix16_t *RESTRICT const mz)
{
    // calculate pitch sine and cosine
    register const fix16_t cos_pitch = fix16_cos(pitch);
    register const fix16_t sin_pitch = fix16_sin(pitch);

    // calculate roll sine and cosine
    register const fix16_t cos_roll = fix16_cos(roll);
    register const fix16_t sin_roll = fix16_sin(roll);

    // calculate intermediate values
    register const fix16_t sin_roll_sin_pitch = fix16_mul(sin_roll, sin_pitch);
    register const fix16_t cos_roll_sin_pitch = fix16_mul(cos_roll, sin_pitch);

    // tilt-compensate magnetometer readings into horizontal (X-Y) plane
    register const fix16_t Xh = fix16_add(fix16_mul(m_magnetometer.x, cos_pitch),
                                          fix16_add(fix16_mul(m_magnetometer.y, sin_roll_sin_pitch),
                                                    fix16_mul(m_magnetometer.z, cos_roll_sin_pitch)
                                                    )
                                          );

    register const fix16_t Yh = fix16_sub(fix16_mul(m_magnetometer.y, cos_roll),
                                          fix16_mul(m_magnetometer.z, sin_roll)
                                          );

    // calculate 2D vector norm
    register const fix16_t pnorm = fix16_sqrt(fix16_add(fix16_sq(Xh), fix16_sq(Yh)));

    // calculate yaw sine and cosine from components
    register const fix16_t sin_yaw = fix16_div(Yh, pnorm);
    register const fix16_t cos_yaw = fix16_div(Xh, pnorm);

    /************************************************************************/
    /* Calculate tilt-compensated readings                                  */
    /************************************************************************/

    *mx = fix16_mul(cos_pitch, sin_yaw);
    *my = fix16_add(fix16_mul(cos_roll, cos_yaw),
                    fix16_mul(sin_roll_sin_pitch, sin_yaw)
                    );
    *mz = fix16_add(fix16_mul(-sin_roll, cos_yaw),
                    fix16_mul(cos_roll_sin_pitch, sin_yaw)
                    );

    return cos_pitch;
}

/*!
* \brief Projects the current magnetometer readings in m_magnetometer into the X/Y plane. Takes the required roll and pitch angles from the estimated state vector.
*        In case of bootstrapping use, update using the accelerometer measurement first.
* \param[out] mx The projected x component
* \param[out] my The projected y component
* \param[out] mz The projected z component
* \return Cosine of pitch. Used to detect singularity.
*/
HOT NONNULL
STATIC_INLINE fix16_t magnetometer_project(fix16_t *const mx, fix16_t *const my, fix16_t *const mz)
{
    fix16_t roll, pitch;
    calculate_roll_pitch(&roll, &pitch);

    // projectify!
    return magnetometer_project_ex(roll, pitch, mx, my, mz);
}

/*!
* \brief Updates the current prediction with gyroscope data
*/
HOT
static void fusion_update_orientation_gyro(register const fix16_t deltaT)
{
    /************************************************************************/
    /* Prepare measurement                                                  */
    /************************************************************************/
    {
        mf16 *const z = &kfm_gyro.z;

        matrix_set(z, 0, 0, m_gyroscope.x);
        matrix_set(z, 1, 0, m_gyroscope.y);
        matrix_set(z, 2, 0, m_gyroscope.z);
    }

    /************************************************************************/
    /* Perform Kalman update                                                */
    /************************************************************************/

    kalman_correct_uc(&kf_orientation, &kfm_gyro);

    /************************************************************************/
    /* Re-orthogonalize and update state matrix                             */
    /************************************************************************/

    fusion_sanitize_state(&kf_orientation);
}

/*!
* \brief Updates the current prediction with magnetometer data
*/
HOT
static void fusion_update_orientation(register const fix16_t deltaT)
{
    /************************************************************************/
    /* Calculate metrics required for update                                */
    /************************************************************************/
    fix16_t mx, my, mz;
    fix16_t cos_pitch = magnetometer_project(&mx, &my, &mz);
    
#if 0
    // check for singularity
    if (cos_pitch < F16(0.17365))
    {
        fusion_update_orientation_gyro(deltaT);
        return;
    }
#endif

    /************************************************************************/
    /* Prepare noise                                                        */
    /************************************************************************/

    tune_measurement_noise(&kfm_magneto);

    /************************************************************************/
    /* Prepare measurement                                                  */
    /************************************************************************/
    {
        mf16 *const z = &kfm_magneto.z;

        matrix_set(z, 0, 0, mx);
        matrix_set(z, 1, 0, my);
        matrix_set(z, 2, 0, mz);

        matrix_set(z, 3, 0, m_gyroscope.x);
        matrix_set(z, 4, 0, m_gyroscope.y);
        matrix_set(z, 5, 0, m_gyroscope.z);
    }

    /************************************************************************/
    /* Perform Kalman update                                                */
    /************************************************************************/

    kalman_correct_uc(&kf_orientation, &kfm_magneto);

    /************************************************************************/
    /* Re-orthogonalize and update state matrix                             */
    /************************************************************************/

    fusion_sanitize_state(&kf_orientation);
}

/*!
* \brief Updates the current prediction with the set measurements.
* \param[in] deltaT The time difference in seconds to the last prediction or observation update call.
*/
void fusion_update(register const fix16_t deltaT)
{
#if 0
#if 0 // force gyro-only
    if (m_attitude_bootstrapped && m_orientation_bootstrapped)
    {
        m_have_accelerometer = false;
        m_have_magnetometer = false;
    }
#else // force axes only
    m_gyroscope.x = 0;
    m_gyroscope.y = 0;
    m_gyroscope.z = 0;
#endif
#endif

    // perform roll and pitch updates
    if (true == m_have_accelerometer)
    {
        // bootstrap filter
        if (false == m_attitude_bootstrapped)
        {
            fix16_t norm = v3d_norm(&m_accelerometer);

            kf_attitude.x.data[0][0] = fix16_div(m_accelerometer.x, norm);
            kf_attitude.x.data[1][0] = fix16_div(m_accelerometer.y, norm);
            kf_attitude.x.data[2][0] = fix16_div(m_accelerometer.z, norm);

            m_attitude_bootstrapped = true;
        }

        fusion_update_attitude(deltaT);
    }
    else
    {
        // perform only rotational update
        fusion_update_attitude_gyro(deltaT);
    }

    // perform yaw updates
    if (true == m_have_magnetometer)
    {
        // bootstrap filter
        // make sure that the attitude filter was already bootstrapped in order to be able to project the
        // magnetometer readings
        if ((false == m_orientation_bootstrapped) && (true == m_attitude_bootstrapped))
        {
            fix16_t mx, my, mz;
            magnetometer_project(&mx, &my, &mz);

            kf_orientation.x.data[0][0] = mx;
            kf_orientation.x.data[1][0] = my;
            kf_orientation.x.data[2][0] = mz;

            m_orientation_bootstrapped = true;
        }

        fusion_update_orientation(deltaT);
    }
    else
    {
        // perform only rotational update
        fusion_update_orientation_gyro(deltaT);
    }
    
    // reset information
    m_have_accelerometer = false;
    m_have_magnetometer = false;
}
