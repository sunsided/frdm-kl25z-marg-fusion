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
* \brief The Kalman filter observation instance used to update the prediction with integrated accelerometer data
*/
static kalman16_observation_t kfm_accel;

/*!
* \def KFM_IDDCM Number of observation variables for integrated difference DCM-only updates
*/
#define KFM_ACCEL 6

/*!
* \brief The Kalman filter observation instance used to update the prediction with magnetometer data
*/
static kalman16_observation_t kfm_magneto;

/*!
* \def KFM_IDDCM Number of observation variables for integrated difference DCM-only updates
*/
#define KFM_MAGNETO 6

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
/* System initialization                                                */
/************************************************************************/

/*!
* \brief Initializes the state matrix of a specific filter based on its state
*/
HOT NONNULL
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
COLD
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
        const fix16_t q_axis = F16(0);
        matrix_set(Q, 0, 0, q_axis);
        matrix_set(Q, 1, 1, q_axis);
        matrix_set(Q, 2, 2, q_axis);

        // gyro process noise
        const fix16_t q_gyro = F16(1);
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
    const fix16_t initial_r_axis = F16(0.02);
    const fix16_t initial_r_gyro = F16(0.001);
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
* \brief Initializes the sensor fusion mechanism.
*/
void fusion_initialize()
{
    initialize_system();
    initialize_observation_accel();
    initialize_observation_magneto();
}

/************************************************************************/
/* State calculation helpers                                            */
/************************************************************************/

/*!
* \brief Sanitizes the state variables
*/
HOT
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
*/
HOT CONST
STATIC_INLINE int32_t fix16_sign(const fix16_t value)
{
    return value >= 0 ? 1 : -1;
}

/*!
* \brief Fetches the values without any modification
* \param[out] roll The roll angle in radians.
* \param[out] pitch The pitch (elevation) angle in radians.
*/
HOT NONNULL LEAF
void calculate_roll_pitch(register fix16_t *RESTRICT const roll, register fix16_t *RESTRICT const pitch)
{
    const mf16 *const x = kalman_get_state_vector_uc(&kf_attitude);

    // fetch axes
    const fix16_t c31 = x->data[0][0];
    const fix16_t c32 = x->data[1][0];
    const fix16_t c33 = x->data[2][0];

    // calculate roll
    const fix16_t c1sq     = fix16_sq(c31);
    const fix16_t c3sq     = fix16_sq(c33);
    const fix16_t c1c3sq   = fix16_add(c1sq, c3sq);
    const fix16_t c1c3norm = fix16_sqrt(c1c3sq);
    *roll = fix16_atan2(c32, fix16_sign(c33)*c1c3norm);

    // calculate pitch
    *pitch = -fix16_asin(c31);
}

/*!
* \brief Fetches the values without any modification
* \param[in] roll The roll angle in radians.
* \param[in] pitch The pitch (elevation) angle in radians.
* \param[out] yaw The yaw angle in radians
*/
HOT NONNULL LEAF
void calculate_yaw(register const fix16_t roll, register const fix16_t pitch, register fix16_t *RESTRICT const yaw)
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

/************************************************************************/
/* State prediction                                                     */
/************************************************************************/

/*!
* \brief Performs a prediction of the current Euler angles based on the time difference to the previous prediction/update iteration.
* \param[in] deltaT The time difference in seconds to the last prediction or observation update call.
*/
HOT
void fusion_predict(register const fix16_t deltaT)
{
    mf16 *const x2 = kalman_get_state_vector_uc(&kf_orientation);
    mf16 *const x3 = kalman_get_state_vector_uc(&kf_attitude);
    
    // fetch old state for integration
    const fix16_t c31 = x3->data[0][0];
    const fix16_t c32 = x3->data[1][0];
    const fix16_t c33 = x3->data[2][0];

    const fix16_t c21 = x2->data[0][0];
    const fix16_t c22 = x2->data[1][0];
    const fix16_t c23 = x2->data[2][0];

    // predict.
#if 0
    kalman_predict_tuned_uc(&kf_attitude, lambda);
    kalman_predict_tuned_uc(&kf_orientation, lambda);
#else
    kalman_predict_uc(&kf_attitude);
    kalman_predict_uc(&kf_orientation);
#endif

    // integrate state
    x3->data[0][0] = fix16_add(c31, fix16_mul(x3->data[0][0], deltaT));
    x3->data[1][0] = fix16_add(c32, fix16_mul(x3->data[1][0], deltaT));
    x3->data[2][0] = fix16_add(c33, fix16_mul(x3->data[2][0], deltaT));

    x2->data[0][0] = fix16_add(c21, fix16_mul(x2->data[0][0], deltaT));
    x2->data[1][0] = fix16_add(c22, fix16_mul(x2->data[1][0], deltaT));
    x2->data[2][0] = fix16_add(c23, fix16_mul(x2->data[2][0], deltaT));

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
    m_accelerometer.x = *ax;
    m_accelerometer.y = *ay;
    m_accelerometer.z = *az;
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
* \brief Updates the current prediction with accelerometer data
*/
HOT
void fusion_update_attitude(register const fix16_t deltaT)
{
    /************************************************************************/
    /* Prepare measurement                                                  */
    /************************************************************************/
    {
        mf16 *const z = &kfm_accel.z;

        matrix_set(z, 0, 0, -m_accelerometer.x);
        matrix_set(z, 1, 0, -m_accelerometer.y);
        matrix_set(z, 2, 0, -m_accelerometer.z);

        matrix_set(z, 3, 0, fix16_deg_to_rad(m_gyroscope.x));
        matrix_set(z, 4, 0, fix16_deg_to_rad(m_gyroscope.y));
        matrix_set(z, 5, 0, fix16_deg_to_rad(m_gyroscope.z));
    }

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
* \brief Updates the current prediction with magnetometer data
*/
HOT
void fusion_update_orientation(register const fix16_t deltaT)
{
    /************************************************************************/
    /* Calculate metrics required for update                                */
    /************************************************************************/

    fix16_t roll, pitch;
    calculate_roll_pitch(&roll, &pitch);

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
    register const fix16_t pnorm = fix16_sqrt(fix16_add(fix16_sq(Xh), 
                                                        fix16_sq(Yh)
                                                        )
                                             );

    // calculate yaw sine and cosine from components
    register const fix16_t sin_yaw = fix16_div(Yh, pnorm);
    register const fix16_t cos_yaw = fix16_div(Xh, pnorm);

    /************************************************************************/
    /* Calculate tilt-compensated readings                                  */
    /************************************************************************/

    const fix16_t mx = fix16_mul(cos_pitch, sin_yaw);
    const fix16_t my = fix16_add(fix16_mul(cos_roll, cos_yaw),
                                 fix16_mul(sin_roll_sin_pitch, sin_yaw)
                       );
    const fix16_t mz = fix16_add(fix16_mul(-sin_roll, cos_yaw),
                                 fix16_mul(cos_roll_sin_pitch, sin_yaw)
                       );

    /************************************************************************/
    /* Prepare measurement                                                  */
    /************************************************************************/
    {
        mf16 *const z = &kfm_magneto.z;

        matrix_set(z, 0, 0, mx);
        matrix_set(z, 1, 0, my);
        matrix_set(z, 2, 0, mz);

        matrix_set(z, 3, 0, fix16_deg_to_rad(m_gyroscope.x));
        matrix_set(z, 4, 0, fix16_deg_to_rad(m_gyroscope.y));
        matrix_set(z, 5, 0, fix16_deg_to_rad(m_gyroscope.z));
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
    if (true == m_have_gyroscope)
    {
        // perform roll and pitch updates
        if (true == m_have_accelerometer)
        {
            fusion_update_attitude(deltaT);
            m_have_accelerometer = false;
        }

        // perform yaw updates
        if (true == m_have_magnetometer)
        {
            fusion_update_orientation(deltaT);
            m_have_magnetometer = false;
        }

        // only rotational updates may be performed.
        if ((false == m_have_accelerometer) && (false == m_have_magnetometer))
        {
            // nothing for now.
        }
    }
    else
    {
        // nothing for now.
    }

    // For now, let's assume gyro readings are always valid
    // m_have_gyroscope = false;
}
