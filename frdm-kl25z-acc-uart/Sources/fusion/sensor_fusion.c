#include <assert.h>
#include <stdbool.h>

#include "fixmath.h"
#include "fixkalman.h"
#include "fixvector3d.h"
#include "fixmatrix.h"

#if !defined(FIXMATRIX_MAX_SIZE) || (FIXMATRIX_MAX_SIZE < 12)
#error FIXMATRIX_MAX_SIZE must be defined to value greater or equal 12.
#endif

#include "fusion/sensor_dcm.h"
#include "fusion/sensor_fusion.h"

/*!
* \brief The Kalman filter instance used to predict the orientation.
*/
static kalman16_uc_t kf_orientation;

/*!
* \def KF_STATES Number of states
*/
#define KF_STATES 12

/*!
* \brief The Kalman filter observation instance used to update the prediction with integrated difference DCM-only data
*
* Measured values are: phi, psi, theta from integrated difference DCM
*/
static kalman16_observation_t kfm_iddcm;

/*!
* \def KFM_IDDCM Number of observation variables for integrated difference DCM-only updates
*/
#define KFM_IDDCM 3

/*!
* \brief The Kalman filter observation instance used to update the prediction with gyroscope-only data.
*
* Measured values are: phi, psi, theta from integrated gyro, as well as omega_phi, omega_psi, omega_theta from direct gyro
*/
static kalman16_observation_t kfm_gyro;

/*!
* \def KFM_GYRO Number of observation variables for gyroscope-only updates
*/
#define KFM_GYRO 6

/*!
* \brief The Kalman filter observation instance used to update the prediction with gyroscope and iddcm data.
*
* Measured values are: phi, psi, theta from integrated difference DCM, phi, psi, theta from integrated gyro, as well as omega_phi, omega_psi, omega_theta from direct gyro
*/
static kalman16_observation_t kfm_gyro_iddcm;

/*!
* \def KFM_GYRO Number of observation variables for gyroscope-only updates
*/
#define KFM_GYROIDDCM 9

/*!
* \brief Lambda parameter for certainty tuning
*/
static fix16_t lambda = F16(0.98);

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

/*!
* \brief yaw/pitch/roll from gyroscope integration
*/
static v3d state_ypr_from_gyro = { 0, 0, 0 };

/*!
* \brief yaw/pitch/roll from difference DCM integration
*/
static v3d state_ypr_from_iddcm = { 0, 0, 0 };

/*!
* \brief DCM from previous loop
*/
static mf16 state_previous_dcm = { 3, 3, 0, 0 };


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

/*!
* \brief Initialization of the filter
*/
COLD
static void initialize_system()
{
    kalman16_uc_t *const kf = &kf_orientation;
    kalman_filter_initialize_uc(kf, KF_STATES);

    /************************************************************************/
    /* Prepare initial state estimation                                     */
    /************************************************************************/
    // intentionally left blank, because the initialize function already set the vector to zero

    /************************************************************************/
    /* Set state transition model                                           */
    /************************************************************************/
    {
        mf16 *const A = &kf->A;

        // discrete model
        // angle_next = (1) * angle_current 
        //            + (dT * angular_velocity) 
        //            + (0.5*dT^2 * angular_acceleration)


        // integrated difference DCM: phi
        matrix_set(A, 0, 0, F16_ONE);
        matrix_set(A, 0, 6, dT(initial_dT));
        matrix_set(A, 0, 9, half_dT_square(initial_dT));

        // integrated difference DCM: psi
        matrix_set(A, 1, 1, F16_ONE);
        matrix_set(A, 1, 7, dT(initial_dT));
        matrix_set(A, 1, 10, half_dT_square(initial_dT));

        // integrated difference DCM: theta
        matrix_set(A, 2, 2, F16_ONE);
        matrix_set(A, 2, 8, dT(initial_dT));
        matrix_set(A, 2, 11, half_dT_square(initial_dT));

        // TODO: ideally, these should not be tracked separately, but together using the measurement update

        // integrated gyro: phi
        matrix_set(A, 3, 3, F16_ONE);
        matrix_set(A, 3, 6, dT(initial_dT));
        matrix_set(A, 3, 9, half_dT_square(initial_dT));

        // integrated gyro: psi
        matrix_set(A, 4, 4, F16_ONE);
        matrix_set(A, 4, 7, dT(initial_dT));
        matrix_set(A, 4, 10, half_dT_square(initial_dT));

        // integrated gyro: theta
        matrix_set(A, 5, 5, F16_ONE);
        matrix_set(A, 5, 8, dT(initial_dT));
        matrix_set(A, 5, 11, half_dT_square(initial_dT));


        // discrete model
        // angular_velocity_next = (1) * angular_velocity_current 
        //                       + (dT * angular_acceleration) 

        // gyro: omega_phi
        matrix_set(A, 6, 6, F16_ONE);
        matrix_set(A, 6, 9, dT(initial_dT));

        // gyro: omega_psi
        matrix_set(A, 7, 7, F16_ONE);
        matrix_set(A, 7, 10, dT(initial_dT));

        // gyro: omega_theta
        matrix_set(A, 8, 8, F16_ONE);
        matrix_set(A, 8, 11, dT(initial_dT));


        // discrete model
        // angular_acceleration_next = (1) * angular_acceleration_current 

        // gyro: alpha_phi
        matrix_set(A, 9, 9, F16_ONE);

        // gyro: alpha_psi
        matrix_set(A, 10, 10, F16_ONE);

        // gyro: alpha_theta
        matrix_set(A, 11, 11, F16_ONE);
    }

    /************************************************************************/
    /* Set state variances                                                  */
    /************************************************************************/
    {
        mf16 *const P = &kf->P;

        // phi, psi and theta variances from integrated difference DCM
        matrix_set(P, 0, 0, F16(1.9));
        matrix_set(P, 1, 1, F16(1.9));
        matrix_set(P, 2, 2, F16(1.9));

        // phi, psi and theta variances from integrated gyro
        matrix_set(P, 3, 3, F16(1));
        matrix_set(P, 4, 4, F16(1));
        matrix_set(P, 5, 5, F16(1));

        // omega_phi, omega_psi and omega_theta variances from direct gyro
        matrix_set(P, 6, 6, F16(1));
        matrix_set(P, 7, 7, F16(1));
        matrix_set(P, 8, 8, F16(1));

        // alpha_phi, alpha_psi and alpha_theta variances from direct gyro
        matrix_set(P, 9, 9, F16(1));
        matrix_set(P, 10, 10, F16(1));
        matrix_set(P, 11, 11, F16(1));
    }

    /************************************************************************/
    /* Set state covariances                                                */
    /************************************************************************/
    {
        mf16 *const P = &kf->P;

        // set phi, psi and theta covariances for integrated difference DCM and integrated gyro
        matrix_set_symmetric(P, 0, 3, 1); // cov(idDCM_x, igyro_x)
        matrix_set_symmetric(P, 1, 4, 1); // cov(idDCM_y, igyro_y)
        matrix_set_symmetric(P, 2, 5, 1); // cov(idDCM_z, igyro_z)
    }

    /************************************************************************/
    /* Set system process noise                                             */
    /************************************************************************/
    // intentionally left blank, because the initialize function already set the matrix to zero
    // zero is a rather bad estimate though, so fill this or use appropriate lambda tuning.
}

/*!
* \brief Initialization of integrated difference DCM-only updates
*/
COLD
static void initialize_observation_iddcm()
{
    kalman16_observation_t *const kfm = &kfm_iddcm;
    kalman_observation_initialize(kfm, KF_STATES, KFM_IDDCM);

    /************************************************************************/
    /* Set observation model                                                */
    /************************************************************************/
    {
        mf16 *const H = &kfm->H;

        matrix_set(H, 0, 0, F16_ONE);
        matrix_set(H, 1, 1, F16_ONE);
        matrix_set(H, 2, 2, F16_ONE);
    }

    /************************************************************************/
    /* Set observation process noise covariance                             */
    /************************************************************************/
    {
        mf16 *const R = &kfm->R;

        matrix_set(R, 0, 0, F16(10.087));
        matrix_set(R, 1, 1, F16(6.1297));
        matrix_set(R, 2, 2, F16(8.8969));
    }
}

/*!
* \brief Initialization of gyroscope and iddcm updates
*/
COLD
static void initialize_observation_gyro_iddcm()
{
    kalman16_observation_t *const kfm = &kfm_gyro_iddcm;
    kalman_observation_initialize(kfm, KF_STATES, KFM_GYROIDDCM);

    /************************************************************************/
    /* Set observation model                                                */
    /************************************************************************/
    {
        mf16 *const H = &kfm->H;
        mf16_fill_diagonal(H, F16_ONE);
    }

    /************************************************************************/
    /* Set observation process noise covariance                             */
    /************************************************************************/
    {
        mf16 *const R = &kfm->R;

        matrix_set(R, 0, 0, F16(10.087));
        matrix_set(R, 1, 1, F16(6.1297));
        matrix_set(R, 2, 2, F16(8.8969));

        matrix_set(R, 3, 3, F16(3.7462));
        matrix_set(R, 4, 4, F16(3.265));
        matrix_set(R, 5, 5, F16(3.9315));

        matrix_set(R, 6, 6, F16_ONE);
        matrix_set(R, 7, 7, F16_ONE);
        matrix_set(R, 8, 8, F16_ONE);

        matrix_set_symmetric(R, 0, 3, F16(0.1));
        matrix_set_symmetric(R, 0, 4, F16(0.67));
        matrix_set_symmetric(R, 0, 5, F16(1.24));

        matrix_set_symmetric(R, 1, 3, F16(0.18));
        matrix_set_symmetric(R, 1, 4, F16(0.065));
        matrix_set_symmetric(R, 1, 5, F16(0.43));

        matrix_set_symmetric(R, 2, 3, F16(0.26));
        matrix_set_symmetric(R, 2, 4, F16(0.05));
        matrix_set_symmetric(R, 2, 5, F16(1.02));
    }
}

/*!
* \brief Initialization of gyroscope-only updates
*/
COLD
static void initialize_observation_gyroscope()
{
    kalman16_observation_t *const kfm = &kfm_gyro;
    kalman_observation_initialize(kfm, KF_STATES, KFM_GYRO);

    /************************************************************************/
    /* Set observation model                                                */
    /************************************************************************/
    {
        mf16 *const H = &kfm->H;
        mf16_fill_diagonal(H, F16_ONE);
    }

    /************************************************************************/
    /* Set observation process noise covariance                             */
    /************************************************************************/
    {
        mf16 *const R = &kfm->R;

        matrix_set(R, 3, 3, F16(3.7462));
        matrix_set(R, 4, 4, F16(3.265));
        matrix_set(R, 5, 5, F16(3.9315));

        matrix_set(R, 6, 6, F16_ONE);
        matrix_set(R, 7, 7, F16_ONE);
        matrix_set(R, 8, 8, F16_ONE);
    }
}

/*!
* \brief Initializes the sensor fusion mechanism.
*/
void fusion_initialize()
{
    initialize_system();
    initialize_observation_iddcm();
    initialize_observation_gyroscope();
    initialize_observation_gyro_iddcm();
}

/*!
* \brief Corrects an angle to be within +/- 180°
*/
HOT
STATIC_INLINE void fusion_clampangle(fix16_t *angle)
{
    while (*angle > F16(180)) {
        *angle = fix16_sub(*angle, F16(360));
    }

    while (*angle < F16(-180)) {
        *angle = fix16_add(*angle, F16(360));
    }
}

/*!
* \brief Sanitizes the state variables
*/
HOT
void fusion_sanitize_state()
{
    mf16 *const x = kalman_get_state_vector_uc(&kf_orientation);

    // iddcm angles
    fusion_clampangle(&x->data[0][0]);
    fusion_clampangle(&x->data[1][1]);
    fusion_clampangle(&x->data[2][2]);

    // integrated gyro angles
    fusion_clampangle(&x->data[3][3]);
    fusion_clampangle(&x->data[4][4]);
    fusion_clampangle(&x->data[5][5]);
}

/*!
* \brief Calculate the weighted sum of two values based on their variance
* \param[in] a The first value
* \param[in] var_a The first value's variance
* \param[in] b The second value
* \param[in] var_b The second value's variance
* \return The sum
*/
HOT CONST
STATIC_INLINE fix16_t variance_weighted_sum(register const fix16_t a, register const fix16_t var_a, register const fix16_t b, register const fix16_t var_b)
{
    // create variance weighting factor
    const fix16_t variance_sum = fix16_add(var_a, var_b);
    const fix16_t inv_variance_sum = fix16_div(F16_ONE, variance_sum);

    // create weighting factors based on variances
    // Note that the variance of B defines the weight for A. Lower variances
    // mean better results, so if B's variance is higher, it gets a lower weight.
    const fix16_t weight_a = fix16_mul(var_b, inv_variance_sum);
    const fix16_t weight_b = fix16_mul(var_a, inv_variance_sum);

    // build weighted sum
    return fix16_add(fix16_mul(a, weight_a), fix16_mul(b, weight_b));
}

/*!
* \brief Fetches the values without any modification
* \param[out] roll The roll angle in degree.
* \param[out] pitch The pitch (elevation) angle in degree.
* \param[out] yaw The yaw (heading, azimuth) angle in degree.
*/
void fetch_values(register fix16_t *const roll, register fix16_t *const pitch, register fix16_t *const yaw)
{
    // fetch state vector and covariance reference
    const mf16 *const x = kalman_get_state_vector_uc(&kf_orientation);
    const mf16 *const P = kalman_get_system_covariance_uc(&kf_orientation);

    // TODO: verify that variance_weighted_sum works as expected

    // fetch yaw
    *roll = variance_weighted_sum(x->data[0][0], P->data[0][0], x->data[3][0], P->data[3][3]);

    // fetch pitch
    *pitch = variance_weighted_sum(x->data[1][0], P->data[1][1], x->data[4][0], P->data[4][4]);

    // fetch yaw
    *yaw = variance_weighted_sum(x->data[2][0], P->data[2][2], x->data[5][0], P->data[5][5]);
}

/*!
* \brief Performs a prediction of the current Euler angles based on the time difference to the previous prediction/update iteration.
* \param[in] deltaT The time difference in seconds to the last prediction or observation update call.
*/
void fusion_predict(register const fix16_t deltaT)
{
    // set state transition based on deltaT
    {
        mf16 *const A = &kf_orientation.A;
        const fix16_t hdt2q = half_dT_square(deltaT);

        // discrete model
        // angle_next = (1) * angle_current 
        //            + (dT * angular_velocity) 
        //            + (0.5*dT^2 * angular_acceleration)


        // integrated difference DCM: phi
        matrix_set(A, 0, 6, deltaT);
        matrix_set(A, 0, 9, hdt2q);

        // integrated difference DCM: psi
        matrix_set(A, 1, 7, deltaT);
        matrix_set(A, 1, 10, hdt2q);

        // integrated difference DCM: theta
        matrix_set(A, 2, 8, deltaT);
        matrix_set(A, 2, 11, hdt2q);

        // TODO: ideally, these should not be tracked separately, but together using the measurement update

        // integrated gyro: phi
        matrix_set(A, 3, 6, deltaT);
        matrix_set(A, 3, 9, hdt2q);

        // integrated gyro: psi
        matrix_set(A, 4, 7, deltaT);
        matrix_set(A, 4, 10, hdt2q);

        // integrated gyro: theta
        matrix_set(A, 5, 8, deltaT);
        matrix_set(A, 5, 11, hdt2q);


        // discrete model
        // angular_velocity_next = (1) * angular_velocity_current 
        //                       + (dT * angular_acceleration) 
        
        // gyro: omega_phi
        matrix_set(A, 6, 9, deltaT); 

        // gyro: omega_psi
        matrix_set(A, 7, 10, deltaT);

        // gyro: omega_theta
        matrix_set(A, 8, 11, deltaT);
    }

    // predict.
    kalman_predict_tuned_uc(&kf_orientation, lambda);

    // sanitize state data
    fusion_sanitize_state();
}

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

/*!
* \brief Updates the current prediction with gyroscope measurements
* \param[in] gx The x-axis gyroscope value.
* \param[in] gy The y-axis gyroscope value.
* \param[in] gz The z-axis gyroscope value.
*/
HOT
void fusion_update_using_gyro_only(register const fix16_t deltaT)
{
    /************************************************************************/
    /* Prepare gyroscope data                                               */
    /************************************************************************/

    v3d scaled_velocity;
    v3d_mul_s(&scaled_velocity, &m_gyroscope, deltaT);

    // angle += velocity * dT
    v3d_add(&state_ypr_from_gyro, &state_ypr_from_gyro, &scaled_velocity);

    /************************************************************************/
    /* Prepare measurement                                                  */
    /************************************************************************/
    {
        mf16 *const z = &kfm_gyro.z;

        matrix_set(z, 0, 0, state_ypr_from_gyro.x);
        matrix_set(z, 1, 0, state_ypr_from_gyro.y);
        matrix_set(z, 2, 0, state_ypr_from_gyro.z);

        matrix_set(z, 3, 0, m_gyroscope.x);
        matrix_set(z, 4, 0, m_gyroscope.y);
        matrix_set(z, 5, 0, m_gyroscope.z);
    }

    /************************************************************************/
    /* Perform Kalman update                                                */
    /************************************************************************/

    kalman_correct_uc(&kf_orientation, &kfm_gyro);
}

/*!
* \brief Updates the current prediction with magnetometer and/or accelerometer measurements
* \param[in] gx The x-axis gyroscope value.
* \param[in] gy The y-axis gyroscope value.
* \param[in] gz The z-axis gyroscope value.
*/
HOT
void fusion_update_using_iddcm_only(register const fix16_t deltaT)
{
    /************************************************************************/
    /* Prepare DCM data                                                     */
    /************************************************************************/

    // fetch current DCM
    mf16 dcm = { 3, 3, 0, 0 };
    sensor_dcm(&dcm, &m_accelerometer, &m_magnetometer);

    // build difference DCM
    fix16_t om_yaw, om_pitch, om_roll;
    sensor_ddcm(&dcm, &state_previous_dcm, &om_yaw, &om_pitch, &om_roll);

    // save current DCM --> previous DCM
    for (uint_fast8_t r = 0; r < 3; ++r)
    {
        for (uint_fast8_t c = 0; c < 3; ++c)
        {
            state_previous_dcm.data[r][c] = dcm.data[r][c];
        }
    }

    // angle += velocity * dT
    // note that dT is already contained in the dDCM.
    state_ypr_from_iddcm.x = fix16_add(state_ypr_from_iddcm.x, om_yaw);
    state_ypr_from_iddcm.y = fix16_add(state_ypr_from_iddcm.y, om_pitch);
    state_ypr_from_iddcm.z = fix16_add(state_ypr_from_iddcm.z, om_roll);

    /************************************************************************/
    /* Prepare measurement                                                  */
    /************************************************************************/
    {
        mf16 *const z = &kfm_iddcm.z;

        matrix_set(z, 0, 0, state_ypr_from_iddcm.x);
        matrix_set(z, 1, 0, state_ypr_from_iddcm.y);
        matrix_set(z, 2, 0, state_ypr_from_iddcm.z);
    }

    /************************************************************************/
    /* Perform Kalman update                                                */
    /************************************************************************/

    kalman_correct_uc(&kf_orientation, &kfm_iddcm);
}

/*!
* \brief Updates the current prediction with gyroscope and iddcm measurements
* \param[in] gx The x-axis gyroscope value.
* \param[in] gy The y-axis gyroscope value.
* \param[in] gz The z-axis gyroscope value.
*/
HOT
void fusion_update_using_gyro_and_iddcm(register const fix16_t deltaT)
{
    /************************************************************************/
    /* Prepare gyroscope data                                               */
    /************************************************************************/

    v3d scaled_velocity;
    v3d_mul_s(&scaled_velocity, &m_gyroscope, deltaT);

    // angle += velocity * dT
    v3d_add(&state_ypr_from_gyro, &state_ypr_from_gyro, &scaled_velocity);

    /************************************************************************/
    /* Prepare DCM data                                                     */
    /************************************************************************/

    // fetch current DCM
    mf16 dcm = { 3, 3, 0, 0 };
    sensor_dcm(&dcm, &m_accelerometer, &m_magnetometer);

    // build difference DCM
    fix16_t om_yaw, om_pitch, om_roll;
    sensor_ddcm(&dcm, &state_previous_dcm, &om_yaw, &om_pitch, &om_roll);

    // save current DCM --> previous DCM
    for (uint_fast8_t r = 0; r < 3; ++r)
    {
        for (uint_fast8_t c = 0; c < 3; ++c)
        {
            state_previous_dcm.data[r][c] = dcm.data[r][c];
        }
    }

    // angle += velocity * dT
    // note that dT is already contained in the dDCM.
    state_ypr_from_iddcm.x = fix16_add(state_ypr_from_iddcm.x, om_yaw);
    state_ypr_from_iddcm.y = fix16_add(state_ypr_from_iddcm.y, om_pitch);
    state_ypr_from_iddcm.z = fix16_add(state_ypr_from_iddcm.z, om_roll);

    /************************************************************************/
    /* Prepare measurement                                                  */
    /************************************************************************/
    {
        mf16 *const z = &kfm_gyro_iddcm.z;

        matrix_set(z, 0, 0, state_ypr_from_iddcm.x);
        matrix_set(z, 1, 0, state_ypr_from_iddcm.y);
        matrix_set(z, 2, 0, state_ypr_from_iddcm.z);

        matrix_set(z, 3, 0, state_ypr_from_gyro.x);
        matrix_set(z, 4, 0, state_ypr_from_gyro.y);
        matrix_set(z, 5, 0, state_ypr_from_gyro.z);

        matrix_set(z, 6, 0, m_gyroscope.x);
        matrix_set(z, 7, 0, m_gyroscope.y);
        matrix_set(z, 8, 0, m_gyroscope.z);
    }

    /************************************************************************/
    /* Perform Kalman update                                                */
    /************************************************************************/

    kalman_correct_uc(&kf_orientation, &kfm_gyro_iddcm);
}

/*!
* \brief Updates the current prediction with the set measurements.
* \param[in] deltaT The time difference in seconds to the last prediction or observation update call.
*/
void fusion_update(register const fix16_t deltaT)
{
    uint_fast8_t strategy = 0;

    // integrate gyroscope if required
    if (true == m_have_gyroscope)
    {
        // in this case, we need to integrate the gyro and either update
        // using kfm_gyro_iddcm or kfm_gyro, depending on the other values
        strategy = 2; // 0b10;
    }

    // integrate difference DCM if required
    if (true == m_have_accelerometer || true == m_have_magnetometer)
    {
        // in this case, we need to build the difference DCM either update
        // using kfm_iddcm or kfm_gyro_iddcm, depending on m_have_gyroscope
        strategy |= 1; // 0b01;
    }

    // select strategy
    switch (strategy)
    {
        case 2: // 0b10 -- if (use_gyro && !use_iddcm)      
        {
            // this is the most probable case: gyro observation is available, 
            // but accelerometer and magnetometer observations are missing
            fusion_update_using_gyro_only(deltaT);

            // sanitize state data
            fusion_sanitize_state();
            break;
        }
        case 3: // 0b11 -- if (use_gyro && use_iddcm)
        {
            // this is the second-most probable case: gyro observation is available, 
            // and either accelerometer or magnetometer observations is available
            fusion_update_using_gyro_and_iddcm(deltaT);

            // sanitize state data
            fusion_sanitize_state();
            break;
        }
        case 1: // 0b01 -- (!use_gyro && use_iddcm)
        {
            // this is the least probable case: and either accelerometer or 
            // magnetometer observations is available, but gyro observation is missing
            fusion_update_using_iddcm_only(deltaT);

            // sanitize state data
            fusion_sanitize_state();
            break;
        }
        default: // 0b00 -- (!use_gyro && !use_iddcm)
        {
            // in any other case, do nothing.
            break;
        }
    }
   

    m_have_accelerometer = false;
    m_have_gyroscope = false;
    m_have_magnetometer = false;
}
