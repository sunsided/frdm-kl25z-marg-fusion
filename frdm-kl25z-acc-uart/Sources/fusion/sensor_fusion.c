#include "fusion/sensor_fusion.h"

/*!
* \brief Fetches the last predicted values without updating.
* \param[out] roll The roll angle in radians.
* \param[out] pitch The pitch (elevation) angle in radians.
* \param[out] yaw The yaw (heading, azimuth) angle in radians.
*/
void fusion_fetch_prediction(register fix16_t *const roll, register fix16_t *const pitch, register fix16_t *const yaw)
{

}

/*!
* \brief Performs a prediction of the current Euler angles based on the time difference to the previous prediction/update iteration.
* \param[in] deltaT The time difference in seconds to the last prediction or observation update call.
* \param[out] roll The roll angle in radians.
* \param[out] pitch The pitch (elevation) angle in radians.
* \param[out] yaw The yaw (heading, azimuth) angle in radians.
*/
void fusion_predict(register const fix16_t deltaT, register fix16_t *const roll, register fix16_t *const pitch, register fix16_t *const yaw)
{

}

/*!
* \brief Updates the current prediction with accelerometer measurements
* \param[in] deltaT The time difference in seconds to the previous prediction/update iteration.
* \param[in] ax The x-axis accelerometer value.
* \param[in] ay The y-axis accelerometer value.
* \param[in] az The z-axis accelerometer value.
*/
void fusion_update_with_accelerometer(register const fix16_t deltaT, register const fix16_t *const ax, register const fix16_t *const ay, register const fix16_t *const az)
{

}

/*!
* \brief Updates the current prediction with gyroscope measurements
* \param[in] deltaT The time difference in seconds to the previous prediction/update iteration.
* \param[in] gx The x-axis gyroscope value.
* \param[in] gy The y-axis gyroscope value.
* \param[in] gz The z-axis gyroscope value.
*/
void fusion_update_with_gyroscope(register const fix16_t deltaT, register const fix16_t *const gx, register const fix16_t *const gy, register const fix16_t *const gz)
{

}

/*!
* \brief Updates the current prediction with magnetometer measurements
* \param[in] deltaT The time difference in seconds to the previous prediction/update iteration.
* \param[in] mx The x-axis magnetometer value.
* \param[in] my The y-axis magnetometer value.
* \param[in] mz The z-axis magnetometer value.
*/
void fusion_update_with_magnetometer(register const fix16_t deltaT, register const fix16_t *const mx, register const fix16_t *const my, register const fix16_t *const mz)
{

}