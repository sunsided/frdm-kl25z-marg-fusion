/*
* output_mode.h
*
*  Created on: Jan 30, 2014
*      Author: Markus
*/

#ifndef _OUTPUT_MODE_H_
#define _OUTPUT_MODE_H_

/*!
* \brief Defines the output modes
*/
typedef enum {
    SENSORS_RAW = 0,        //!< Raw sensor information
    RPY = 42,               //!< Derived roll/pitch/yaw angles
    QUATERNION = 43,        //!< Fused quaternion only
    QUATERNION_RPY = 44,    //!< Fused quaternion and derived roll/pitch/yaw angles

} output_mode_t;

#endif