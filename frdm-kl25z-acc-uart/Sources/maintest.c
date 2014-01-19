#include "ARMCM0plus.h"
#include "derivative.h" /* include peripheral declarations */
#include "bme.h"

#include "cpu/clock.h"
#include "cpu/systick.h"

#include "fixvector3d.h"
#include "fusion/sensor_calibration.h"

#include "fusion/sensor_fusion.h"

void main()
{
    /* initialize the core clock and the systick timer */
    InitClock();
    InitSysTick();
    
    fusion_initialize();

    v3d a = { F16(-0.99976), F16(0.026611), F16(-0.14551) };

    mpu6050_calibrate_accelerometer(&a.x, &a.y, &a.z);
    
    float x = fix16_to_float(a.x);
    float y = fix16_to_float(a.y);
    float z = fix16_to_float(a.z);
    
    for (;;) {}
}
