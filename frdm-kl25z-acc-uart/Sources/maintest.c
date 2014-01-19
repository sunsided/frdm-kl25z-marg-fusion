#include "ARMCM0plus.h"
#include "derivative.h" /* include peripheral declarations */
#include "bme.h"

#include "cpu/clock.h"
#include "cpu/systick.h"

#include "fusion/sensor_dcm.h"

void main()
{
    /* initialize the core clock and the systick timer */
    InitClock();
    InitSysTick();
    
    fix16_t roll = 0;
    fix16_t pitch = 0;
    fix16_t yaw = 0;

    mf16 old_dcm = { 3, 3, 0,
    {
        { F16(0.11206), F16(-0.25495), F16(-0.96044) },
        { F16(0.037374), F16(0.96693), F16(-0.25231) },
        { F16(0.993), F16(-0.0076226), F16(0.11788) },
    }
    };

    mf16 new_dcm = { 3, 3, 0,
    {
        { F16(0.11165), F16(-0.26533), F16(-0.95767) },
        { F16(0.038577), F16(0.96413), F16(-0.26263) },
        { F16(0.993), F16(-0.0076226), F16(0.11788) },
    }
    };

    sensor_ddcm(&new_dcm, &old_dcm, &roll, &pitch, &yaw);

    float om_rollf = fix16_to_float(fix16_rad_to_deg(roll));
    float om_pitchf = fix16_to_float(fix16_rad_to_deg(pitch));
    float om_yawf = fix16_to_float(fix16_rad_to_deg(yaw));
    
    for (;;) {}
}
