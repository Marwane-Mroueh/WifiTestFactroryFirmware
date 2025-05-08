#include <ch.h>
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <motors.h>

#include "main.h"
#include "grid.h"
#include "ir_sensors.h"
// #include "pi_regulator.h"

#define GRID_LENGTH 40
#define GRID_ANGLE 6
#define ROTATION_CORRECTION_COEF 1.076

void check_pause_request(void);

thread_reference_t grid_ref = NULL;

static THD_WORKING_AREA(waGridMove, 256);
static THD_FUNCTION(GridMove, arg)
{
    (void)arg;
    while (1)
    {
        if (!get_object_found() && !get_object_close() /*&& get_mode_one_on()*/)
        {
            for (int i = 0; i < GRID_LENGTH; i++) // straight line
            {
                check_pause_request();
                if (/*get_mode_one_on()*/ true && !get_object_found() && !get_object_close())
                {
                    right_motor_set_speed(GRID_MOTOR_SPEED);
                    left_motor_set_speed(GRID_MOTOR_SPEED);
                }
                else
                {
                    right_motor_set_speed(0);
                    left_motor_set_speed(0);
                }
                chThdSleepMilliseconds(100);
            }
            for (int i = 0; i < GRID_ANGLE; i++) // 90 degree turn
            {
                check_pause_request();
                if (/*get_mode_one_on()*/ true && !get_object_found() && !get_object_close())
                {
                    right_motor_set_speed(GRID_MOTOR_SPEED * ROTATION_CORRECTION_COEF);
                    left_motor_set_speed(-GRID_MOTOR_SPEED * ROTATION_CORRECTION_COEF);
                }
                else
                {
                    right_motor_set_speed(0);
                    left_motor_set_speed(0);
                }
                chThdSleepMilliseconds(100);
            }
        }
    }
}

void grid_move_start(void)
{
    chThdCreateStatic(waGridMove, sizeof(waGridMove), NORMALPRIO, GridMove, NULL);
}

void check_pause_request()
{
    if (get_pause_request())
    {
        right_motor_set_speed(0);
        left_motor_set_speed(0);
        chSysLock();              // Section critique
        chThdSuspendS(&grid_ref); // suspend self, grid_ref ← this thread
        chSysUnlock();
        grid_ref = NULL;
    }
    return;
}