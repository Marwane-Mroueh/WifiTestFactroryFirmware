#include <ch.h>
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <motors.h>

#include "main.h"
#include "pi_regulator.h"
#include "process_image.h"
#include "ir_sensors.h"
#include "spi_comm.h"

static int goal_distance = DEFAULT_GOAL_DISTANCE;

static bool enabled_motors = false;
static bool mode_one_on = true;
static bool attack_true = false;

float get_goal_distance(void)
{
    return goal_distance;
}
void set_goal_distance(float a)
{
    goal_distance = a;
    return;
}
bool get_mode_one_on(void)
{
    return mode_one_on;
}
void set_mode_one_on(bool a)
{
    mode_one_on = a;
    return;
}
bool get_attack_true(void)
{
    return attack_true;
}
void set_attack_true(bool a)
{
    attack_true = a;
    return;
}

// simple PI regulator implementation
int16_t pi_regulator(float distance, float goal)
{

    float error = 0;
    float speed = 0;

    static float sum_error = 0;

    error = distance - goal;

    // disables the PI regulator if the error is to small
    // this avoids to always move as we cannot exactly be where we want and
    // the camera is a bit noisy
    if (fabs(error) < ERROR_THRESHOLD)
    {
        return 0;
    }

    sum_error += error;

    // we set a maximum and a minimum for the sum to avoid an uncontrolled growth
    if (sum_error > MAX_SUM_ERROR)
    {
        sum_error = MAX_SUM_ERROR;
    }
    else if (sum_error < -MAX_SUM_ERROR)
    {
        sum_error = -MAX_SUM_ERROR;
    }

    speed = KP * error + KI * sum_error;

    return (int16_t)speed;
}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg)
{

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time = 0;

    int16_t speed = 0;
    int16_t speed_correction = 0;

    while (1)
    {
        time = chVTGetSystemTime();
        if (mode_one_on && get_object_found())
        {
            // computes the speed to give to the motors
            // distance_cm is modified by the image processing thread
            speed = pi_regulator(get_distance_cm(), get_goal_distance() * 1.5);
            /* computes a correction factor to let
            the robot rotate to be in front of the line*/
            speed_correction = (get_line_position() - (IMAGE_BUFFER_SIZE / 2));

            // if the line is nearly in front of the camera, don't rotate
            if (abs(speed_correction) < ROTATION_THRESHOLD)
            {
                speed_correction = 0;
            }

            if (enabled_motors)
            {
                /* applies the speed from the PI regulator and
                the correction for the rotation*/
                right_motor_set_speed(speed - ROTATION_COEFF * speed_correction / 2);
                left_motor_set_speed(speed + ROTATION_COEFF * speed_correction / 2);
            }
            else
            {
                // stop the motors
                right_motor_set_speed(0);
                left_motor_set_speed(0);
            }
        }

        // 100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void)
{
    chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO,
                      PiRegulator, NULL);
}

void set_enabled_motors(bool enable)
{
    enabled_motors = enable;
}

void toogle_enabled_motors()
{
    enabled_motors = !enabled_motors;
}
