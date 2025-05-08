#include <ch.h>
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <stdio.h>
#include <motors.h>
#include <sensors/proximity.h>

#include "main.h"
#include "ir_sensors.h"
// #include "pi_regulator.h"
#include "grid.h"

#define IR_ROTATION_COEF 1.06 /*Determined experimentally to ensure a 90° rotation */
#define IR_ROTATION_SPEED 350 /*[Steps/s]*/

static bool object_close = false;
static bool object_found = false;
static bool pause_request = false;

/*...Function declarations...*/
void send_ir_data_to_computer(void);
void turn_to_find_object(int *proximity_counter);
void turn_back_to_continue(int *proximity_counter);
void handle_object_found(int *proximity_counter);

/*...Setter/Getter declarations...*/
bool get_object_close(void);
bool get_object_found(void);
void set_object_close(bool val);
void set_object_found(bool val);
bool get_pause_request(void);
void set_pause_request(bool val);

/*................Thread that processes IR data................*/
static THD_WORKING_AREA(waIRPrintThread, 256);
static THD_FUNCTION(IRPrintThread, arg)
{
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    calibrate_ir();
    uint16_t prox_value = 0;
    int proximity_counter = 0;
    while (1)
    {
        if (object_found == false /*&& get_mode_one_on()*/)
        {
            prox_value = 0;
            for (int i = IR5_ID; i <= IR8_ID; i++)
            {
                /* Gets the IR value only if no object is detected, and for IR1 and 8*/
                if ((object_close == false) || i == IR1_ID || i == IR8_ID)
                    prox_value = get_calibrated_prox(i);

                if (prox_value > IR_THRESHOLD && object_close == false)
                    object_close = true;
                /*Checks if an object has been detected, and if it hasnt been to long*/
                if (object_close == true && proximity_counter < PROXIMITY_TIMEOUT)
                    turn_to_find_object(&proximity_counter);
                /*If it has been too long, continue the grid*/
                else if (object_close == true)
                    turn_back_to_continue(&proximity_counter);
                /*Checks if the object is now in front of the robot*/
                if ((i == IR1_ID || i == IR8_ID) && (prox_value > IR_THRESHOLD))
                    handle_object_found(&proximity_counter);
            }
        }
        /*if (!get_mode_one_on())
            send_ir_data_to_computer();*/
        chThdSleepMilliseconds(100);
    }
}

/*............Function definitions............*/
void turn_to_find_object(int *proximity_counter)
{
    (*proximity_counter)++;
    pause_request = true;

    /*...Checks if there is a reference to the suspended grid thread...*/
    while (grid_ref == NULL)
        chThdSleepMilliseconds(10); // Small delay between checks

    /*...Turn to try and find the detected presence...*/
    right_motor_set_speed(IR_ROTATION_SPEED);
    left_motor_set_speed(-IR_ROTATION_SPEED);
    chThdSleepMilliseconds(100);
}

void turn_back_to_continue(int *proximity_counter)
{
    *proximity_counter = 0;
    object_close = false;
    object_found = false;
    for (int i = 0; i < PROXIMITY_TIMEOUT; ++i)
    {
        /*...Turn back to continue if did not find a presence...*/
        right_motor_set_speed(-IR_ROTATION_SPEED * IR_ROTATION_COEF);
        left_motor_set_speed(IR_ROTATION_SPEED * IR_ROTATION_COEF);
        chThdSleepMilliseconds(120);
    }
    right_motor_set_speed(0);
    left_motor_set_speed(0);
    /*...Resume grid thread...*/
    pause_request = false;
    chThdResume(&grid_ref, MSG_OK);
}

void handle_object_found(int *proximity_counter)
{
    /*...Resume grid thread only if it was suspended...*/
    /*It is possible to get to this point without
    suspending the grid thread, if the object was directly
    detected from IR1 or IR8*/
    *proximity_counter = 0;
    pause_request = false;
    if (grid_ref != NULL)
        chThdResume(&grid_ref, MSG_OK);
    object_found = true;
    object_close = false;
}

void send_ir_data_to_computer(void)
{
    for (uint8_t i = IR1_ID; i <= IR8_ID; i++)
    {
        uint16_t v = get_calibrated_prox(i);
        uint8_t buf[5]; // 5 bytes
        /*...One packet is composed of [I][D][ID_val][LSB][MSB]...*/
        buf[0] = 'I';
        buf[1] = 'D';
        buf[2] = i + 1;
        buf[3] = (uint8_t)(v & 0xFF);        // Mask to get LSB
        buf[4] = (uint8_t)((v >> 8) & 0xFF); // mask to get MSB

        // chSequentialStreamWrite((BaseSequentialStream *)&SD3,
        // buf, sizeof(buf));
    }
}

/*...Thread starter...*/
void ir_sensors_start(void)
{
    chThdCreateStatic(waIRPrintThread,
                      sizeof(waIRPrintThread), NORMALPRIO, IRPrintThread, NULL);
}

/*...Setters and Getters for all the booleans...*/
bool get_object_close(void)
{
    return object_close;
}
bool get_object_found(void)
{
    return object_found;
}

void set_object_close(bool val)
{
    object_close = val;
    return;
}
void set_object_found(bool val)
{
    object_found = val;
    return;
}

bool get_pause_request(void)
{
    return pause_request;
}

void set_pause_request(bool val)
{
    pause_request = val;
    return;
}