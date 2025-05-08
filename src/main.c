#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "chprintf.h"
#include "hal.h"
#include "shell.h"

#include "aseba_vm/aseba_node.h"
#include "aseba_vm/skel_user.h"
#include "aseba_vm/aseba_can_interface.h"
#include "aseba_vm/aseba_bridge.h"
#include "audio/audio_thread.h"
#include "audio/play_melody.h"
#include "audio/play_sound_file.h"
#include "audio/microphone.h"
#include "camera/camera.h"
#include "epuck1x/Asercom.h"
#include "epuck1x/Asercom2.h"
#include "epuck1x/a_d/advance_ad_scan/e_acc.h"
#include "epuck1x/motor_led/advance_one_timer/e_led.h"
#include "epuck1x/utility/utility.h"
#include "sensors/battery_level.h"
#include "sensors/ground.h"
#include "sensors/imu.h"
#include "sensors/mpu9250.h"
#include "sensors/proximity.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "behaviors.h"
#include "button.h"
#include "cmd.h"
#include "config_flash_storage.h"
#include "exti.h"
#include "fat.h"
#include "i2c_bus.h"
#include "ir_remote.h"
#include "leds.h"
#include <main.h>
#include "memory_protection.h"
#include "motors.h"
#include "sdio.h"
#include "selector.h"
#include "serial_comm.h"
#include "spi_comm.h"
#include "usbcfg.h"
#include "communication.h"
#include "uc_usage.h"
#include "ircom/ircom.h"
#include "ircom/ircomReceive.h"
#include "ircom/ircomMessages.h"
#include "ircom/ircomSend.h"
#include "ircom/transceiver.h"
#include "grid.h"
#include "ir_sensors.h"
#include "pi_regulator.h"
#include "process_image.h"
uint8_t stop_loop = 0;
uint8_t demo15_state = 0;
uint8_t demo15_state_1 = 0;
bool one_time_2 = false;
bool one_time = false;

bool get_one_time(void)
{
	return one_time;
}
bool get_demo15_state_1(void)
{
	return demo15_state_1;
}

#define SHELL_WA_SIZE THD_WORKING_AREA_SIZE(2048)

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

parameter_namespace_t parameter_root, aseba_ns;

static THD_WORKING_AREA(selector_thd_wa, 2048);

static bool load_config(void)
{
	extern uint32_t _config_start;

	return config_load(&parameter_root, &_config_start);
}

static THD_FUNCTION(selector_thd, arg)
{
	(void)arg;
	chRegSetThreadName(__FUNCTION__);

	systime_t time;

	messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
	proximity_msg_t prox_values;
	int16_t leftSpeed = 0, rightSpeed = 0;
	int16_t prox_values_temp[8];

	messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
	imu_msg_t imu_values;

	uint16_t prox_thr = 1000;

	uint8_t hw_test_state = 0;
	uint8_t *img_buff_ptr;
	uint16_t r = 0, g = 0, b = 0;
	uint8_t rgb_state = 0, rgb_counter = 0;
	uint16_t melody_state = 0, melody_counter = 0;
	int8_t cam_error = 0;

	uint8_t magneto_state = 0;

	uint8_t temp_rx = 0;

	uint8_t rab_addr = 0x20;
	uint8_t rab_state = 0;
	int8_t i2c_err = 0;
	uint8_t regValue[2] = {0};
	uint16_t rab_data = 0;
	double rab_bearing = 0.0;
	uint16_t rab_range = 0;
	uint16_t rab_sensor = 0;
	uint8_t rab_buff[35];
	uint16_t rab_tx_data = 0;
	uint8_t rab_counter = 0;

	uint8_t back_and_forth_state = 0;
	float turn_angle_rad = 0.0;
	uint8_t led_animation_state = 0;
	uint32_t led_animation_count = 0;

	uint8_t wav_volume = 20;
	uint8_t wav_play_state = 0;

	double heading = 0.0;
	float mag_values[3];

	calibrate_acc();
	calibrate_gyro();
	calibrate_ir();

	while (stop_loop == 0)
	{
		time = chVTGetSystemTime();

		switch (get_selector())
		{
		case 15:
			switch (demo15_state)
			{
			case 0:

				if (cam_advanced_config(FORMAT_COLOR, 0, 0, 640, 480, SUBSAMPLING_X4, SUBSAMPLING_X4) != MSG_OK)
				{
					set_led(LED1, 1);
				}

				if (one_time_2 == false)
				{
					cam_set_exposure(512, 0); // Fix the exposure to have a stable framerate.

					dcmi_set_capture_mode(CAPTURE_ONE_SHOT);

					if (dcmi_prepare() < 0)
					{
						set_led(LED5, 1);
					}
				}

				spi_image_transfer_enable();

				mpu9250_magnetometer_setup();

				// Flush the uart input to avoid interpreting garbage as real commands.
				while (chnReadTimeout(&SD3, (uint8_t *)&temp_rx, 1, MS2ST(1) > 0))
				{
					chThdSleepMilliseconds(1);
				}

				demo15_state = 1;
				demo15_state_1 = 1;
				break;

			case 1:
				if (one_time_2 == false)
					run_asercom2();
				stop_loop = 1;
				break;
			}
			break;
		}
		chThdSleepMilliseconds(100);
	}
	while (1)
	{
		chThdSleepMilliseconds(1000);
	}
}

int main(void)
{

	halInit();
	chSysInit();
	mpu_init();

	/** Inits the Inter Process Communication bus. */
	messagebus_init(&bus, &bus_lock, &bus_condvar);

	parameter_namespace_declare(&parameter_root, NULL, NULL);

	// Init the peripherals.
	clear_leds();
	set_body_led(0);
	set_front_led(0);
	usb_start();
	dcmi_start();
	cam_start();
	motors_init();
	proximity_start(FAST_UPDATE);
	battery_level_start();
	dac_start();
	exti_start();
	imu_start();
	ir_remote_start();
	spi_comm_start();
	VL53L0X_start();
	serial_start();
	mic_start(NULL);
	sdio_start();
	playMelodyStart();
	playSoundFileStart();
	ground_start();
	behaviors_start();
	grid_move_start();
	ir_sensors_start();

	// Initialise Aseba system, declaring parameters
	parameter_namespace_declare(&aseba_ns, &parameter_root, "aseba");
	aseba_declare_parameters(&aseba_ns);

	/* Load parameter tree from flash. */
	load_config();

	/* Start AsebaCAN. Must be after config was loaded because the CAN id
	 * cannot be changed at runtime. */
	aseba_vm_init();
	aseba_can_start(&vmState);

	chThdCreateStatic(selector_thd_wa, sizeof(selector_thd_wa), NORMALPRIO, selector_thd, NULL);

	/* Infinite loop. */

	while (1)
	{
		chThdSleepMilliseconds(100);
		if (get_object_found() == true && !one_time)
		{
			spi_image_transfer_disable();
			// set_end_asercom2(true);
			left_motor_set_speed(0);
			right_motor_set_speed(0);

			set_enabled_motors(true);
			process_image_start();
			pi_regulator_start();

			one_time = true;
			demo15_state_1 = 0;
		}
		if (get_object_found() == 0 && one_time == 1)
		{
			if (one_time_2 == 0)
			{
				left_motor_set_speed(0);
				right_motor_set_speed(0);
			}
			one_time_2 = true;
			demo15_state = 0;
			stop_loop = 0;
		}
	}
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
	chSysHalt("Stack smashing detected");
}
