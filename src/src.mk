
INCDIR += $(GLOBAL_PATH)/ChibiOS_ext/os/hal/include

INCDIR += $(GLOBAL_PATH)/ChibiOS_ext/os/hal/ports/STM32/STM32F4xx

INCDIR += $(GLOBAL_PATH)/src/epuck1x

INCDIR += $(GLOBAL_PATH)/src/

CSRC += $(GLOBAL_PATH)/ChibiOS_ext/os/hal/ports/STM32/LLD/SPIv1/spi3_slave_lld.c
CSRC += $(GLOBAL_PATH)/ChibiOS_ext/os/hal/ports/STM32/STM32F4xx/dcmi_lld.c
CSRC += $(GLOBAL_PATH)/ChibiOS_ext/os/hal/src/dcmi.c
CSRC += $(GLOBAL_PATH)/ChibiOS_ext/os/hal/src/spi3_slave.c
CSRC += $(GLOBAL_PATH)/src/audio/audio_thread.c
CSRC += $(GLOBAL_PATH)/src/audio/microphone.c
CSRC += $(GLOBAL_PATH)/src/audio/mp45dt02_processing.c
CSRC += $(GLOBAL_PATH)/src/audio/play_melody.c
CSRC += $(GLOBAL_PATH)/src/button.c
CSRC += $(GLOBAL_PATH)/src/camera/camera.c
CSRC += $(GLOBAL_PATH)/src/camera/dcmi_camera.c
CSRC += $(GLOBAL_PATH)/src/camera/ov2640.c
CSRC += $(GLOBAL_PATH)/src/camera/ov7670.c
CSRC += $(GLOBAL_PATH)/src/camera/po6030.c
CSRC += $(GLOBAL_PATH)/src/camera/po8030.c
CSRC += $(GLOBAL_PATH)/src/cmd.c
CSRC += $(GLOBAL_PATH)/src/epuck1x/Asercom.c
CSRC += $(GLOBAL_PATH)/src/epuck1x/Asercom2.c
CSRC += $(GLOBAL_PATH)/src/epuck1x/DataEEPROM.c
CSRC += $(GLOBAL_PATH)/src/epuck1x/I2C/e_I2C_protocol.c
CSRC += $(GLOBAL_PATH)/src/epuck1x/a_d/advance_ad_scan/e_acc.c
CSRC += $(GLOBAL_PATH)/src/epuck1x/a_d/advance_ad_scan/e_ad_conv.c
CSRC += $(GLOBAL_PATH)/src/epuck1x/a_d/advance_ad_scan/e_micro.c
CSRC += $(GLOBAL_PATH)/src/epuck1x/a_d/advance_ad_scan/e_prox.c
CSRC += $(GLOBAL_PATH)/src/epuck1x/acc_gyro/e_lsm330.c
CSRC += $(GLOBAL_PATH)/src/epuck1x/camera/fast_2_timer/e_common.c
CSRC += $(GLOBAL_PATH)/src/epuck1x/codec/e_sound.c
CSRC += $(GLOBAL_PATH)/src/epuck1x/motor_led/advance_one_timer/e_agenda.c
CSRC += $(GLOBAL_PATH)/src/epuck1x/motor_led/advance_one_timer/e_led.c
CSRC += $(GLOBAL_PATH)/src/epuck1x/motor_led/advance_one_timer/e_motors.c
CSRC += $(GLOBAL_PATH)/src/epuck1x/motor_led/advance_one_timer/e_remote_control.c
CSRC += $(GLOBAL_PATH)/src/epuck1x/motor_led/e_init_port.c
CSRC += $(GLOBAL_PATH)/src/epuck1x/uart/e_uart_char.c
CSRC += $(GLOBAL_PATH)/src/epuck1x/utility/utility.c
CSRC += $(GLOBAL_PATH)/src/exti.c
CSRC += $(GLOBAL_PATH)/src/flash/flash.c
CSRC += $(GLOBAL_PATH)/src/i2c_bus.c
CSRC += $(GLOBAL_PATH)/src/ir_remote.c
CSRC += $(GLOBAL_PATH)/src/leds.c
CSRC += $(GLOBAL_PATH)/src/memory_protection.c
CSRC += $(GLOBAL_PATH)/src/motors.c
CSRC += $(GLOBAL_PATH)/src/panic.c
CSRC += $(GLOBAL_PATH)/src/selector.c
CSRC += $(GLOBAL_PATH)/src/sensors/battery_level.c
CSRC += $(GLOBAL_PATH)/src/sensors/ground.c
CSRC += $(GLOBAL_PATH)/src/sensors/imu.c
CSRC += $(GLOBAL_PATH)/src/sensors/mpu9250.c
CSRC += $(GLOBAL_PATH)/src/sensors/proximity.c
CSRC += $(GLOBAL_PATH)/src/serial_comm.c
CSRC += $(GLOBAL_PATH)/src/spi_comm.c
CSRC += $(GLOBAL_PATH)/src/sdio.c
CSRC += $(GLOBAL_PATH)/src/usbcfg.c
CSRC += $(GLOBAL_PATH)/src/uc_usage.c
CSRC += $(GLOBAL_PATH)/src/chibios-syscalls/malloc_lock.c
CSRC += $(GLOBAL_PATH)/src/chibios-syscalls/newlib_syscalls.c
CSRC += $(GLOBAL_PATH)/src/msgbus/examples/chibios/port.c
CSRC += $(GLOBAL_PATH)/src/communication.c
CSRC += $(GLOBAL_PATH)/src/config_flash_storage.c
CSRC += $(GLOBAL_PATH)/src/sensors/VL53L0X/Api/core/src/vl53l0x_api.c
CSRC += $(GLOBAL_PATH)/src/sensors/VL53L0X/Api/core/src/vl53l0x_api_calibration.c
CSRC += $(GLOBAL_PATH)/src/sensors/VL53L0X/Api/core/src/vl53l0x_api_core.c
CSRC += $(GLOBAL_PATH)/src/sensors/VL53L0X/Api/core/src/vl53l0x_api_ranging.c
CSRC += $(GLOBAL_PATH)/src/sensors/VL53L0X/Api/core/src/vl53l0x_api_strings.c
CSRC += $(GLOBAL_PATH)/src/sensors/VL53L0X/Api/platform/src/vl53l0x_i2c_platform.c
CSRC += $(GLOBAL_PATH)/src/sensors/VL53L0X/Api/platform/src/vl53l0x_platform.c
CSRC += $(GLOBAL_PATH)/src/sensors/VL53L0X/VL53L0X.c
CSRC += $(GLOBAL_PATH)/src/serial-datagram/serial_datagram.c
CSRC += $(GLOBAL_PATH)/src/cmp/cmp.c
CSRC += $(GLOBAL_PATH)/src/cmp_mem_access/cmp_mem_access.c
CSRC += $(GLOBAL_PATH)/src/crc/crc16.c
CSRC += $(GLOBAL_PATH)/src/crc/crc32.c
CSRC += $(GLOBAL_PATH)/src/msgbus/messagebus.c
CSRC += $(GLOBAL_PATH)/src/parameter/parameter.c
CSRC += $(GLOBAL_PATH)/src/parameter/parameter_msgpack.c
CSRC += $(GLOBAL_PATH)/src/parameter/parameter_print.c
CSRC += $(GLOBAL_PATH)/src/fat.c
CSRC += $(GLOBAL_PATH)/src/audio/play_sound_file.c
CSRC += $(GLOBAL_PATH)/src/behaviors.c
CSRC += $(GLOBAL_PATH)/src/ircom/ircom.c
CSRC += $(GLOBAL_PATH)/src/ircom/ircomReceive.c
CSRC += $(GLOBAL_PATH)/src/ircom/ircomTools.c
CSRC += $(GLOBAL_PATH)/src/ircom/ircomMessages.c
CSRC += $(GLOBAL_PATH)/src/ircom/ircomSend.c
CSRC += $(GLOBAL_PATH)/src/ircom/transceiver.c
CSRC += $(GLOBAL_PATH)/src/grid.c
CSRC += $(GLOBAL_PATH)/src/ir_sensors.c
CSRC += $(GLOBAL_PATH)/src/pi_regulator.c
CSRC += $(GLOBAL_PATH)/src/process_image.c

