#ifndef IMU_H
#define IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include <hal.h>

/** Message containing one measurement from the IMU. */
typedef struct {
    float acceleration[3]; // m/s^2
    float gyro[3]; // rad/s
    float temperature;
    int16_t acc_raw[3];
    int16_t gyro_raw[3];
    int16_t acc_raw_offset[3];
    int16_t gyro_raw_offset[3];
    int16_t acc_raw_filtered[3];
    int16_t gyro_raw_filtered[3];
    uint8_t status;
} imu_msg_t;


 /**
 * @brief   Starts the Inertial Motion Unit (IMU) publisher.
 *          Broadcasts a imu_msg_t message on the /imu topic
 */
void imu_start(void);

 /**
 * @brief   Returns the last accelerometer value measured
 *          for the axis given
 * 
 * @param axis      0-3, respectively x,y or z
 * 
 * @return          Last accelerometer value measured
 */
int16_t get_acc(uint8_t axis);

 /**
 * @brief   Returns the last accelerometer value measured
 *          for the three axis
 * 
 * @param value     pointer to a buffer (of at least a size of 3 * int16_t) 
 *                  to which store the measures   
 */
void get_acc_all(int16_t *values);

 /**
 * @brief   Returns an average of the last "filter_size" axis values 
 *          read from the accelerometer.
 *          
 * 
 * @param axis          0-3, respectively x,y or z
 * @param filter_size   number of samples to take for the averaging process  
 * 
 * @return              Averaged accelerometer measure  
 */
int16_t get_acc_filtered(uint8_t axis, uint8_t filter_size);

 /**
 * @brief   Returns the calibration value of the accelerometer
 *          for the axis given
 * 
 * @param axis      0-3, respectively x,y or z
 * 
 * @return          Calibration value of the accelerometer
 */
int16_t get_acc_offset(uint8_t axis);

 /**
 * @brief   Launches a calibration process of the accelerometer
 */
void calibrate_acc(void);

 /**
 * @brief   Returns the last gyroscope value measured
 *          for the axis given
 * 
 * @param axis      0-3, respectively x,y or z
 * 
 * @return          Last gyroscope value measured
 */
int16_t get_gyro(uint8_t axis);

 /**
 * @brief   Returns the last gyroscope value measured
 *          for the three axis
 * 
 * @param value     pointer to a buffer (of at least a size of 3 * int16_t) 
 *                  to which store the measures   
 */
void get_gyro_all(int16_t *values);

 /**
 * @brief   Returns an average of the last "filter_size" axis values 
 *          read from the gyroscope.
 *          
 * 
 * @param axis          0-3, respectively x,y or z
 * @param filter_size   number of samples to take for the averaging process  
 * 
 * @return              Averaged accelerometer measure  
 */
int16_t get_gyro_filtered(uint8_t axis, uint8_t filter_size);

 /**
 * @brief   Returns the calibration value of the gyroscope
 *          for the axis given
 * 
 * @param axis      0-3, respectively x,y or z
 * 
 * @return          Calibration value of the gyroscope
 */
int16_t get_gyro_offset(uint8_t axis);

 /**
 * @brief   Launches a calibration process of the gyroscope
 */
void calibrate_gyro(void);

 /**
 * @brief   Returns the last temperature value measured
 * 
 * @return          Last temperature value measured in °C
 */
float get_temperature(void);

#ifdef __cplusplus
}
#endif
#endif /* IMU_H */
