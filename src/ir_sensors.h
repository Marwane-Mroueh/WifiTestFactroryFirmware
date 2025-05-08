#ifndef IR_SENSORS_H
#define IR_SENSORS_H

/*IRxx_ID correspond to the xx-1 so that it could be used in for loops*/
#define IR1_ID 0
#define IR5_ID 4
#define IR8_ID 7

/*...Determined Experimentally...*/
#define IR_THRESHOLD 1000
#define PROXIMITY_TIMEOUT 12

/*...Start the IR Sensors thread...*/
void ir_sensors_start(void);

/*...Getters and Setters...*/
bool get_object_found(void);
bool get_object_close(void);
bool get_pause_request(void);
void set_pause_request(bool val);
void set_object_found(bool val);
void set_object_close(bool val);

#endif /* IR_SENSORS_H */
