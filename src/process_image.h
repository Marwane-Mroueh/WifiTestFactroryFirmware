#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

// Specify the 2 consecutive lines used for tracking the black line
#define USED_LINE 200 // Must be inside [0..478]

// constants for the PI Regulator/Camera processing
#define IMAGE_BUFFER_SIZE 640
#define WIDTH_SLOPE 5
#define MIN_LINE_WIDTH 40
#define ROTATION_THRESHOLD 10
#define ROTATION_COEFF 2
#define PXTOCM 1570.0f // experimental value

float get_distance_cm(void);
uint16_t get_line_position(void);
void process_image_start(void);
void set_config_cam_pid(bool val);
bool get_config_cam_pid(void);

#endif /* PROCESS_IMAGE_H */