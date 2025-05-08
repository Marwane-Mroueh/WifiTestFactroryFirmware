#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

#define MAX_DISTANCE 25.0f
#define ERROR_THRESHOLD 0.1f //[cm] because of the noise of the camera
#define KP 800.0f
#define KI 3.5f // must not be zero
#define MAX_SUM_ERROR (MOTOR_SPEED_LIMIT / KI)
/*...Goal distances defined experimentally...*/
#define DEFAULT_GOAL_DISTANCE 10.0f
#define ATTACK_GOAL_DISTANCE 2.0f

// start the PI regulator thread
void pi_regulator_start(void);

// handle motors
void set_enabled_motors(bool enable);
void toogle_enabled_motors(void);

void set_mode_one_on(bool a);
bool get_mode_one_on(void);

bool get_attack_true(void);
void set_attack_true(bool a);

void set_goal_distance(float a);
float get_goal_distance(void);

#endif /* PI_REGULATOR_H */