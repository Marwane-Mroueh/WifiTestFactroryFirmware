#ifndef GRID_H
#define GRID_H

// motor speed for mode 1 [step/s]
#define GRID_MOTOR_SPEED 500

void grid_move_start(void);
extern thread_reference_t grid_ref;
#endif /* GRID_H */
