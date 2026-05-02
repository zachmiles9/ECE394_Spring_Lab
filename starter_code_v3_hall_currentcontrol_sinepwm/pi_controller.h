#ifndef PI_CONTROLLER_H
#define PI_CONTROLLER_H
#include <stdbool.h>

typedef struct {
    float Kp;
    float Ki;
    float integral;
    float old_error;
    float output_limit;
    float integrator_limit;
} PIController;

// Initialize state and gains
void pi_init(PIController* ctrl, float Kp, float Ki, float output_limit, float integrator_limit);

// Evaluate PI update: output = Kp*error + Ki * ∫error
void pi_update(PIController* ctrl, float error, float Ts, bool integrator_enabled, float* output);

#endif
