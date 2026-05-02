#include "pi_controller.h"

void pi_init(PIController* ctrl, float Kp, float Ki, float output_limit, float integrator_limit)
{
    ctrl->Kp = Kp;
    ctrl->Ki = Ki;
    ctrl->integral = 0.0f;
    ctrl->old_error = 0.0f;
    ctrl->output_limit = output_limit;
    ctrl->integrator_limit = integrator_limit;
}

void pi_update(PIController* ctrl, float error, float Ts, bool integrator_enabled, float* output)
{
    if (!integrator_enabled) {
        ctrl->integral = 0.0f;
        *output = 0.0f;
        return;
    }

    // Integrate error (trapezoidal)
    ctrl->integral += ctrl->Ki * 0.5f * Ts * (error + ctrl->old_error);
    ctrl->old_error = error;

    // Clamp integral (anti-windup)
    if (ctrl->integrator_limit > 0.0f) {
        if (ctrl->integral > ctrl->integrator_limit) ctrl->integral = ctrl->integrator_limit;
        else if (ctrl->integral < -ctrl->integrator_limit) ctrl->integral = -ctrl->integrator_limit;
    }

    // Compute total output
    float u = ctrl->Kp * error + ctrl->integral;

    // Clamp output (optional)
    if (ctrl->output_limit > 0.0f) {
        if (u > ctrl->output_limit) u = ctrl->output_limit;
        else if (u < -ctrl->output_limit) u = -ctrl->output_limit;
    }
    
    *output = u;
}
