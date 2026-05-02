#include "dq_controller.h"

#define PI 3.14159265359f

#define OFFSET (2.0f*PI/3.0f)

void dq_init(DQController* ctrl, float Kp, float Ki, float Ts, float Vdc, float Ls)
{
    // Assign parameters
    ctrl->Kp = Kp;
    ctrl->Ki = Ki;
    ctrl->Ts = Ts;
    ctrl->Vdc = Vdc;
    ctrl->Ls = Ls;

    // Initialize states to zero
    ctrl->theta_e = 0.0f;
    ctrl->id_err_prev = 0.0f;
    ctrl->iq_err_prev = 0.0f;
    ctrl->d_int_prev = 0.0f;
    ctrl->q_int_prev = 0.0f;
}

void dq_update(DQController* ctrl, 
               float iLA, float iLB, float iLC, 
               float id_ref, float iq_ref, float omega_e, float theta_hall,
               float* m_a, float* m_b, float* m_c, 
               float* Id_k, float* Iq_k, float* Vd_k, float* Vq_k)
{
    // Local intermediate variables
    float id, iq, id_err, iq_err, d_int, q_int, vd, vq, md, mq;

    // Reference angle generation
    ctrl->theta_e = ctrl->theta_e + omega_e * ctrl->Ts;
    if (ctrl->theta_e > 2.0f * PI) {
        ctrl->theta_e = ctrl->theta_e - 2.0f * PI;
    }

    /**********************************************************************
     * TODO 1: Implement the amplitude-invariant abc-to-dq transformation.
     *
     * Use the measured phase currents iLA, iLB, iLC and the electrical
     * angle ctrl->theta_e to compute:
     *
     *   id
     *   iq
     *
     * Use the amplitude-invariant scaling factor:
     *
     *   2.0 / 3.0
     *
     * and the class convention:
     *
     *   id uses cosine terms
     *   iq uses negative sine terms
     *
     * Helpful reminder:
     *
     *   phase B angle = theta_e - OFFSET
     *   phase C angle = theta_e + OFFSET
     *
     **********************************************************************/
    
    /* STUDENT CODE STARTS HERE */

    id = (2.0f / 3.0f) * ( iLA * cosf(theta_hall) + 
                           iLB * cosf(theta_hall - OFFSET) + 
                           iLC * cosf(theta_hall + OFFSET) );

    iq = (2.0f / 3.0f) * ( -iLA * sinf(theta_hall) 
                           -iLB * sinf(theta_hall - OFFSET) 
                           -iLC * sinf(theta_hall + OFFSET) );

    /* STUDENT CODE ENDS HERE */

    id_err = id_ref - id;
    iq_err = iq_ref - iq;

    /**********************************************************************
     * TODO 2: Implement the trapezoidal PI integrators.
     *
     * Compute:
     *
     *   d_int
     *   q_int
     *
     * using:
     *   - current error
     *   - previous error
     *   - previous integrator state
     *   - ctrl->Ki
     *   - ctrl->Ts
     *
     * Do NOT use backward Euler here.
     *
     * Reminder:
     * trapezoidal integration uses the average of the current and previous
     * input values over one time step.
     *
     **********************************************************************/
    
    /* STUDENT CODE STARTS HERE */

    d_int = ctrl->d_int_prev + ctrl->Ki * (ctrl->Ts / 2.0f) * (id_err + ctrl->id_err_prev);
    
    q_int = ctrl->q_int_prev + ctrl->Ki * (ctrl->Ts / 2.0f) * (iq_err + ctrl->iq_err_prev);

    /* STUDENT CODE ENDS HERE */

    // Voltage calculation with cross-coupling decoupling
    vd = d_int + ctrl->Kp * id_err - omega_e * ctrl->Ls * iq;
    vq = q_int + ctrl->Kp * iq_err + omega_e * ctrl->Ls * id;

    // Modulation indices calculation
    md = vd * (2.0f / ctrl->Vdc);
    mq = vq * (2.0f / ctrl->Vdc);

    // Save current states for the next time step
    ctrl->id_err_prev = id_err;
    ctrl->iq_err_prev = iq_err;
    ctrl->d_int_prev = d_int;
    ctrl->q_int_prev = q_int;

    /**********************************************************************
     * TODO 3: Implement the dq-to-abc transformation for modulation.
     *
     * Use md and mq to compute:
     *
     *   *m_a
     *   *m_b
     *   *m_c
     *
     * Use the same electrical angle convention as above.
     *
     * Helpful reminder:
     *
     *   phase A uses theta_e
     *   phase B uses theta_e - OFFSET
     *   phase C uses theta_e + OFFSET
     *
     **********************************************************************/
    
    /* STUDENT CODE STARTS HERE */

    *m_a = md * cosf(theta_hall) - mq * sinf(theta_hall);
    *m_b = md * cosf(theta_hall - OFFSET) - mq * sinf(theta_hall - OFFSET);
    *m_c = md * cosf(theta_hall + OFFSET) - mq * sinf(theta_hall + OFFSET);

    /* STUDENT CODE ENDS HERE */

    // Duty cycle saturation limits
    if (*m_a >= 0.99f) *m_a = 0.99f;
    else if (*m_a <= -0.99f) *m_a = -0.99f;

    if (*m_b >= 0.99f) *m_b = 0.99f;
    else if (*m_b <= -0.99f) *m_b = -0.99f;

    if (*m_c >= 0.99f) *m_c = 0.99f;
    else if (*m_c <= -0.99f) *m_c = -0.99f;

    // Assign outputs
    *Id_k = id;
    *Iq_k = iq;
    *Vd_k = vd;
    *Vq_k = vq;
}
