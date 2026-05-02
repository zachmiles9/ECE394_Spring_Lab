#ifndef DQ_CONTROLLER_H
#define DQ_CONTROLLER_H

#include <math.h>

// Controller structure containing parameters and state memory
typedef struct {
    // Parameters
    float Kp;
    float Ki;
    float Ts;
    float Vdc;
    float Ls;

    // State Variables (Memory)
    float theta_e;
    float id_err_prev;
    float iq_err_prev;
    float d_int_prev;
    float q_int_prev;
} DQController;

// Initialize state and parameters
void dq_init(DQController* ctrl, float Kp, float Ki, float Ts, float Vdc, float Ls);

// Evaluate DQ update, abc-dq-abc transformations, and PI control
void dq_update(DQController* ctrl, 
               float iLA, float iLB, float iLC, 
               float id_ref, float iq_ref, float omega_e, float theta_hall,
               float* m_a, float* m_b, float* m_c, 
               float* Id_k, float* Iq_k, float* Vd_k, float* Vq_k);

#endif
