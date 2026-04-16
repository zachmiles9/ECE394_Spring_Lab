#ifndef DQ_CONTROLLER_H
#define DQ_CONTROLLER_H

#include <math.h>

#ifndef PI
#define PI    3.14159265358979323846
#endif
#define OFFSET (2.0 * PI / 3.0)
#define SQRT3_2 0.86602540378 // SQRT3_2 = sqrt(3)/2

// Controller structure containing parameters and state memory
typedef struct {
    // Parameters
    double Kp;
    double Ki;
    double Ts;
    double Vdc;
    double Ls;

    // State Variables (Memory)
    double theta_e;
    double id_err_prev;
    double iq_err_prev;
    double d_int_prev;
    double q_int_prev;
	
	//	Protection
	double integrator_limit;
} DQController;

// Initialize state and parameters
void dq_init(DQController* ctrl, double Kp, double Ki, double Ts, double Vdc, double Ls, double integrator_limit);

// Evaluate DQ update, abc-dq-abc transformations, and PI control
void dq_update(DQController* ctrl, 
               double iLA, double iLB, double iLC, 
               double id_ref, double iq_ref, double omega_e, 
               double* m_a, double* m_b, double* m_c, 
               double* Id_k, double* Iq_k, double* Vd_k, double* Vq_k);

#endif
