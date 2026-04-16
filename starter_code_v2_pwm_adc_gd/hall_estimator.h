#ifndef HALL_ESTIMATOR_H
#define HALL_ESTIMATOR_H

#include <math.h>

#ifndef PI
#define PI 3.14159265359 // Increased precision from 3.14
#endif

// Controller structure containing parameters and state memory
typedef struct {
    // Parameters
    double Tsamp;
    double pp;
    
    // State Variables (Memory)
    int counter;
    int counter2;
    int sector_prev;
    double th_prev;
    double w_est_prev;
} HallEstimator;

// Initialize states and parameters
void hall_init(HallEstimator* ctrl, double Tsamp, double pp);

// Evaluate Hall logic, sector splitting, and omega estimation
void hall_update(HallEstimator* ctrl, 
                 double hall_a, double hall_b, double hall_c, 
                 double* sector_out, double* omega_out, double* angle_out);

#endif
