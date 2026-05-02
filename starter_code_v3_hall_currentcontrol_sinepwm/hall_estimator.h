#ifndef HALL_ESTIMATOR_H
#define HALL_ESTIMATOR_H

#include <math.h>

// Controller structure containing parameters and state memory
typedef struct {
    // Parameters
    float Tsamp;
    float pp;
    float offset;
    
    // State Variables (Memory)
    int counter;
    int counter2;
    int sector_prev;
    float th_prev;
    float w_est_prev;
} HallEstimator;

// Initialize states and parameters
void hall_init(HallEstimator* ctrl, float Tsamp, float pp, float offset);

// Evaluate Hall logic, sector splitting, and omega estimation
void hall_update(HallEstimator* ctrl, 
                 float hall_a, float hall_b, float hall_c, 
                 float* sector_out, float* omega_out, float* angle_out);

#endif
