#include "hall_estimator.h"

#define PI 3.14159265359f

void hall_init(HallEstimator* ctrl, float Tsamp, float pp, float offset)
{
    // Assign parameters
    ctrl->Tsamp = Tsamp;
    ctrl->pp = pp;
    ctrl->offset = offset;
    
    // Initialize states
    ctrl->counter = 0;
    ctrl->counter2 = 0;
    ctrl->sector_prev = 0;
    ctrl->th_prev = 0.0f;
    ctrl->w_est_prev = 0.0f;
}

void hall_update(HallEstimator* ctrl, 
                 float hall_a, float hall_b, float hall_c, 
                 float* sector_out, float* omega_out, float* angle_out)
{
    // Increment timing counter for the major step
    ctrl->counter++;
    
    // Decode binary info into decimal place
    int hall = (int)hall_a * 4 + (int)hall_b * 2 + (int)hall_c * 1;
    
    int sector_current = 0;
    float w_est_current = ctrl->w_est_prev; // Default to previous value unless updated
    float th_current = 0.0f;
    float angle = 0.0f;

    // Sector splitting
    if (hall == 4)      sector_current = 1;
    else if (hall == 6) sector_current = 2;
    else if (hall == 2) sector_current = 3;
    else if (hall == 3) sector_current = 4;
    else if (hall == 1) sector_current = 5;
    else if (hall == 5) sector_current = 6;
    else                sector_current = 0;

    // Omega estimation
    if (sector_current != ctrl->sector_prev) 
    {
        ctrl->th_prev = 0.0f;
        ctrl->counter2++;            // Increase a timing counter
        
        // Averaging over a pole pair; estimate omega once it sees every north pole
        if (ctrl->counter2 >= 2)         
        {         
            w_est_current = (2.0f * PI) / (ctrl->counter * ctrl->Tsamp * 3.0f); // Speed in electrical rad/sec
            ctrl->counter  = 0;
            ctrl->counter2 = 0;
        }
    }
    
    // Integrate the omega to obtain theta
    th_current = 0.5f * ctrl->Tsamp * (w_est_current + ctrl->w_est_prev) + ctrl->th_prev;  

    // Angle in a sector cannot be greater than 60 deg
    if (th_current > (2.0f * PI / 6.0f)) 
    {
        th_current = 2.0f * PI / 6.0f;
    }
   
    // Get the angle estimate by adding the crude sector info with the fine angle calculated
    angle = (sector_current - 1) * (2.0f * PI / 6.0f) + th_current - (PI / 6.0f);  

    // Update states for the next time step
    ctrl->sector_prev = sector_current;
    ctrl->th_prev     = th_current;
    ctrl->w_est_prev  = w_est_current;  
 
    // Assign outputs
    *sector_out = (float)sector_current;
    *omega_out  = w_est_current;
    *angle_out  = angle + ctrl->offset;
}
