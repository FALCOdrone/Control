#ifndef CTRL_H
#define CTRL_H

#include "common/types.h"
#include <Arduino.h>

class Controller {
    private:

        // Controller parameters (take note of defaults before modifying!):
        float i_limit = 25.0;   // Integrator saturation level, mostly for safety (default 25.0)

        float Kp_roll_angle = 0.22694;    // Roll P-gain - angle mode
        float Ki_roll_angle = 0.0132408;    // Roll I-gain - angle mode
        float Kd_roll_angle = 0.86403;   // Roll D-gain - angle mode (has no effect on controlANGLE2)
        float Kp_pitch_angle = 0.1444;   // Pitch P-gain - angle mode
        float Ki_pitch_angle = 0.00548;   // Pitch I-gain - angle mode
        float Kd_pitch_angle = 0.4367;  // Pitch D-gain - angle mode (has no effect on controlANGLE2)

        float Kp_yaw = 0.0350;      // Yaw P-gain
        float Kd_yaw = 0.8;  // Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)
        float Kp_pos_z = 19.29;
        float Kd_pos_z = 18.41;
        float takeoffGain = 0.1;
        float g = 9.81;
        float vehicleMass = 3.76946;
        float thrust[4];

    public:

        Controller(float thrust[4]);
        void gravityFeedforward_equilibriumThrust(vec_t desPos, PID_altitude *PID, vec_t pos_est, vec_t vel_est, float dt_prev);
        void attitudePID(attitude_t ref_attitude, attitude_t est_attitude, PID_attitude *PID, float throttleCmd);
};

#endif