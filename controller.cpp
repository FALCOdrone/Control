#include "controller.h"

Controller::Controller(float thrust[4]) {
    this->thrust = thrust;
}

void Controller::gravityFeedforward_equilibriumThrust(vec_t desPos, PIDpos_t *PID, vec_t pos_est, vec_t vel_est, float dt_prev) {
    
    float currTime = millis();
    float dt = desPos.dt;
    bool takeoff_flag = 1;

    if (currTime >= 1000) { // at 1 second the flag takeoff_flag is set to 0, see on matlab the takeoff_flag time series
        takeoff_flag = 0;

        PID->p.z = desPos.z - pos_est.z;  // Error in z-axis
        PID->d.z = (PID->p.z - PID->pPrev.z) / dt;
        PID->out.z = 0.01 * (Kp_pos_z * PID->p.z + Kd_pos_z * PID->d.z);  // PD controller for z-axis position
    } else {
        PID->out.z = -takeoffGain*(g * vehicleMass);
    }
    PID->out.z = PID->out.z - g * vehicleMass;

    thrust[3] = PID->out.z; // from simulink is the totalThrust signal

    // Update z variables
    PID->pPrev.z = PID->p.z;
}

void controller::attitudePID(attitude_t ref_attitude, attitude_t est_attitude, PIDattitude_t *PID, float throttleCmd) {

    float dt_pitch = ref_attitude.dt;
    float dt_roll = ref_attitude.dt;
    float dt_yaw = ref_attitude.dt;

    // pitch PID
    PID->p.pitch = ref_attitude.pitch - est_attitude.pitch;
    PID->i.pitch = PID->iPrev.pitch + PID->p.pitch * dt_pitch;

    if (throttleCmd < 1060) {  // Don't let integrator build if throttle is too low
        PID->i.pitch = 0;
    }

    PID->i.pitch = constrain(PID->i.pitch, -i_limit, i_limit);  // Saturate integrator to prevent unsafe buildup
    PID->d.pitch = (PID->p.pitch - PID->pPrev.pitch) / dt_pitch;
    PID->out.pitch = .01 * (Kp_pitch_angle * PID->p.pitch + Ki_pitch_angle * PID->i.pitch - Kd_pitch_angle * PID->d.pitch);  // Scaled by .01 to bring within -1 to 1 range

    thrust[0] = PID->out.pitch; // from simulink is the tau_pitch signal

    // roll PID
    PID->p.roll = ref_attitude.roll - est_attitude.roll;
    PID->i.roll = PID->iPrev.roll + PID->p.roll * dt_roll;

    if (throttleCmd < 1060) {  // Don't let integrator build if throttle is too low
        PID->i.roll = 0;
    }

    PID->i.roll = constrain(PID->i.roll, -i_limit, i_limit);  // Saturate integrator to prevent unsafe buildup
    PID->d.roll = (PID->p.roll - PID->pPrev.roll) / dt_roll;
    PID->out.roll = .01 * (Kp_roll_angle * PID->p.roll + Ki_roll_angle * PID->i.roll - Kd_roll_angle * PID->d.roll);  // Scaled by .01 to bring within -1 to 1 range

    thrust[1] = PID->out.roll; // from simulink is the tau_roll signal

    // yaw PD
    PID->p.yaw = ref_attitude.yaw - est_attitude.yaw;
    PID->d.yaw = (PID->p.yaw - PID->pPrev.yaw) / dt_pitch;

    PID-out.yaw = .01 * (Kp_yaw * PID->p.yaw + Kd_yaw * PID->d.yaw);  // Scaled by .01 to bring within -1 to 1 range
    thrust[2] = PID->out.yaw; // from simulink is the tau_yaw signal

    PID->iPrev.pitch = PID->i.pitch;
    PID->pPrev.roll = PID->p.roll;
    PID->dPrev.roll = PID->d.roll;
    PID->iPrev.roll = PID->i.roll;
    PID->pPrev.yaw = PID->p.yaw;
    PID->dPrev.yaw = PID->d.yaw;
}
