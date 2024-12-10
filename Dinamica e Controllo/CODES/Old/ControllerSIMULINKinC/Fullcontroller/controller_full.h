#ifndef CONTROLLER_H_
#define CONTROLLER_H_

//Global Variable
extern double err_vel_last[3];
extern double I_last[3];
extern double vel_fb_last[3];
extern double D_last[3];
extern double q_d[4];
extern double  F_B[3];
extern double M[3];

//Input
extern double dt;
extern double pos_err[3];
extern double vel_fb[3];
extern double q_fb[4];
extern double omega_fb[3];
//Output
extern double thrust[4];





//Function
extern void loop(void);


#endif