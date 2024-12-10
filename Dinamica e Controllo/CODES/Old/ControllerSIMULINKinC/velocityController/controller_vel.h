#ifndef CONTROLLER_H_
#define CONTROLLER_H_

//Global Variable

extern double err_vel_last[3];
extern double I[3];

 extern double  F_B[3];
 extern double dt;
 extern double vel_fb[3];
 extern double q_fb[4];
 extern double pos_err[3];
extern double q_d[4];




//Function
extern void loop(void);



#endif