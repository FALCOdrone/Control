// I have opted for global variables to save the inputs and the outputs of every function,
// since a function can only return one value and we need to work with 3D vector.

// The script ..._fb stsands for feed-back, and represents the numerical values of the vectors that will 
//be outputed from the sensors + filters and feeded back to the control algorothm;

float setpoint[3];
float pos_fb[3];
float pos_err[3];

// ..._des stands for desired;
// err_vel_last stands for the velocity error acknowledged in the last loop run, and it's needed to update the error integral over time;

float Kp[3] = {0.25,0.25,0.5};
float vel_des[3];
float err_vel_last[3];
float vel_fb[3];
float vel_sat = ;

float PID_xy[3] = {0,0,0};  // x and y have equal PID gains due to simmetry;
float PID_z[3] = {0,0,0};
float I[3];
float dt =;
float acc_des[3];

float m =;
float g = 9.81;

float q_fb[4]; 
float F_B[3];
float q_d[4];
float omega_fb[4];
float roll_pitch_PD[2] = {0,0};
float yaw_PD[2] = {0,0};
float M[3];

float thrust[4];

// This function 

float cross_product_x();
float cross_product_y();
float cross_product_z();


void setpoint_generator();
void error_generator();

void position_controller();
void velocity_controller();
void feedforward_gravity_comp();
void extract_attitude_setpoint();
void attitude_regulator();
void mixer_matrix();

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

  // STEP 1: setpoint generator;
  setpoint_generator();

  //STEP 2: error generator;
  error_generator();

  // STEP 3 : position controller:
  position_controller()

  // STEP 4: velocity controller:
  velocity_controller()

  // STEP 5: feedforward gravity compensation:
  feedforward_gravity_comp();

  // STEP 6: Exctract attitude setpoint + convert forces from NED to BFR:
  extract_attitude_setpoint();

  // STEP 7: Attitude regulator:
  attitude_regulator();

  // STEP 8: Mixer Matrix:
  mixer_matrix();
}


float cross_product_x(b,c,m,n){

  float x = b*n - c*m;

  return x;
}

float cross_product_y(a,c,l,n){

  float y = c*l - a*n;

  return y;
}

float cross_product_z(a,b,l,m){

  float z = a*m - b*l;

  return z;
}

void setpoint_generator(){
  
  float x_setpoint = ;
  float y_setpoint = ;
  float z_setpoint = ;

  setpoint[1] = x_setpoint;
  setpoint[2] = y_setpoint;
  setpoint[3] = z_setpoint;
}

void error_generator(){

  for(int i = 0, i < 3, i++){
    pos_err[i] = setpoint[i] - pos_fb[i];
  }
}

void position_controller(){

  for(int i = 0, i < 3, i++){
    vel_des[i] = pos_err[i] * Kp[i];
    
    // Add saturation:
    if(vel_des[i] > vel_sat){
      vel_des[i] = vel_sat;
    }
  }
}

void velocity_controller(){

  // X direction:
  
  // Proportional:
  float p_x = (vel_des[0] - vel_fb[0]) * PID_xy[0];
  //Integral:
  float err_vel = (vel_des[0] - vel_fb[0]);
  I[0] = I[0] + (err_vel + err_vel_last[0]) * dt/2;
  float i_x = I[0] * PID_xy[1];
  // Derivative:
  float d_x = (err_vel - err_vel_last[0])/dt * PID_xy[2];

  acc_des[0] = p_x + i_x + d_x;

  err_vel_last[0] = err_vel;

  // Y direction:
  
  // Proportional:
  float p_y = (vel_des[1] - vel_fb[1]) * PID_xy[0];
  //Integral:
  float err_vel = (vel_des[1] - vel_fb[1]);
  I[1] = I[1] + (err_vel + err_vel_last[1]) * dt/2;
  float i_y = I[1] * PID_xy[1];
  // Derivative:
  float d_y = (err_vel - err_vel_last[1])/dt * PID_xy[2];

  acc_des[1] = p_y + i_y + d_y;

  err_vel_last[1] = err_vel;

  // Z direction:

  // Proportional:
  float p_z = (vel_des[2] - vel_fb[2]) * PID_z[0];
  //Integral:
  float err_vel = (vel_des[2] - vel_fb[2]);
  I[2] = I[2] + (err_vel + err_vel_last[2]) * dt/2;
  float i_z = I[2] * PID_z[1];
  // Derivative:
  float d_z = (err_vel - err_vel_last[2])/dt * PID_xy[2];

  acc_des[2] = p_z + i_z + d_z;

  err_vel_last[2] = err_vel;
}

void feedforward_gravity_comp(){

  acc_des[2] = acc_des[2] - m*g;
}

void extract_attitude_setpoint(){

  // Step 1: define matrix 'A' from q_fb == q0 from Simulink;

  float q[4];

  float norm_q_fb = sqrt(q_fb[0]^2 + q_fb[1]^2 + q_fb[2]^2 + q_fb[3]^2);

  q[0] = q_fb[0]/norm_q_fb;
  q[1] = q_fb[1]/norm_q_fb;
  q[2] = q_fb[2]/norm_q_fb;
  q[3] = q_fb[3]/norm_q_fb;

  int q_v[3];

  q_v[0] = q[0];
  q_v[1] = q[1];
  q_v[2] = q[2];

  float A_1_1 = q[3] - (q_v[0]^2 + q_v[1]^2 + q_v[2]^2);

  float A_row_1[3] = {A_1_1,0,0};

  A_row_1[0] += 2*q_v[0]^2;
  A_row_1[1] += q_v[0] * q_v[1];
  A_row_1[2] += q_v[0] * q_v[2];

  A_row_1[1] -= 2 * q[3] * (- q[2]);
  A_row_1[2] -= 2 * q[3] * q[1];

  // Step 2: define array b1, b2, b3, row vectors of matrix 'R'
  // Note that only the first row of matrix A is needed!!!
  // acc_des == F_I from Symulink;

  float b1_d[3] = {A_row_1[0], A_row_1[1], A_row_1[2]};

  float norm_acc_des = sqrt(acc_des[0]^2 + acc_des[1]^2 + acc_des[2]^2);
  float b3[3] = {acc_des[0]/norm_acc_des, acc_des[1]/norm_acc_des, acc_des[2]/norm_acc_des};
  float b2_c[3];

  b2_c[0] = cross_product_x(b3[1],b3[2],b1_d[1],b1_d[2]);
  b2_c[1] = cross_product_y(b3[0],b3[2],b1_d[0],b1_d[2]);
  b2_c[2] = cross_product_z(b3[0],b3[1],b1_d[0],b1_d[1]);

  float norm_b2_c = sqrt(b2_c[0]^2 + b2_c[1]^2 + b2_c[2]^2);

  float b2[3] = {b2_c[0]/norm_b2_c, b2_c[1]/norm_b2_c, b2_c[2]/norm_b2_c};

  float b1[3];

  b1[0] = cross_product_x(b2[1],b2[2],b2[1],b3[2]);
  b1[1] = cross_product_y(b2[0],b2[2],b3[0],b3[2]);
  b1[2] = cross_product_z(b2[0],b2[1],b3[0],b3[1]);

  // Step 3: define array 'vector' 

  float vector[4] = {b1[0], b2[1], b3[2], b1[0]+b2[1]+b3[2]};

  // Find max of 'vector' and its index

  float max = vector[0];
  int j = 0;

  for(int i=1, i < 4,i++){
    if(vector[i] > max){
      max = vector[i];
      j = i;
    }
  }
  
  switch j
  {
    case 0:
    q_d[0] = 1 + b1[0] - b2[1] - b3[2];
    q_d[1] = b1[1] + b2[0];
    q_d[2] = b1[2] + b3[0];
    q_d[3] = b2[2] - b3[1];
    break;

    case 1:
    q_d[0] = b2[0] + b1[1];
    q_d[1] = 1 + b2[1] - b3[2] - b1[0];
    q_d[2] = b2[2] + b3[1];
    q_d[3] = b3[0] - b1[2];
    break;

    case 2:
    q_d[0] = b3[0] + b1[2];
    q_d[1] = b3[1] + b2[2];
    q_d[2] = 1 + b3[2] - b1[0] - b2[1];
    q_D[3] = b1[1] - b2[0];
    break;

    case 3:
    q_d[0] = b2[2] - b3[1];
    q_d[1] = b3[0] - b1[2];
    q_d[2] = b1[1] - b2[0];
    q_d[3] = 1 + b1[0] + b2[1] + b3[2];
    break;
  }

  // Last step: define desired attitude and forces in bf and save it in global variables;

  float norm_q_d = sqrt(q_d[0]^2 + q_d[1]^2 + q_d[2]^2 + q_d[3]^2);

  q_d[0] = q_d[0]/norm_q_d; 
  q_d[1] = q_d[1]/norm_q_d;
  q_d[2] = q_d[2]/norm_q_d;
  q_d[3] = q_d[3]/norm_q_d;

  F_B[0] = b1[0] * acc_des[0] + b1[1] * acc_des[1] + b1[2] * acc_des[2];
  F_B[1] = b2[0] * acc_des[0] + b2[1] * acc_des[1] + b2[2] * acc_des[2];
  F_B[2] = b3[0] * acc_des[0] + b3[1] * acc_des[1] + b3[2] * acc_des[2];
}

void attitude_regulator(){

  // Note that q_fb == q from Simulink;

  float q_conj[4] = {-q_d[0], -q_d[1], -q_d[2], q_d[3]};
  float q_e[4];

  q_e[0] = q_fb[3] * q_conj[0] + q_fb[2] * q_conj[1] - q_fb[1] * q_conj[2] + q_fb[0] * q_conj[3];
  q_e[1] = - q_fb[2] * q_conj[0] + q_fb[3] * q_conj[1] + q_fb[0] * q_conj[2] + q_fb[1] * q_conj[3];
  q_e[2] = q_fb[1] * q_conj[0] - q_fb[0] * q_conj[1] + q_fb[3] * q_conj[2] + q_fb[2] * q_conj[3];
  q_e[3] = - q_fb[0] * q_conj[0] - q_fb[1] * q_conj[1] - q_fb[2] * q_conj[2] + q_fb[3] * q_conj[3];

  float norm_q_e = sqrt(q_e[0]^2 + q_e[1]^2 + q_e[2]^2 + q_e[3]^2);

  q_e[0] = q_e[0]/norm_q_e;
  q_e[1] = q_e[1]/norm_q_e;
  q_e[2] = q_e[2]/norm_q_e;
  q_e[3] = q_e[3]/norm_q_e;

  M[0] = -(2 * roll_pitch_PD[0] * (q_e[0] * q_e[3]) + roll_pitch_PD[1] * omega_fb[0]);
  M[1] = -(2 * roll_pitch_PD[0] * (q_e[1] * q_e[3]) + roll_pitch_PD[1] * omega_fb[1]);
  M[2] = -(2 * yaw_PD[0] * (q_e[2] * q_e[3]) + yaw_PD[1] * omega_fb[2]);
}

void mixer_matrix(){

  // Numerical value of mixer_inv;

  thrust[0] = -0.25 * F_B[2] - 0.498 * M[0] + 0.498 * M[1] + 25 * M[2];
  thrust[1] = -0.25 * F_B[2] + 0.498 * M[0] - 0.498 * M[1] + 25 * M[2];
  thrust[2] = -0.25 * F_B[2] + 0.498 * M[0] + 0.498 * M[1] - 25 * M[2];
  thrust[3] = -0.25 * F_B[2] - 0.498 * M[0] - 0.498 * M[1] - 25 * M[2];

  // Maybe we will have to output a percentage of the thrust force compared to the max thrust force possible;
  // We will probably have to output a throttle to the esc (?), therefore we need to study Throttle = f(thrust);
  
}