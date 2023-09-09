// constants.h

#ifndef CONSTANTS_H
#define CONSTANTS_H

extern int pi;
extern const int ACC_ADDR;
extern const int GYR_ADDR;
extern const int pin1;
extern const int pin2;
extern const int IN1;
extern const int IN2;
extern const int PWM;
extern const int LED_Y;
extern const int LED_R;
extern const int LED_G;


//=========================================================
// Accelerometer and gyro statistical data
extern int sample_num;
extern float meas_interval; // us micro seconds
extern float theta_mean;
extern float theta_variance;
extern float theta_dot_mean;
extern float theta_dot_variance;

//=========================================================
// Rotary encoder variables
extern int rotary_encoder_update_rate; // usec
extern int rotary_encoder_resolution;
extern int encoder_value;
extern int table[16];
extern float pre_theta2;

//=========================================================
// Kalman filter (for angle estimation) variables
// Update rate
extern float theta_update_freq; // Hz
extern float theta_update_interval; 
extern int th1_dura;

// measurement noise
extern float measure_variance_mat[4][4];
// System noise
extern float voltage_error; // volt
extern float voltage_variance;

//=========================================================
// Motor control variables
extern int feedback_rate; // 0.01; //sec
extern float motor_value;
extern int pwm_duty;
extern int motor_direction;
extern float motor_offset; // volt

//=========================================================
// imu data
extern float acc_x;
extern float acc_y;
extern float acc_z;

//=========================================================
// Gain vector for the state feedback
//(R=1000, Q = diag(1, 1, 10, 10), f=100Hz)
// float Gain[4] = {29.87522919, 4.59857246, 0.09293, 0.37006248};

extern float Gain[4];


#endif // CONSTANTS_H