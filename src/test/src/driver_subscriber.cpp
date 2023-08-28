#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <cmath>
#include <unistd.h>
#include <pigpiod_if2.h>
#include "matrix_operations.h"
#include <test/CustomState.h>

const int IN1 = 6;   // Motor driver input 1
const int IN2 = 5;   // Motor driver input 2
const int PWM = 12; // Motor driver PWM input

const int LED_Y = 17;
const int LED_R = 22;
const int LED_G = 27;

int pi3;

int rotary_encoder_resolution = 100;
float theta_variance=0;
float theta_dot_variance;
float theta_data;
float theta_dot_data;
float theta2_data;

float VOLTAGE = 3.3;

//=========================================================
//Kalman filter (for all system estimation) variables
//State vector
//[[theta1(rad)], [theta1_dot(rad/s)], [theta2(rad)]. [theta2_dot(rad/s)]]
float y[4][1];
float x_data_predict[4][1];
float x_data[4][1];
//Covariance matrix
float P_x_predict[4][4];
float P_x[4][4];
//"A" of the state equation (update freq = 100 Hz)
float A_x[4][4] = {
{1.00210e+00,1.00070e-02,0.00000e+00,3.86060e-05},
{4.20288e-01,1.00210e+00,0.00000e+00,7.65676e-03},
{-1.15751e-03,-3.87467e-06,1.00000e+00,9.74129e-03},
{-2.29569e-01,-1.15751e-03,0.00000e+00,9.48707e-01}
};
//"B" of the state equation (update freq = 100 Hz)
float B_x[4][1] = {{-2.70805e-04},{-5.37090e-02},{1.81472e-03},{3.59797e-01}};
//"C" of the state equation (update freq = 100 Hz)
float C_x[4][4] = {{1, 0, 0, 0},{0, 1, 0, 0},{0, 0, 1, 0},{0, 0, 0, 1}};
//measurement noise
float measure_variance_mat[4][4];
//System noise
float voltage_error = 0.01; //volt
float voltage_variance = voltage_error * voltage_error;

//=========================================================
//Motor control variables
float feedback_rate = 0.01; //sec
float motor_value = 0;
int pwm_duty = 0;
int motor_direction = 1;
float motor_offset = 0.17; //volt

//=========================================================
//Gain vector for the state feedback 
// (R=1000, Q = diag(1, 1, 10, 10), f=100Hz)
float Gain[4] = {29.87522919, 4.59857246, 0.09293, 0.37006248};
// float Gain[4] = {2.987522919, 0.459857246, 0.009293, 0.037006248};
// //(R=1000, Q = diag(10, 10, 1, 1), f=100Hz)
// float Gain[4] = {26.59058346, 4.08393671, 0.02957888, 0.31837809};

// // (R=1000, Q = diag(100, 100, 1, 1), f=100Hz)
// float Gain[4] = {26.61752566, 4.09754450, 0.02956402, 0.31832250};

void Kalman_main(){
     //---------------------------------------
    //Kalman Filter (all system)
    //---------------------------------------                
    //calculate Kalman gain: G = P'C^T(W+CP'C^T)^-1
    float tran_C_x[4][4] = {};
    float P_CT[4][4] = {};
    mat_tran(C_x[0], tran_C_x[0], 4, 4);//C^T
    mat_mul(P_x_predict[0], tran_C_x[0], P_CT[0], 4, 4, 4, 4);//P'C^T
    float G_temp1[4][4] = {};
    mat_mul(C_x[0], P_CT[0], G_temp1[0], 4, 4, 4, 4);//CPC^T
    float G_temp2[4][4] = {};
    mat_add(G_temp1[0], measure_variance_mat[0], G_temp2[0], 4, 4);//W+CP'C^T
    float G_temp2_inv[4][4] = {};
    mat_inv(G_temp2[0], G_temp2_inv[0], 4, 4);//(W+CP'C^T)^-1
    float G[4][4] = {};
    mat_mul(P_CT[0], G_temp2_inv[0], G[0], 4, 4, 4, 4); //P'C^T(W+CP'C^T)^-1
      
    //x_data estimation: x = x'+G(y-Cx')
    float C_x_x[4][1] = {};
    mat_mul(C_x[0], x_data_predict[0], C_x_x[0], 4, 4, 4, 1);//Cx'
    float delta_y[4][1] = {};
    mat_sub(y[0], C_x_x[0], delta_y[0], 4, 1);//y-Cx'
    float delta_x[4][1] = {};
    mat_mul(G[0], delta_y[0], delta_x[0], 4, 4, 4, 1);//G(y-Cx')
    mat_add(x_data_predict[0], delta_x[0], x_data[0], 4, 1);//x'+G(y-Cx')
        
    //calculate covariance matrix: P=(I-GC)P'
    float GC[4][4] = {};
    mat_mul(G[0], C_x[0], GC[0], 4, 4, 4, 4);//GC
    float I4[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    float I4_GC[4][4] = {};
    mat_sub(I4[0], GC[0], I4_GC[0], 4, 4);//I-GC
    mat_mul(I4_GC[0], P_x_predict[0], P_x[0], 4, 4, 4, 4);//(I-GC)P'
}

//=============================================================
//callback関数の宣言
void callback_cvar(const std_msgs::Float64::ConstPtr& c_var_msg)
{
    theta_variance = c_var_msg->data;
    // ROS_INFO("Received theta_variance: %f", theta_variance);
}
void callback_cdotvar(const std_msgs::Float64::ConstPtr& cdot_var_msg)
{
    theta_dot_variance = cdot_var_msg->data;
    // ROS_INFO("Received theta_dot_variance: %f", theta_dot_variance);
}
void callback_imu1(const std_msgs::Float64::ConstPtr& imu_msg1)
{
    theta_data = imu_msg1->data;
    // ROS_INFO("Received theta1: %f", theta_data);
}
void callback_imu2(const std_msgs::Float64::ConstPtr& imu_msg2)
{
    theta_dot_data = imu_msg2->data;
    // ROS_INFO("Received theta1dot: %f", theta_dot_data);
}
void callback_enc(const std_msgs::Float64::ConstPtr& enc_msg)
{
    theta2_data = enc_msg->data;
    //ROS_INFO("Received theta2: %f", theta2_data);
}


int main(int argc, char** argv)
{
    pi3 = pigpio_start(NULL,NULL);

    set_mode(pi3,LED_R,PI_OUTPUT);
    set_mode(pi3,LED_Y,PI_OUTPUT);
    set_mode(pi3,LED_G,PI_OUTPUT);
    gpio_write(pi3,LED_R,1);
    gpio_write(pi3,LED_Y,1);
    gpio_write(pi3,LED_G,1);
    sleep(1);
    gpio_write(pi3,LED_R,0);
    gpio_write(pi3,LED_G,0);
    //-------------------------------------------
    //Kalman filter (all system) variables
    //------------------------------------------- 
    //variable for measurement data
    float pre_theta2=0;
    
    //variables for x prediction
    float Vin;
    float A_x_x[4][1];
    float B_x_Vin[4][1];
   
    //variables for covariance prediction
    float tran_A_x[4][4];
    float AP[4][4]; 
    float APAT[4][4];
    float BBT[4][4];
    float tran_B_x[1][4];
    float BUBT[4][4];

    //-------------------------------------------  
    //Kalman filter (all system) initialization
    //-------------------------------------------
    //initial value of x_data_predict
    for(int i=0; i<4; i++)
    {
        x_data_predict[i][0] = 0;
    }
    
    //initial value of P_x_predict
    for(int i=0; i<4; i++)
    {
        for(int j=0; j<4; j++)
        {
            P_x_predict[i][j] = 0;    
        } 
    }
    for(int i=0; i<4; i++)
    {
        P_x_predict[i][i] = 1e-4;   
    }
    
    //measurement noise matrix
    for(int i=0; i<4; i++)
    {
        for(int j=0; j<4; j++)
        {
            measure_variance_mat[i][j] = 0;    
        } 
    }
    ros::init(argc, argv, "driver_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber var_sub1 = nh.subscribe("c_var_topic", 10, callback_cvar);
    ros::Subscriber var_sub2 = nh.subscribe("cdot_var_topic", 10, callback_cdotvar);
    ros::Subscriber imu_sub1 = nh.subscribe("theta1_topic", 10, callback_imu1);
    ros::Subscriber imu_sub2 = nh.subscribe("theta1dot_temp_topic", 10, callback_imu2);
    ros::Subscriber enc_sub  = nh.subscribe("theta2_topic", 10, callback_enc);

    while(theta_variance==0 || theta_dot_variance==0){
        ros::spinOnce();
    }

    float deg_rad_coeff = (3.14*3.14)/(180*180);
    measure_variance_mat[0][0] = theta_variance * deg_rad_coeff;
    measure_variance_mat[1][1] = theta_dot_variance * deg_rad_coeff;
    float encoder_error = 0.1f*2*3.14f/(4*rotary_encoder_resolution);
    measure_variance_mat[2][2] = encoder_error * encoder_error;
    float encoder_rate_error = encoder_error / feedback_rate;
    measure_variance_mat[3][3] = encoder_rate_error * encoder_rate_error;

    ros::Rate rate(1/feedback_rate);  // サブスクライブの頻度を設定 (100 Hz) 0.01 sec

    int pwm_duty=0;
    set_mode(pi3,IN1, PI_OUTPUT);
    set_mode(pi3,IN2, PI_OUTPUT);
    set_mode(pi3,PWM, PI_OUTPUT);
    set_PWM_frequency(pi3,PWM, 200);
    set_PWM_range(pi3, PWM, 255);
    set_PWM_dutycycle(pi3,PWM, pwm_duty);
    gpio_write(pi3,IN1, 0);
    gpio_write(pi3,IN2, 0);
    gpio_write(pi3,LED_Y,0);
    ROS_INFO("Received initial data");

    int count=0;
    while (ros::ok()) {
        // ros::Time prev_time = ros::Time::now();
        gpio_write(pi3,LED_Y,1);
        gpio_write(pi3,LED_G,0);
        gpio_write(pi3,LED_R,0);
        ros::spinOnce();
        y[0][0] = theta_data * 3.14f/180;
        y[1][0] = theta_dot_data;
        y[2][0] = theta2_data;
        y[3][0] = (y[2][0] - pre_theta2)/feedback_rate; 
        pre_theta2 = y[2][0];
        Kalman_main();
        //------------------------------------------- 
        //drive motor
        //------------------------------------------- 
        //predict the next step data: x'=Ax+Bu
        Vin = motor_value;
        if(motor_value > VOLTAGE)
        {
            Vin = VOLTAGE;
        }
        if(motor_value < -VOLTAGE)
        {
            Vin = -VOLTAGE;    
        }
        mat_mul(A_x[0], x_data[0], A_x_x[0], 4, 4, 4, 1);//Ax_hat
        mat_mul_const(B_x[0], Vin , B_x_Vin[0], 4, 1);//Bu
        mat_add(A_x_x[0], B_x_Vin[0], x_data_predict[0], 4, 1);//Ax+Bu 
        
        //predict covariance matrix: P'=APA^T + BUB^T
        mat_tran(A_x[0], tran_A_x[0], 4, 4);//A^T
        mat_mul(A_x[0], P_x[0], AP[0], 4, 4, 4, 4);//AP
        mat_mul(AP[0], tran_A_x[0], APAT[0], 4, 4, 4, 4);//APA^T
        mat_tran(B_x[0], tran_B_x[0], 4, 1);//B^T
        mat_mul(B_x[0], tran_B_x[0], BBT[0], 4, 1, 1, 4);//BB^T
        mat_mul_const(BBT[0], voltage_variance, BUBT[0], 4, 4);//BUB^T
        mat_add(APAT[0], BUBT[0], P_x_predict[0], 4, 4);//APA^T+BUB^T

        //---------------------------------------
        //Motor control
        //---------------------------------------
        //reset
        motor_value = 0;
        
        //calculate Vin
        for(int i=0; i<4; i++)
        { 
            motor_value += Gain[i] * x_data[i][0];
        }
        if (count%100 == 0){ 
            printf("----------------\n");
            for(int i=0; i<4; i++)
            { 
                printf("state%d: %f\n", i, x_data[i][0]);
            }      
        }
        // if (count%100 == 0){ 
        //   printf("motor_value: %f\n", motor_value);
        // }

        
        //offset
        if(motor_value > 0)
        {
            motor_value += motor_offset ;
        }
        if(motor_value < 0)
        {
            motor_value -= motor_offset;    
        }
        
        //calculate PWM pulse width
        pwm_duty = int( motor_value*255.0f/VOLTAGE );
        if (count%100 == 0){ 
          printf("pwm duty cycle: %d\n", pwm_duty);
        }
        count+=1;
        //drive the motor in forward
        if(pwm_duty>=0)
        {
            //over voltage
            if(pwm_duty>255)
            {
                pwm_duty = 255;    
            }
            //to protect TA7291P
            if(motor_direction == 2)
            {
                gpio_write(pi3,IN1, 0);
                gpio_write(pi3,IN2, 0);
                usleep(100);
            }        
            //forward
            set_PWM_dutycycle(pi3,PWM, pwm_duty);
            gpio_write(pi3,IN1, 1);
            gpio_write(pi3,IN2, 0);
            gpio_write(pi3,LED_Y,0);
            gpio_write(pi3,LED_G,1); 
            motor_direction = 1;
        }      
        //drive the motor in reverse
        else
        {
            //calculate the absolute value
            pwm_duty = -1 * pwm_duty;

            //over voltage
            if(pwm_duty>255)
            {
                pwm_duty = 255;    
            }
            //to protect TA7291P
            if(motor_direction == 1)
            {
                gpio_write(pi3,IN1, 0);
                gpio_write(pi3,IN2, 0);
                usleep(100); //wait 100 usec    
            }
            //reverse
            set_PWM_dutycycle(pi3,PWM, pwm_duty);
            gpio_write(pi3,IN1, 0);
            gpio_write(pi3,IN2, 1);
            gpio_write(pi3,LED_Y,0);
            gpio_write(pi3,LED_R,1);
            motor_direction = 2;
        }

        rate.sleep();

        // 1ループあたりの処理時間を表示
        // ros::Time current_time = ros::Time::now();
        // ros::Duration elapsed_time = current_time - prev_time;
        // count+=1;
        // if (count%100==0){
        //     ROS_INFO("Elapsed time: %.4f seconds", elapsed_time.toSec());
        // }
    }
    gpio_write(pi3,IN1, 0);
    gpio_write(pi3,IN2, 0);
    gpio_write(pi3,LED_R,0);
    gpio_write(pi3,LED_G,0);
    gpio_write(pi3,LED_Y,0);
    pigpio_stop(pi3);

    return 0;
}

