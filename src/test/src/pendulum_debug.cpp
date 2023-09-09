#include <ros/ros.h>
#include "pigpiod_if2.h"
#include <cmath>
#include <unistd.h>
#include <iostream>
#include <thread>
#include <chrono>
// for detect ctrol+c
#include <csignal>
#include <cstdlib>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include "matrix_operations.h"
#include "signal_handler.h"
// #include "constants.h"

std::thread thread1;
std::thread thread2;
int enc_syn = 1;
int update_theta_syn_flag = 1;


//=========================================================
// Port Setting
int pi;
const int ACC_ADDR = 0x19;
const int GYR_ADDR = 0x69;
const int pin1 = 24; // to A
const int pin2 = 23; // to B

const int IN1 = 6;  // Motor driver input 1
const int IN2 = 5;  // Motor driver input 2
const int PWM = 12; // Motor driver PWM input

const int LED_Y = 17;
const int LED_R = 22;
const int LED_G = 27;

//=========================================================
// Accelerometer and gyro statistical data
int sample_num = 100;
float meas_interval = 10000; // us micro seconds
float theta_mean;
float theta_variance;
float theta_dot_mean;
float theta_dot_variance;

//=========================================================
// Rotary encoder variables
int rotary_encoder_update_rate = 25; // usec
int rotary_encoder_resolution = 100;
int encoder_value = 0;
int table[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};
float pre_theta2 = 0;

//=========================================================
// Kalman filter (for angle estimation) variables
// Update rate
float theta_update_freq = 400; // Hz
float theta_update_interval = 1.0f / theta_update_freq;
int th1_dura = 1000 * 1.0f / theta_update_freq;


//=========================================================
// Motor control variables
int feedback_rate = 10000; // 0.01; //sec
float motor_value = 0;
int pwm_duty = 0;
int motor_direction = 1;
float motor_offset = 0.17; // volt

//=========================================================
// imu data
float acc_x = 0.0;
float acc_y = 0.0;
float acc_z = 0.0;

//=========================================================
// Accelerometer (BMX055)
//=========================================================
// get data
float get_acc_data(int bus)
{
    unsigned char data[6];
    // i2c_read_i2c_block_data(pi, bus, 0x04, (char *)data, 4);
    i2c_read_i2c_block_data(pi, bus, 0x02, (char *)data, 6);

    acc_x = ((data[0] & 0xF0) + (data[1] * 256)) / 16;
    if (acc_x > 2047)
    {
        acc_x -= 4096;
    }
    // convert to G, 1bit sign + 11bit (full scale +/- 2G)
    acc_x = acc_x / 1024;
    // ROS_INFO("%f", acc_x);

    return acc_x;
}

// statistical data of accelerometer
void acc_init(int bus)
{
    // initialize ACC register 0x0F (range)
    // Full scale = +/- 2 G
    i2c_write_byte_data(pi, bus, 0x0F, 0x03);
    // initialize ACC register 0x10 (band width)
    // Filter bandwidth = 1000 Hz
    i2c_write_byte_data(pi, bus, 0x10, 0x0F);

    // get data
    float theta_array[sample_num];
    for (int i = 0; i < sample_num; i++)
    {
        theta_array[i] = get_acc_data(bus);
        usleep(meas_interval);
    }

    // calculate mean
    theta_mean = 0;
    for (int i = 0; i < sample_num; i++)
    {
        theta_mean += theta_array[i];
    }
    theta_mean /= sample_num;

    // calculate variance
    float temp;
    theta_variance = 0;
    for (int i = 0; i < sample_num; i++)
    {
        temp = theta_array[i] - theta_mean;
        theta_variance += temp * temp;
    }
    theta_variance /= sample_num;
    return;
}


//=========================================================
// Kalman filter for "theta" & "theta_dot_bias"
// It takes 650 usec. (NUCLEO-F401RE 84MHz, BMX055)
//=========================================================
void update_theta(int bus_acc, int bus_gyr)
{
    if (update_theta_syn_flag == 0)
    {
        return;
    }
    // detach the rotary encoder polling
    enc_syn = 0;

    // measurement data
    float y = get_acc_data(bus_acc); // degree
    // ROS_INFO("%f", acc_x);
    // printf("%f", y)

    // attach a timer for the rotary encoder (40 kHz)
    enc_syn = 1;
    std::chrono::milliseconds dura2(th1_dura);
    std::this_thread::sleep_for(dura2);
}


//=========================================================
// Main
//=========================================================
int main(int argc, char **argv)
{
    // Ctrl+Cによる中断をキャッチするためのシグナルハンドラを設定
    std::signal(SIGINT, signalHandler);
    // Ctrl+Z
    std::signal(SIGTSTP, signalHandler);

    ros::init(argc, argv, "imu_publisher");
    ros::NodeHandle nh;
    ros::Publisher imu_data_raw_pub = nh.advertise<sensor_msgs::Imu>("imu/data_raw", 10);
    ros::Publisher imu_data = nh.advertise<sensor_msgs::Imu>("imu/data", 10); // Kalman filter
    ros::Publisher mag_data_raw_pub = nh.advertise<sensor_msgs::MagneticField>("imu/mag", 10);

    ros::Rate rate(10);
    // ros::Publisher var_pub1 = nh.advertise<std_msgs::Float64>("c_var_topic", 10);
    // ros::Publisher var_pub2 = nh.advertise<std_msgs::Float64>("cdot_var_topic", 10);
    // ros::Publisher imu_pub1 = nh.advertise<std_msgs::Float64>("theta1_topic", 10);
    // ros::Publisher imu_pub2 = nh.advertise<std_msgs::Float64>("theta1dot_temp_topic", 10);
    // ros::Rate rate(400);  // パブリッシュの頻度を設定 (400 Hz)

    pi = pigpio_start(NULL, NULL);
    int bus_acc = i2c_open(pi, 1, ACC_ADDR, 0);
    int bus_gyr = i2c_open(pi, 1, GYR_ADDR, 0);
    set_mode(pi, LED_R, PI_OUTPUT);
    set_mode(pi, LED_Y, PI_OUTPUT);
    set_mode(pi, LED_G, PI_OUTPUT);
    set_mode(pi, pin1, PI_INPUT);
    set_mode(pi, pin2, PI_INPUT);
    set_mode(pi, IN1, PI_OUTPUT);
    set_mode(pi, IN2, PI_OUTPUT);
    set_mode(pi, PWM, PI_OUTPUT);

    //-------------------------------------------
    // LED
    //-------------------------------------------
    gpio_write(pi, LED_R, 1);
    gpio_write(pi, LED_Y, 1);
    gpio_write(pi, LED_G, 1);
    sleep(1);
    gpio_write(pi, LED_R, 0);
    gpio_write(pi, LED_G, 0);

    //-------------------------------------------
    // Accelerometer & Gyro initialization
    //-------------------------------------------
    acc_init(bus_acc);

    //-------------------------------------------
    // Rotary encoder initialization
    //-------------------------------------------
    encoder_value = 0;
    //-------------------------------------------
    // Motor driver intialization
    //-------------------------------------------
    gpio_write(pi, IN1, 0);
    gpio_write(pi, IN2, 0);
    set_PWM_frequency(pi, PWM, 10000);
    set_PWM_range(pi, PWM, 100);
    set_PWM_dutycycle(pi, PWM, 0);

    //-------------------------------------------
    // Timer
    //-------------------------------------------
    // thread1 = std::thread(rotary_encoder);
    thread2 = std::thread(update_theta, bus_acc, bus_gyr);
    // thread1.join();
    thread2.join();

    //-------------------------------------------
    // initialization done
    //-------------------------------------------
    gpio_write(pi, LED_Y, 0);

    //===========================================
    // Main loop
    // it takes 700 usec (calculation)
    //===========================================
    float start_time;
    float end_time;
    float elapsed_time;
    int count = 0;

    while (1)
    {
        // stop theta update process
        update_theta_syn_flag = 0;

        // turn off LEDs
        gpio_write(pi, LED_R, 0);
        gpio_write(pi, LED_G, 0);

        // start the angle update process
        update_theta_syn_flag = 1;

        // publish imu data
        count += 1;
        if (count % 10 == 0)
        {
            ros::Time current_time = ros::Time::now();
            sensor_msgs::Imu imu_msg;
            imu_msg.header.frame_id = "map";
            imu_msg.header.stamp = current_time;
            // 線形加速度データをセット

            imu_msg.linear_acceleration.x = acc_x; // X軸の線形加速度
            // ROS_INFO("%f", acc_x);
            imu_msg.linear_acceleration.y = acc_y; // Y軸の線形加速度
            imu_msg.linear_acceleration.z = acc_z; // Z軸の線形加速度（通常は重力）
            // クオータニオンデータをセット（オイラー角をクオータニオンに変換することもできます）
            // imu_msg.orientation.x = quat_x;
            // imu_msg.orientation.y = quat_y;
            // imu_msg.orientation.z = quat_z;
            // imu_msg.orientation.w = quat_w;
            imu_msg.orientation.x = 0.0;
            imu_msg.orientation.y = 0.0;
            imu_msg.orientation.z = 0.0;
            imu_msg.orientation.w = 0.0;
            // 角速度データをセット
            imu_msg.angular_velocity.x = 0.0; // X軸の角速度
            imu_msg.angular_velocity.y = 0.0; // Y軸の角速度
            imu_msg.angular_velocity.z = 0.0; // Z軸の角速度

            imu_data_raw_pub.publish(imu_msg);
            ros::spinOnce();
        }

        // wait
        std::chrono::microseconds dura3(feedback_rate);
        std::this_thread::sleep_for(dura3);
    }
    //======10000//=====================================
    // Main loop (end)
    //===========================================
    return 0;
}
