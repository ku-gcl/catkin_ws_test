#ifndef IMU_BMX055_H
#define IMU_BMX055_H
#include "pigpiod_if2.h"

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

    acc_y = ((data[2] & 0xF0) + (data[3] * 256)) / 16;
    if (acc_y > 2047)
    {
        acc_y -= 4096;
    }

    acc_z = ((data[4] & 0xF0) + (data[5] * 256)) / 16;
    if (acc_z > 2047)
    {
        acc_z -= 4096;
    }

    float theta1_deg = atan2(float(acc_z), float(acc_y)) * 57.29578f;

    // convert to G, 1bit sign + 11bit (full scale +/- 2G)
    acc_x = acc_x / 1024;
    acc_y = acc_y / 1024;
    acc_z = acc_z / 1024;
    // ROS_INFO("%f", acc_x);

    return theta1_deg;
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
// Gyroscope (BMX055)
//=========================================================
// get data
float get_gyr_data(int bus)
{
    unsigned char data[2];
    i2c_read_i2c_block_data(pi, bus, 0x02, (char *)data, 2);

    int theta1_dot = data[0] + 256 * data[1];
    if (theta1_dot > 32767)
    {
        theta1_dot -= 65536;
    }
    theta1_dot = -1 * theta1_dot; // !caution!
    // +1000 (deg/sec) / 2^15 = 0.0305176
    return float(theta1_dot) * 0.0305176f;
}

// statistical data of gyro
void gyr_init(int bus)
{
    // initialize Gyro register 0x0F (range)
    // Full scale = +/- 1000 deg/s
    i2c_write_byte_data(pi, bus, 0x0F, 0x01);
    // initialize Gyro register 0x10 (band width)
    // Data rate = 1000 Hz, Filter bandwidth = 116 Hz
    i2c_write_byte_data(pi, bus, 0x10, 0x02);

    // get data
    float theta_dot_array[sample_num];
    for (int i = 0; i < sample_num; i++)
    {
        theta_dot_array[i] = get_gyr_data(bus);
        usleep(meas_interval);
    }

    // calculate mean
    theta_dot_mean = 0;
    for (int i = 0; i < sample_num; i++)
    {
        theta_dot_mean += theta_dot_array[i];
    }
    theta_dot_mean /= sample_num;

    // calculate variance
    float temp;
    theta_dot_variance = 0;
    for (int i = 0; i < sample_num; i++)
    {
        temp = theta_dot_array[i] - theta_dot_mean;
        theta_dot_variance += temp * temp;
    }
    theta_dot_variance /= sample_num;
    return;
}

#endif