#include "complementary.h"
#include <math.h>

static uint32_t last_tick = 0;

HAL_StatusTypeDef MPU6050_Calibrate_Gyro(MPU6050_t *dev)
{
    /* Reset nilai offset */
    dev->gyro_offset_x = 0.0f;
    dev->gyro_offset_y = 0.0f;
    dev->gyro_offset_z = 0.0f;

    float sum_gx = 0.0f;
    float sum_gy = 0.0f;
    float sum_gz = 0.0f;

    /* Menjumlahkan pembacaan gyro 3 sumbu untuk jumlah sample yang diinginkan*/
    for (uint16_t i = 0; i < COMP_CALIB_SAMPLES; i++) {
        sum_gx += dev->gyro_x;
        sum_gy += dev->gyro_y;
        sum_gz += dev->gyro_z;

        HAL_Delay(2);
    }

    /*Penentuan offset dengan mencari rata2 dari data gyro yang telah diperoleh*/
    dev->gyro_offset_x = sum_gx / (float)COMP_CALIB_SAMPLES;
    dev->gyro_offset_y = sum_gy / (float)COMP_CALIB_SAMPLES;
    dev->gyro_offset_z = sum_gz / (float)COMP_CALIB_SAMPLES;

    return HAL_OK;
}

void MPU6050_Update_Angle(MPU6050_t *dev)
{
    if (dev == NULL) {
        return;
    }

    uint32_t now = HAL_GetTick();
    float dt = (float)(now - last_tick) / 1000.0f;
    last_tick = now;

    /*Data raw gyro yang dikurangi offset didapat dari callibration function*/
    float raw_gx = dev->gyro_x - dev->gyro_offset_x;
    float raw_gy = dev->gyro_y - dev->gyro_offset_y;
    float raw_gz = dev->gyro_z - dev->gyro_offset_z;

    /*Mapping data mpu menggunakan konvensi NED (North - Easdt - Down)*/
    float ax =  dev->accel_x;
    float ay = -dev->accel_y;
    float az =  dev->accel_z;

    /*menyesuaikan tanda (+/-) gyro terhadap orientasi IMU*/
    float gyro_rate_x =  raw_gx;   
    float gyro_rate_y = -raw_gy;
    float gyro_rate_z = -raw_gz;

    /*Referensi nilai accelerometer sesuai dengan konvensi NED*/
    float acc_roll  = atan2f(ay, az) * RAD_TO_DEG;
    float acc_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;

    /*Inisialisasi nilai*/
    if (!dev->angle_init) {
        dev->roll  = acc_roll;
        dev->pitch = acc_pitch;
        dev->yaw   = 0.0f;
        dev->angle_init = 1;
        return;
    }

    /*Pengaman dt*/
    if (dt <= 0.0f || dt > 0.5f) {
        dt = 0.05f;
    }

    /*Algortima agar konstanta comp_alpha dapat menyesuaikan apabila nilai accelerometer terganggu*/
    float accel_mag = sqrtf(ax * ax + ay * ay + az * az);
    float alpha = COMP_ALPHA;

    /*Ketika nilai total percepatan melebihi 1G dan nilai tollerance (0.2G), complementary filter hanya menggunakan gyro*/
    if (fabsf(accel_mag - 1.0f) > COMP_ACCEL_TOLERANCE) {
        alpha = 1.0f;
    }

    /* Complementary Filter untuk Roll */
    if ((acc_roll < -90.0f && dev->roll > 90.0f) || (acc_roll > 90.0f && dev->roll < -90.0f)) {
        dev->roll = acc_roll;
    } else {
        dev->roll = alpha * (dev->roll + gyro_rate_x * dt) + (1.0f - alpha) * acc_roll;
    }

    /* Complementary Filter untuk Pitch */
    dev->pitch = alpha * (dev->pitch + gyro_rate_y * dt) + (1.0f - alpha) * acc_pitch;

    /*Algoritma agar noise pada gyro sumbu z tidak dianggap sebagai pergerakan*/
    if (fabsf(gyro_rate_z) < COMP_YAW_DEADBAND) {
        gyro_rate_z = 0.0f;
    }

    dev->yaw += gyro_rate_z * dt;

    /*Algoritma untuk menyesuaikan nilai yaw dengan rentang -180 hingga +180 derajat*/
    while (dev->yaw > 180.0f) {
        dev->yaw -= 360.0f;
    }
    while (dev->yaw < -180.0f) {
        dev->yaw += 360.0f;
    }
}