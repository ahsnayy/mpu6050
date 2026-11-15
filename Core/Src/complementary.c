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

    /* Membaca dan menjumlahkan sampel gyro baru sebanyak COMP_CALIB_SAMPLES */
    for (uint16_t i = 0; i < COMP_CALIB_SAMPLES; i++) {
        if (MPU6050_ReadAll(dev) != HAL_OK) {
            return HAL_ERROR;
        }

        sum_gx += dev->gyro_x;
        sum_gy += dev->gyro_y;
        sum_gz += dev->gyro_z;

        HAL_Delay(2);
    }

    /* Penentuan offset rata-rata */
    dev->gyro_offset_x = sum_gx / (float)COMP_CALIB_SAMPLES;
    dev->gyro_offset_y = sum_gy / (float)COMP_CALIB_SAMPLES;
    dev->gyro_offset_z = sum_gz / (float)COMP_CALIB_SAMPLES;

    return HAL_OK;
}

void MPU6050_Update_Angle(MPU6050_t *dev)
{
    uint32_t now = HAL_GetTick();
    uint32_t elapsed_ms = now - last_tick;

    if (elapsed_ms == 0) {
        return; /* Tunggu minimal 1 ms agar dt akurat */
    }

    last_tick = now;
    float dt = (float)elapsed_ms / 1000.0f;

    /* Pengaman jika terjadi delay besar (misal saat startup atau breakpoint) */
    if (dt > 0.5f) {
        dt = 0.05f;
    }

    /* Data raw gyro dikurangi offset hasil kalibrasi */
    float raw_gx = dev->gyro_x - dev->gyro_offset_x;
    float raw_gy = dev->gyro_y - dev->gyro_offset_y;
    float raw_gz = dev->gyro_z - dev->gyro_offset_z;

    /* Mapping sumbu agar 100% sinkron dengan kalman.c */
    float ax =  dev->accel_x;
    float ay = -dev->accel_y;
    float az =  dev->accel_z;

    float gyro_rate_x = -raw_gx;   /* +Roll  = right wing down (sama dengan KalmanX) */
    float gyro_rate_y =  raw_gy;   /* +Pitch = nose up         (sama dengan KalmanY) */
    float gyro_rate_z = -raw_gz;   /* +Yaw   = nose right (CW) (sama dengan KalmanZ) */

    /* Referensi sudut accelerometer */
    float acc_roll  = atan2f(ay, az) * RAD_TO_DEG;
    float acc_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;

    /* Inisialisasi sudut awal saat startup */
    if (!dev->angle_init) {
        dev->roll  = acc_roll;
        dev->pitch = acc_pitch;
        dev->yaw   = 0.0f;
        dev->angle_init = 1;
        return;
    }

    /* Adaptive alpha untuk menolak percepatan linear */
    float accel_mag = sqrtf(ax * ax + ay * ay + az * az);
    float alpha = COMP_ALPHA;

    if (fabsf(accel_mag - 1.0f) > COMP_ACCEL_TOLERANCE) {
        alpha = 0.995f;
    }

    /* Complementary Filter untuk Roll */
    if ((acc_roll < -90.0f && dev->roll > 90.0f) || (acc_roll > 90.0f && dev->roll < -90.0f)) {
        dev->roll = acc_roll;
    } else {
        dev->roll = alpha * (dev->roll + gyro_rate_x * dt) + (1.0f - alpha) * acc_roll;
    }

    /* Complementary Filter untuk Pitch */
    dev->pitch = alpha * (dev->pitch + gyro_rate_y * dt) + (1.0f - alpha) * acc_pitch;

    /* Stabilized Z-Axis Yaw Tracking */
    if (fabsf(gyro_rate_z) < COMP_YAW_DEADBAND) {
        gyro_rate_z = 0.0f;
    }

    dev->yaw += gyro_rate_z * dt;

    while (dev->yaw > 180.0f) {
        dev->yaw -= 360.0f;
    }
    while (dev->yaw < -180.0f) {
        dev->yaw += 360.0f;
    }
}