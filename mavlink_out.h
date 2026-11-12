/**
 * mavlink_out.h - kirim hasil filter MPU6050 sebagai MAVLink v2 dari STM32.
 *
 * Pesan yang dikirim:
 *   HEARTBEAT   1 Hz
 *   ATTITUDE    50 Hz  -> sudut Kalman (roll, pitch; yaw = 0 karena tanpa magnetometer)
 *   DEBUG_VECT  50 Hz  -> name "COMP", x = roll, y = pitch complementary (derajat)
 *   SCALED_IMU  50 Hz  -> accel (mG) dan gyro (mrad/s)
 *
 * Letakkan di Core/Inc.
 */
#ifndef MAVLINK_OUT_H
#define MAVLINK_OUT_H

#include "main.h"
#include "mpu6050.h"

#ifdef __cplusplus
extern "C" {
#endif

/* huart: UART yang dipakai. Kalau memakai USB CDC, definisikan MAVOUT_USE_USB_CDC
 * (di Project Settings > Defined symbols) dan kirim NULL. */
void MavOut_Init(UART_HandleTypeDef *huart);

/* Panggil di setiap putaran while(1); fungsi ini mengatur laju kirimnya sendiri. */
void MavOut_Service(const MPU6050_t *imu);

#ifdef __cplusplus
}
#endif
#endif
