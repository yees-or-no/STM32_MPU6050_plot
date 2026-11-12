/**
 ******************************************************************************
 * @file    mpu6050.h
 * @brief   Driver MPU6050 (I2C, STM32 HAL) dengan complementary filter
 *          dan Kalman filter.
 *
 * Sumber gabungan:
 *  - mpu6050.c/.h dan main.c contoh oleh Ibrahim Cahit Ozdemir (Oktober 2021),
 *    termasuk Kalman filter (berasal dari TKJ Electronics / Kristian Lauszus)
 *  - versi "MPU6050 di main.c": init lengkap, complementary filter
 *
 * Konvensi sudut (sama dengan repo asal):
 *   X = roll  : -90 .. +90 derajat    (rotasi sumbu X, laju = Gx)
 *   Y = pitch : -180 .. +180 derajat  (rotasi sumbu Y, laju = Gy)
 *
 * Catatan: kalibrasi offset gyro sudah dihapus. Bias gyro ditangani oleh
 * Kalman filter (state "bias") dan dikoreksi accelerometer pada
 * complementary filter.
 *
 * Letakkan file ini di Core/Inc.
 ******************************************************************************
 */
#ifndef MPU6050_H
#define MPU6050_H

#include "main.h"      /* menarik header HAL sesuai seri STM32 di project */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================ KONFIGURASI ============================ */
/* Alamat I2C 8-bit (HAL memakai alamat yang sudah digeser 1 bit).
 * AD0 = GND / tidak disambung -> 0x68,  AD0 = VCC -> 0x69 */
#ifndef MPU6050_ADDR
#define MPU6050_ADDR              (0x68 << 1)
#endif

/* Bobot gyro pada complementary filter (0..1). Makin besar = makin
 * percaya gyro, makin kecil = makin percaya accelerometer. */
#ifndef MPU6050_COMP_ALPHA
#define MPU6050_COMP_ALPHA        0.98f
#endif

/* Parameter Kalman (nilai bawaan TKJ Electronics / repo asal) */
#ifndef MPU6050_KALMAN_Q_ANGLE
#define MPU6050_KALMAN_Q_ANGLE    0.001f   /* noise proses sudut        */
#endif
#ifndef MPU6050_KALMAN_Q_BIAS
#define MPU6050_KALMAN_Q_BIAS     0.003f   /* noise proses bias gyro    */
#endif
#ifndef MPU6050_KALMAN_R_MEASURE
#define MPU6050_KALMAN_R_MEASURE  0.03f    /* noise ukur (accelerometer)*/
#endif

/* ============================== TIPE DATA ============================ */
typedef struct {
    float Q_angle;       /* kovarians noise proses sudut            */
    float Q_bias;        /* kovarians noise proses bias gyro        */
    float R_measure;     /* kovarians noise pengukuran              */

    float angle;         /* sudut hasil filter (derajat)            */
    float bias;          /* estimasi bias gyro (derajat/detik)      */
    float P[2][2];       /* matriks kovarians error                 */
} Kalman_t;

typedef struct {
    /* Data mentah */
    int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
    int16_t Gyro_X_RAW,  Gyro_Y_RAW,  Gyro_Z_RAW;

    /* Data terkonversi */
    float Ax, Ay, Az;            /* g                */
    float Gx, Gy, Gz;            /* derajat/detik    */

    /* Sudut hasil filter (derajat) */
    float KalmanAngleX, KalmanAngleY;   /* roll, pitch - Kalman          */
    float CompAngleX,   CompAngleY;     /* roll, pitch - complementary   */

    /* Internal: tiap sensor punya state filter sendiri, jadi aman dipakai
     * untuk lebih dari satu MPU6050 (misalnya satu di I2C1, satu di I2C2). */
    Kalman_t KalmanX, KalmanY;
    uint32_t last_tick;
} MPU6050_t;

/* ============================== FUNGSI =============================== */
/* Semua fungsi I2C mengembalikan HAL_OK bila sukses. */

/* Cek device, reset, konfigurasi (accel +-2 g, gyro +-250 dps,
 * DLPF ~44 Hz, sample rate 200 Hz), lalu baca accelerometer sekali untuk
 * sudut awal filter. Sensor harus DIAM (sekitar 0,2 detik) saat Init. */
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c, MPU6050_t *dev);

HAL_StatusTypeDef MPU6050_Read_Accel(I2C_HandleTypeDef *hi2c, MPU6050_t *dev);
HAL_StatusTypeDef MPU6050_Read_Gyro (I2C_HandleTypeDef *hi2c, MPU6050_t *dev);

/* Baca accel + suhu + gyro sekaligus (14 byte, satu transaksi I2C). */
HAL_StatusTypeDef MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_t *dev);

/* Hitung KalmanAngleX/Y dan CompAngleX/Y dari data terakhir hasil
 * MPU6050_Read_All(). Panggil sekali setiap selesai membaca. */
void MPU6050_Update_Angle(MPU6050_t *dev);

/* Kalman filter 1 dimensi (sudut + bias gyro) */
void  Kalman_Init(Kalman_t *k);
void  Kalman_setAngle(Kalman_t *k, float angle);
float Kalman_getAngle(Kalman_t *k, float newAngle, float newRate, float dt);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H */