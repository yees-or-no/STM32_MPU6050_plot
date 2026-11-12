/**
 ******************************************************************************
 * @file    mpu6050.c
 * @brief   Driver MPU6050 (I2C, STM32 HAL) dengan complementary filter
 *          dan Kalman filter. Lihat mpu6050.h untuk sumber dan konvensi.
 *
 * Letakkan file ini di Core/Src.
 ******************************************************************************
 */
#include "mpu6050.h"
#include <math.h>
#include <string.h>

/* ---------- Register MPU6050 ---------- */
#define REG_SMPLRT_DIV      0x19
#define REG_CONFIG          0x1A
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_ACCEL_XOUT_H    0x3B
#define REG_GYRO_XOUT_H     0x43
#define REG_PWR_MGMT_1      0x6B

/* ---------- Skala, sesuai konfigurasi di MPU6050_Init (+-2 g, +-250 dps) ---------- */
#define ACCEL_LSB_PER_G     16384.0f
#define GYRO_LSB_PER_DPS    131.0f

#define I2C_TIMEOUT_MS      100U
#define RAD_TO_DEG          57.2957795f

/* ====================================================================== */
/*                        Fungsi bantu (private)                          */
/* ====================================================================== */
static HAL_StatusTypeDef write_reg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                             &val, 1, I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef read_regs(I2C_HandleTypeDef *hi2c, uint8_t reg,
                                   uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                            buf, len, I2C_TIMEOUT_MS);
}

/* MPU6050 mengirim data 16-bit big-endian (byte tinggi dulu) */
static int16_t to_i16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static void apply_accel(MPU6050_t *dev, const uint8_t *p)
{
    dev->Accel_X_RAW = to_i16(&p[0]);
    dev->Accel_Y_RAW = to_i16(&p[2]);
    dev->Accel_Z_RAW = to_i16(&p[4]);

    dev->Ax = (float)dev->Accel_X_RAW / ACCEL_LSB_PER_G;
    dev->Ay = (float)dev->Accel_Y_RAW / ACCEL_LSB_PER_G;
    dev->Az = (float)dev->Accel_Z_RAW / ACCEL_LSB_PER_G;
}

static void apply_gyro(MPU6050_t *dev, const uint8_t *p)
{
    dev->Gyro_X_RAW = to_i16(&p[0]);
    dev->Gyro_Y_RAW = to_i16(&p[2]);
    dev->Gyro_Z_RAW = to_i16(&p[4]);

    dev->Gx = (float)dev->Gyro_X_RAW / GYRO_LSB_PER_DPS;
    dev->Gy = (float)dev->Gyro_Y_RAW / GYRO_LSB_PER_DPS;
    dev->Gz = (float)dev->Gyro_Z_RAW / GYRO_LSB_PER_DPS;
}

/* Sudut dari accelerometer (rumus repo asal):
 * roll dibatasi +-90 derajat, pitch +-180 derajat.
 * atan2f(y, sqrtf(...)) otomatis aman saat sqrt bernilai 0. */
static void accel_to_angles(const MPU6050_t *dev, float *roll, float *pitch)
{
    *roll  = atan2f(dev->Ay, sqrtf(dev->Ax * dev->Ax + dev->Az * dev->Az)) * RAD_TO_DEG;
    *pitch = atan2f(-dev->Ax, dev->Az) * RAD_TO_DEG;
}

/* ====================================================================== */
/*                           Kalman filter                                */
/*   Algoritma: TKJ Electronics (Kristian Lauszus), via port Ozdemir      */
/* ====================================================================== */
void Kalman_Init(Kalman_t *k)
{
    k->Q_angle   = MPU6050_KALMAN_Q_ANGLE;
    k->Q_bias    = MPU6050_KALMAN_Q_BIAS;
    k->R_measure = MPU6050_KALMAN_R_MEASURE;

    k->angle = 0.0f;
    k->bias  = 0.0f;
    k->P[0][0] = k->P[0][1] = 0.0f;
    k->P[1][0] = k->P[1][1] = 0.0f;
}

void Kalman_setAngle(Kalman_t *k, float angle)
{
    k->angle = angle;
}

float Kalman_getAngle(Kalman_t *k, float newAngle, float newRate, float dt)
{
    /* --- Prediksi --- */
    float rate = newRate - k->bias;
    k->angle += dt * rate;

    k->P[0][0] += dt * (dt * k->P[1][1] - k->P[0][1] - k->P[1][0] + k->Q_angle);
    k->P[0][1] -= dt * k->P[1][1];
    k->P[1][0] -= dt * k->P[1][1];
    k->P[1][1] += k->Q_bias * dt;

    /* --- Koreksi dengan pengukuran accelerometer --- */
    float S = k->P[0][0] + k->R_measure;
    float K[2];
    K[0] = k->P[0][0] / S;
    K[1] = k->P[1][0] / S;

    float y = newAngle - k->angle;
    k->angle += K[0] * y;
    k->bias  += K[1] * y;

    /* --- Perbarui kovarians error ---
     * Langkah ini WAJIB. Tanpanya P terus membesar, gain Kalman mendekati 1,
     * dan filter berubah menjadi sekadar mengikuti accelerometer (berisik). */
    float P00_temp = k->P[0][0];
    float P01_temp = k->P[0][1];

    k->P[0][0] -= K[0] * P00_temp;
    k->P[0][1] -= K[0] * P01_temp;
    k->P[1][0] -= K[1] * P00_temp;
    k->P[1][1] -= K[1] * P01_temp;

    return k->angle;
}

/* ====================================================================== */
/*                            Driver sensor                               */
/* ====================================================================== */
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c, MPU6050_t *dev)
{
    HAL_StatusTypeDef st;

    memset(dev, 0, sizeof(*dev));
    Kalman_Init(&dev->KalmanX);
    Kalman_Init(&dev->KalmanY);

    /* Cek apakah device ada di bus */
    st = HAL_I2C_IsDeviceReady(hi2c, MPU6050_ADDR, 3, I2C_TIMEOUT_MS);
    if (st != HAL_OK) return st;

    /* Reset device */
    st = write_reg(hi2c, REG_PWR_MGMT_1, 0x80);
    if (st != HAL_OK) return st;
    HAL_Delay(100);

    /* Bangunkan dari sleep, clock source = PLL gyro X (lebih stabil) */
    st = write_reg(hi2c, REG_PWR_MGMT_1, 0x01);
    if (st != HAL_OK) return st;
    HAL_Delay(10);

    /* DLPF = 3 -> bandwidth sekitar 44 Hz, gyro output rate 1 kHz */
    st = write_reg(hi2c, REG_CONFIG, 0x03);
    if (st != HAL_OK) return st;

    /* Sample rate = 1 kHz / (1 + 4) = 200 Hz */
    st = write_reg(hi2c, REG_SMPLRT_DIV, 0x04);
    if (st != HAL_OK) return st;

    /* Gyro +-250 dps */
    st = write_reg(hi2c, REG_GYRO_CONFIG, 0x00);
    if (st != HAL_OK) return st;

    /* Accel +-2 g */
    st = write_reg(hi2c, REG_ACCEL_CONFIG, 0x00);
    if (st != HAL_OK) return st;

    /* Beri waktu sensor menghasilkan sampel pertama yang valid, lalu pakai
     * accelerometer untuk sudut awal filter (bukan mulai dari 0). */
    HAL_Delay(50);
    st = MPU6050_Read_Accel(hi2c, dev);
    if (st != HAL_OK) return st;

    float roll, pitch;
    accel_to_angles(dev, &roll, &pitch);

    Kalman_setAngle(&dev->KalmanX, roll);
    Kalman_setAngle(&dev->KalmanY, pitch);
    dev->KalmanAngleX = dev->CompAngleX = roll;
    dev->KalmanAngleY = dev->CompAngleY = pitch;

    dev->last_tick = HAL_GetTick();
    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_Read_Accel(I2C_HandleTypeDef *hi2c, MPU6050_t *dev)
{
    uint8_t buf[6];
    HAL_StatusTypeDef st = read_regs(hi2c, REG_ACCEL_XOUT_H, buf, sizeof(buf));
    if (st != HAL_OK) return st;

    apply_accel(dev, buf);
    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_Read_Gyro(I2C_HandleTypeDef *hi2c, MPU6050_t *dev)
{
    uint8_t buf[6];
    HAL_StatusTypeDef st = read_regs(hi2c, REG_GYRO_XOUT_H, buf, sizeof(buf));
    if (st != HAL_OK) return st;

    apply_gyro(dev, buf);
    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_t *dev)
{
    uint8_t buf[14];
    HAL_StatusTypeDef st = read_regs(hi2c, REG_ACCEL_XOUT_H, buf, sizeof(buf));
    if (st != HAL_OK) return st;

    /* Urutan register: AX AY AZ TEMP GX GY GZ (masing-masing 2 byte).
     * Suhu (buf[6], buf[7]) tidak dipakai. */
    apply_accel(dev, &buf[0]);
    apply_gyro (dev, &buf[8]);
    return HAL_OK;
}

void MPU6050_Update_Angle(MPU6050_t *dev)
{
    uint32_t now = HAL_GetTick();
    float dt = (float)(now - dev->last_tick) * 0.001f;

    if (dt <= 0.0f) return;   /* dua pembacaan dalam 1 ms: belum ada waktu berlalu */
    dev->last_tick = now;

    float roll, pitch;
    accel_to_angles(dev, &roll, &pitch);

    float gx_rate = dev->Gx;  /* salinan lokal; nilai Gx yang dilaporkan tidak diubah */
    float gy_rate = dev->Gy;

    /* --- Pitch (sumbu Y) --- */
    /* Saat pitch melewati +-180, accelerometer melompat dari +180 ke -180
     * (atau sebaliknya). Tanpa penanganan ini filter akan berputar balik. */
    if ((pitch < -90.0f && dev->KalmanAngleY >  90.0f) ||
        (pitch >  90.0f && dev->KalmanAngleY < -90.0f)) {
        Kalman_setAngle(&dev->KalmanY, pitch);
        dev->KalmanAngleY = pitch;
        dev->CompAngleY   = pitch;
    } else {
        dev->KalmanAngleY = Kalman_getAngle(&dev->KalmanY, pitch, gy_rate, dt);
    }
    dev->CompAngleY = MPU6050_COMP_ALPHA * (dev->CompAngleY + gy_rate * dt)
                    + (1.0f - MPU6050_COMP_ALPHA) * pitch;

    /* Saat |pitch| > 90 sensor "terbalik", sehingga arah putaran roll
     * (dibatasi +-90) berlawanan dengan gyro X: balik tandanya. */
    if (fabsf(dev->KalmanAngleY) > 90.0f) gx_rate = -gx_rate;

    /* --- Roll (sumbu X) --- */
    dev->KalmanAngleX = Kalman_getAngle(&dev->KalmanX, roll, gx_rate, dt);
    dev->CompAngleX   = MPU6050_COMP_ALPHA * (dev->CompAngleX + gx_rate * dt)
                      + (1.0f - MPU6050_COMP_ALPHA) * roll;
}