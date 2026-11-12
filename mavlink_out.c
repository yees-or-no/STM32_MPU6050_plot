/**
 * mavlink_out.c - lihat mavlink_out.h
 *
 * Library: https://github.com/mavlink/c_library_v2
 *   Salin folder "common" beserta header di root repo itu ke Core/Inc/mavlink/,
 *   tambahkan Core/Inc/mavlink ke Include paths. Header-only, tidak ada .c tambahan.
 *   Kalau ada error jumlah argumen di mavlink_msg_scaled_imu_pack, versi library-mu
 *   punya/tidak punya field "temperature": tambah/hapus argumen terakhir (0).
 */
#include "mavlink_out.h"
#include "common/mavlink.h"

#ifdef MAVOUT_USE_USB_CDC
#include "usbd_cdc_if.h"
#endif

#define SYS_ID          1
#define COMP_ID         MAV_COMP_ID_IMU
#define STREAM_PERIOD   20U      /* 50 Hz */
#define HEARTBEAT_PERIOD 1000U
#define DEG2RAD         0.01745329252f

static UART_HandleTypeDef *s_uart;
static uint32_t t_stream, t_hb;

static void write_msg(const mavlink_message_t *m)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, m);

#ifdef MAVOUT_USE_USB_CDC
    uint32_t t0 = HAL_GetTick();
    while (CDC_Transmit_FS(buf, len) == USBD_BUSY && (HAL_GetTick() - t0) < 5U) { }
#else
    HAL_UART_Transmit(s_uart, buf, len, 5);
#endif
}

void MavOut_Init(UART_HandleTypeDef *huart)
{
    s_uart = huart;
    t_stream = t_hb = HAL_GetTick();
}

void MavOut_Service(const MPU6050_t *d)
{
    uint32_t now = HAL_GetTick();
    mavlink_message_t msg;

    if ((now - t_hb) >= HEARTBEAT_PERIOD) {
        t_hb = now;
        mavlink_msg_heartbeat_pack(SYS_ID, COMP_ID, &msg, MAV_TYPE_QUADROTOR,
                                   MAV_AUTOPILOT_GENERIC, 0, 0, MAV_STATE_ACTIVE);
        write_msg(&msg);
    }

    if ((now - t_stream) >= STREAM_PERIOD) {
        t_stream = now;

        /* Kalman -> ATTITUDE (radian) */
        mavlink_msg_attitude_pack(SYS_ID, COMP_ID, &msg, now,
                                  d->KalmanAngleX * DEG2RAD,
                                  d->KalmanAngleY * DEG2RAD,
                                  0.0f,
                                  d->Gx * DEG2RAD, d->Gy * DEG2RAD, d->Gz * DEG2RAD);
        write_msg(&msg);

        /* Complementary -> DEBUG_VECT (derajat) */
        mavlink_msg_debug_vect_pack(SYS_ID, COMP_ID, &msg, "COMP",
                                    (uint64_t)now * 1000ULL,
                                    d->CompAngleX, d->CompAngleY, 0.0f);
        write_msg(&msg);

        /* Data sensor: mG dan mrad/s */
        mavlink_msg_scaled_imu_pack(SYS_ID, COMP_ID, &msg, now,
                                    (int16_t)(d->Ax * 1000.0f),
                                    (int16_t)(d->Ay * 1000.0f),
                                    (int16_t)(d->Az * 1000.0f),
                                    (int16_t)(d->Gx * DEG2RAD * 1000.0f),
                                    (int16_t)(d->Gy * DEG2RAD * 1000.0f),
                                    (int16_t)(d->Gz * DEG2RAD * 1000.0f),
                                    0, 0, 0, 0);
        write_msg(&msg);
    }
}