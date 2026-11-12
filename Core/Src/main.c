/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PUTTY_DASHBOARD   1     /* 1 = tampilan tetap di PuTTY, 0 = baris CSV untuk animasi_mpu.py */
#define MPU6050_LOOP_MS   5U    /* periode baca sensor: 5 ms = 200 Hz */
#define I2C_MAX_FAIL      10U   /* gagal baca berturut-turut sebelum I2C di-reset */
#define REINIT_PERIOD_MS  500U  /* jeda antar percobaan Init kalau sensor tidak terdeteksi */

#if PUTTY_DASHBOARD
  #define TX_PERIOD_MS    50U   /* 20 Hz, cukup untuk dibaca mata */
#else
  #define TX_PERIOD_MS    10U   /* 100 Hz */
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
MPU6050_t         MPU6050;
HAL_StatusTypeDef MPU6050_status = HAL_ERROR;
uint32_t          i2c_error_count = 0;   /* total error sejak boot */
static uint8_t    i2c_fail_streak = 0;   /* error berturut-turut */
static uint32_t   t_loop  = 0;
static uint32_t   t_tx    = 0;
static uint32_t   t_retry = 0;
extern USBD_HandleTypeDef hUsbDeviceFS;
static char       line[256];             /* static: buffer harus tetap valid sampai USB selesai mengirim */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
static uint8_t usb_ready(void);
static void    usb_send(int n);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  /* Sensor harus DIAM dan datar sekitar 0,2 detik saat Init, karena sudut awal
   * filter diambil dari accelerometer. */
  MPU6050_status = MPU6050_Init(&hi2c1, &MPU6050);
  t_loop  = HAL_GetTick();
  t_retry = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    /* ---- Sensor belum terdeteksi: coba Init ulang + beri kabar lewat USB ---- */
    if (MPU6050_status != HAL_OK)
    {
      if ((now - t_retry) >= REINIT_PERIOD_MS)
      {
        t_retry = now;
        MPU6050_status = MPU6050_Init(&hi2c1, &MPU6050);

        if (MPU6050_status != HAL_OK && usb_ready())
        {
          /* tanpa koma, supaya tidak dianggap baris data oleh animasi_mpu.py */
          usb_send(snprintf(line, sizeof(line),
                   "MPU6050 tidak terdeteksi - cek kabel SDA SCL VCC GND\r\n"));
        }
        t_loop = HAL_GetTick();
      }
      continue;
    }

    /* ---- Baca sensor + update filter ---- */
    if ((now - t_loop) >= MPU6050_LOOP_MS)
    {
      t_loop = now;

      if (MPU6050_Read_All(&hi2c1, &MPU6050) == HAL_OK)
      {
        MPU6050_Update_Angle(&MPU6050);
        i2c_fail_streak = 0;
      }
      else
      {
        i2c_error_count++;
        if (++i2c_fail_streak >= I2C_MAX_FAIL)
        {
          /* Bus I2C kemungkinan nyangkut (flag BUSY): reset peripheral,
           * lalu paksa jalur Init ulang di atas. */
          HAL_I2C_DeInit(&hi2c1);
          MX_I2C1_Init();
          i2c_fail_streak = 0;
          MPU6050_status = HAL_ERROR;
        }
      }
    }

    /* ---- Kirim lewat USB CDC (tidak blocking) ---- */
    if ((now - t_tx) >= TX_PERIOD_MS && usb_ready())
    {
      t_tx = now;

#if PUTTY_DASHBOARD
      int n = snprintf(line, sizeof(line),
          "\x1b[H"                                        /* kursor ke pojok kiri atas */
          "MPU6050 - sudut (derajat)\x1b[K\r\n"
          "-------------------------\x1b[K\r\n"
          "           Roll(X)  Pitch(Y)\x1b[K\r\n"
          "Kalman  %8.2f  %8.2f\x1b[K\r\n"
          "Comp    %8.2f  %8.2f\x1b[K\r\n"
          "I2C err %8lu\x1b[K\r\n"
          "\x1b[J",                                       /* hapus sisa layar di bawahnya */
          MPU6050.KalmanAngleX, MPU6050.KalmanAngleY,
          MPU6050.CompAngleX,   MPU6050.CompAngleY,
          (unsigned long)i2c_error_count);
#else
      int n = snprintf(line, sizeof(line), "%.2f,%.2f,%.2f,%.2f\r\n",
                       MPU6050.KalmanAngleX, MPU6050.KalmanAngleY,
                       MPU6050.CompAngleX,   MPU6050.CompAngleY);
#endif
      usb_send(n);
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000; //400kHz
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* true kalau USB sudah terkonfigurasi dan transmit sebelumnya sudah selesai */
static uint8_t usb_ready(void)
{
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
  return (hcdc != NULL && hcdc->TxState == 0);
}

/* kirim isi 'line' sepanjang n byte (non-blocking) */
static void usb_send(int n)
{
  if (n <= 0) return;
  if (n >= (int)sizeof(line)) n = (int)sizeof(line) - 1;
  CDC_Transmit_FS((uint8_t *)line, (uint16_t)n);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
