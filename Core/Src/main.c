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
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	int16_t raw_x;
	float filtered_x;
} AccelerometerData;

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temp;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} IMU_Sample_t;
/* USER CODE END PTD */

/* USER CODE BEGIN PV */
#define SNAPSHOT_SAMPLES 200
#define MPU6050_ADDR (0x68 << 1)
#define MPU6050_REG_DATA_START 0x3B

// The 2-second buffer
volatile IMU_Sample_t imu_buffer[SNAPSHOT_SAMPLES];

// State tracking variables
volatile uint16_t sample_index = 0;
volatile uint8_t snapshot_ready = 0;
volatile uint8_t dma_busy = 0;

typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} IMU_Physical_t;

// The final 2-second processed buffer
IMU_Physical_t processed_buffer[SNAPSHOT_SAMPLES];

/* USER CODE END PV */


/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
AccelerometerData sensor_data;
uint8_t dataBuffer[2];
char uart_buffer[50];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float apply_ema_filter(int16_t raw_x) {
    float alpha = 0.8f;
    static float previous_ema = 0.0f;
    float current_raw_float = (float)raw_x;
    float filtered_x = (alpha * current_raw_float) + ((1.0f - alpha) * previous_ema);
    previous_ema = filtered_x;
    return filtered_x;
}

int16_t swap_bytes(int16_t val) {
    // Cast to unsigned to prevent sign-extension issues during the shift
    return (int16_t)( ((uint16_t)val << 8) | ((uint16_t)val >> 8) );
}

int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  // Wake up the MPU6050 by writing 0 to the Power Management register (0x6B)
  // 1. Turn off printf buffering so it sends data instantly!
  	  setvbuf(stdout, NULL, _IONBF, 0);

    // 2. Print a boot message to prove the USB cable and COM port work
  	  printf("STM32 is awake and booting!\r\n");

      uint8_t data;

      // 1. Wake Up the Sensor (PWR_MGMT_1 = 0x00)
      data = 0x00;
      HAL_I2C_Mem_Write(&hi2c1, (0x68 << 1), 0x6B, 1, &data, 1, HAL_MAX_DELAY);

      // 2. Enable the Low-Pass Filter (CONFIG = 0x03)
      data = 0x03;
      HAL_I2C_Mem_Write(&hi2c1, (0x68 << 1), 0x1A, 1, &data, 1, HAL_MAX_DELAY);

      // 3. Set the 100Hz Sample Rate (SMPLRT_DIV = 0x09)  1000 / (1 + 9) = 100
      data = 0x09;
      HAL_I2C_Mem_Write(&hi2c1, (0x68 << 1), 0x19, 1, &data, 1, HAL_MAX_DELAY);

      // 4. Configure the Interrupt Pin (INT_PIN_CFG = 0x00)
      data = 0x10;
      HAL_I2C_Mem_Write(&hi2c1, (0x68 << 1), 0x37, 1, &data, 1, HAL_MAX_DELAY);

      // 5. Enable the Data Ready Interrupt (INT_ENABLE = 0x01)
      data = 0x01;
      HAL_I2C_Mem_Write(&hi2c1, (0x68 << 1), 0x38, 1, &data, 1, HAL_MAX_DELAY);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

      // Create a variable to track how many times the loop runs
        uint32_t free_cpu_counter = 0;

        while (1)
        {
            // 1. Let the main loop spin as fast as it possibly can
            free_cpu_counter++;

            if (snapshot_ready) {

                // 2. Process the 200 samples (Your exact Day 4 code)
                for (int i = 0; i < SNAPSHOT_SAMPLES; i++) {
                    int16_t raw_ax = swap_bytes(imu_buffer[i].accel_x);
                    int16_t raw_ay = swap_bytes(imu_buffer[i].accel_y);
                    int16_t raw_az = swap_bytes(imu_buffer[i].accel_z);

                    int16_t raw_gx = swap_bytes(imu_buffer[i].gyro_x);
                    int16_t raw_gy = swap_bytes(imu_buffer[i].gyro_y);
                    int16_t raw_gz = swap_bytes(imu_buffer[i].gyro_z);

                    processed_buffer[i].accel_x = raw_ax / 16384.0f;
                    processed_buffer[i].accel_y = raw_ay / 16384.0f;
                    processed_buffer[i].accel_z = raw_az / 16384.0f;

                    processed_buffer[i].gyro_x = raw_gx / 131.0f;
                    processed_buffer[i].gyro_y = raw_gy / 131.0f;
                    processed_buffer[i].gyro_z = raw_gz / 131.0f;
                }

                // 3. Print a quick sanity check of the first sample's Z-axis
                // 3. Print a quick sanity check
                                if (snapshot_ready) {
                                	printf("$");
                                	for (int i = 0; i < SNAPSHOT_SAMPLES; i++){
                                		printf("%.3f\r\n", processed_buffer[i].accel_x);

                                		HAL_Delay(10);
                                	}


                                	snapshot_ready = 0;
                                }
                            } // Closes: if (snapshot_ready)
                        } // Closes: while (1)
                  /* USER CODE END WHILE */

                  /* USER CODE BEGIN 3 */

                } // Closes: int main(void)
            /* USER CODE END WHILE */
 /* while (1)
  {
	        if (snapshot_ready) {
	        	for (int i = 0; i < SNAPSHOT_SAMPLES; i++) {
					int16_t raw_ax = swap_bytes(imu_buffer[i].accel_x);
					int16_t raw_ay = swap_bytes(imu_buffer[i].accel_y);
					int16_t raw_az = swap_bytes(imu_buffer[i].accel_z);

					int16_t raw_gx = swap_bytes(imu_buffer[i].gyro_x);
					int16_t raw_gy = swap_bytes(imu_buffer[i].gyro_y);
					int16_t raw_gz = swap_bytes(imu_buffer[i].gyro_z);

								  // 3. Convert to physical units (g's and deg/s)
					processed_buffer[i].accel_x = raw_ax / 16384.0f;
					processed_buffer[i].accel_y = raw_ay / 16384.0f;
					processed_buffer[i].accel_z = raw_az / 16384.0f;

					processed_buffer[i].gyro_x = raw_gx / 131.0f;
					processed_buffer[i].gyro_y = raw_gy / 131.0f;
					processed_buffer[i].gyro_z = raw_gz / 131.0f;
	        	          }
	            snapshot_ready = 0; // Reset the flag
*/
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // Read High and Low bytes of X-axis
	/*  HAL_I2C_Mem_Read(&hi2c1, (0x68 << 1), 0x3B, 1, dataBuffer, 2, 1000);

	  // Assemble 16-bit integer
	  sensor_data.raw_x = (int16_t)(dataBuffer[0] << 8 | dataBuffer[1]);

	  // Apply EMA Filter
	  sensor_data.filtered_x = apply_ema_filter(sensor_data.raw_x);

	  // Format and send to Serial Monitor
	  sprintf(uart_buffer, "%d, %.2f\r\n", sensor_data.raw_x, sensor_data.filtered_x);
	  HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);

	  HAL_Delay(10); */

  /* USER CODE END 3 */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// 1. This fires when the MPU6050 INT pin hits the STM32
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_0) {
       // printf("E\r\n"); // <-- Added \r\n

        if (!snapshot_ready && !dma_busy) {
            dma_busy = 1;

            if (HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_ADDR, MPU6050_REG_DATA_START,
                                     I2C_MEMADD_SIZE_8BIT, (uint8_t*)&imu_buffer[sample_index], 14) != HAL_OK) {
                dma_busy = 0;
                //printf("F\r\n"); // <-- Added \r\n
            }
        }
    }
}

// 2. This fires automatically when the DMA finishes
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {
       // printf("D\r\n"); // <-- Added \r\n

        dma_busy = 0;
        sample_index++;

        if (sample_index >= SNAPSHOT_SAMPLES) {
            snapshot_ready = 1;
            sample_index = 0;
        }
    }
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
