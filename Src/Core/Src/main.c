/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BME280_STM32.h" // BME280 environmental sensor

#include "fatfs_sd.h"
#include <stdbool.h>
#include <stdio.h>   // snprintf for SD card log formatting
#include <string.h>

#include "lwgps/lwgps.h" // GPS parser

#include "max30102_for_stm32_hal.h" // Pulse oximeter

#include "moving_average.h" // Moving average filter

#include "fonts.h"  // OLED fonts
#include "ssd1306.h" // OLED display
#include "test.h"
#include <math.h>  // Real-time distance calculation
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  LOGGER_STATE_IDLE = 0,
  LOGGER_STATE_START_REQUESTED = 1,
  LOGGER_STATE_RUNNING = 2,
  LOGGER_STATE_STOP_REQUESTED = 3
} LoggerState;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.14159265358979323846f
#define EARTH_RADIUS_M 6372797.56085f
#define DEG_TO_RAD (PI / 180.0f)
#define GPS_RX_BUFFER_SIZE 128U
#define RUN_SAMPLE_PERIOD_MS 400U
#define RUN_TARGET_DISTANCE_M 100.0f
#define RUN_MAX_DURATION_MS 240000U
#define HEART_RATE_MIN_INTERVAL_MS 333U
#define BUTTON_DEBOUNCE_MS 10U

#define DEFAULT_WEIGHT_KG 90.0f
#define DEFAULT_AGE_YEARS 24.0f

#define LOG_LINE_BUFFER_SIZE 200U
#define LOG_FILE_NAME_BUFFER_SIZE 50U
#define LCD_TEXT_BUFFER_SIZE 50U

float spo2_percent=0;
uint8_t lcd_spo2_percent=0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
float distance = 0.0;
float previous_distance = 0.0;
float filtered_distance=0.0;
float raw_velocity = 0.0;
float filtered_velocity_stage1 = 0.0;
float filtered_velocity_stage2 = 0.0;

float current_velocity_stage3 = 0.0;
float previous_velocity_stage3 = 0.0;
FilterTypeDef velocity_filter1;
FilterTypeDef velocity_filter2;
FilterTypeDef velocity_filter3;
float current_acceleration = 0.0;
float previous_acceleration = 0.0;

float filtered_acceleration = 0.0;
float start_latitude = 0.0;
float start_longitude = 0.0;
int heart_rate_bpm=0;
int filtered_heart_rate_stage1=0;
int filtered_heart_rate_stage2=0;
int filtered_heart_rate=0;

FilterTypeDef heart_rate_filter1;
FilterTypeDef heart_rate_filter2;
FilterTypeDef heart_rate_filter3;

uint32_t heart_rate_interval_ms = 0;
uint32_t previous_heart_rate_tick = 0;
uint32_t current_heart_rate_tick = 0;

uint32_t previous_start_button_tick = 0;
uint32_t current_start_button_tick = 0;
uint32_t previous_stop_button_tick = 0;
uint32_t current_stop_button_tick = 0;
volatile int logger_state = LOGGER_STATE_IDLE;
uint16_t heart_rate_sample_period_ms = 0;
volatile uint16_t run_sample_period_counter_ms = 0;
volatile uint32_t run_elapsed_time_ms=0;
float sea_level_adjusted_time_ms=0;
float average_speed_mps=0;
float max_speed_mps=0;
uint32_t max_speed_time_ms=0;
int max_speed_heart_rate_bpm=0;
int average_heart_rate_bpm=0;

float max_speed_body_temperature_c=0;
//float weight_kg=100;
//float age_years=0;     //

float weight_kg=DEFAULT_WEIGHT_KG;
float age_years=DEFAULT_AGE_YEARS;
volatile uint8_t start_button_pressed=0;
volatile uint8_t stop_button_pressed=0;
//// printf() function
//int __io_putchar(int ch)
//{
//  uint8_t temp = ch;
//  HAL_UART_Transmit(&huart1, &temp, 1, HAL_MAX_DELAY);
//  return ch;
//}

// MAX30102 plot callback override
void max30102_plot(uint32_t ir_sample, uint32_t red_sample)
{
	if(ir_sample == 0U)
	{
		return;
	}

	float spo2_candidate = 100.0f * ((float)red_sample / (float)ir_sample);
	if(spo2_candidate < 100.0f)
	{
		spo2_percent = spo2_candidate;
	}
}

// MAX30102 instance

max30102_t max30102;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM5_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

// GPS distance calculation
static float CalculateGpsDistance(float latitude1, float longitude1, float latitude2, float longitude2);
static float MinFloat(float a, float b);

static void App_Init(void);
static void App_Run(void);
static void HeartRateFilters_Init(void);
static void RunFilters_Init(void);
static void PulseOximeter_Init(void);
static void PulseOximeter_Process(void);
static void DS18B20_UpdateTemperature(void);
static void Logger_Process(void);
static void Logger_Start(void);
static void Logger_Record(void);
static bool Logger_ShouldStop(void);
static void Logger_Stop(void);
static void Logger_ResetSession(void);
static void Logger_ResetMeasurements(void);
static void Logger_WriteHeader(void);
static void Logger_WriteSample(void);
static void Logger_UpdateMotion(void);
static void Logger_UpdateStatistics(void);
static void Display_Update(void);
static bool GPS_HasFix(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// GPS //////////////////////////////////////////////////////////
lwgps_t gps;

uint8_t gps_rx_buffer[GPS_RX_BUFFER_SIZE];
uint8_t gps_rx_index = 0;
uint8_t gps_rx_byte = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart2)
	{
		if(gps_rx_byte == '\n')
		{
			if(gps_rx_index < sizeof(gps_rx_buffer))
			{
				gps_rx_buffer[gps_rx_index++] = gps_rx_byte;
			}

			lwgps_process(&gps, gps_rx_buffer, gps_rx_index);
			gps_rx_index = 0;
			gps_rx_byte = 0;
		}
		else if(gps_rx_index < sizeof(gps_rx_buffer))
		{
			gps_rx_buffer[gps_rx_index++] = gps_rx_byte;
		}
		else
		{
			gps_rx_index = 0;
		}
		HAL_UART_Receive_IT(&huart2, &gps_rx_byte, 1);
	}
}

// SD card
int log_sample_index=0;
char log_line_buffer[LOG_LINE_BUFFER_SIZE];
char log_file_name[LOG_FILE_NAME_BUFFER_SIZE];
//// OLED text buffer
char lcd_text_buffer[LCD_TEXT_BUFFER_SIZE];
//// BME280 measurements
float ambient_temperature_c, ambient_pressure_pa, ambient_humidity_rh;
//// DS18B20 measurements
float body_temperature_c;

uint8_t lcd_body_temperature_c=0;
uint8_t ds18b20_temp_lsb, ds18b20_temp_msb;
uint16_t ds18b20_raw_temperature;
void microDelay (uint16_t delay)
{
  __HAL_TIM_SET_COUNTER(&htim5, 0);
  while (__HAL_TIM_GET_COUNTER(&htim5) < delay);
}
void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

#define DS18B20_PORT GPIOA
#define DS18B20_PIN GPIO_PIN_1

uint8_t DS18B20_Start (void)
{
	uint8_t response = 0;
	Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);
	HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);
	microDelay(480);

	Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);
	microDelay(80);

	if (!(HAL_GPIO_ReadPin (DS18B20_PORT, DS18B20_PIN))) response = 1;
	else response = 0;

	microDelay(480);

	return response;
}

void DS18B20_Write (uint8_t data)
{
	Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);

	for (int i=0; i<8; i++)
	{

		if ((data & (1<<i))!=0)
		{
			// Write bit 1

			Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);
			HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);
			microDelay(1);

			Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);
			microDelay(50);
		}

		else
		{
			// Write bit 0

			Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);
			HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);
			microDelay(50);

			Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);
		}
	}
}

uint8_t DS18B20_Read (void)
{
	uint8_t value=0;

	Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);

	for (int i=0;i<8;i++)
	{
		Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);

		HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);
		microDelay(1);  // wait for > 1us

		Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);
		if (HAL_GPIO_ReadPin (DS18B20_PORT, DS18B20_PIN))
		{
			value |= 1<<i;  // read = 1
		}
		microDelay(50);
	}
	return value;
}
// FATFS

FATFS fs;  // file system
FIL fil; // File
FILINFO fno;
FRESULT fresult;  // result
UINT br, bw;  // File read/write count
static bool log_file_open = false;

/**** Capacity info *****/
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;


static void App_Init(void)
{
  SSD1306_Init();

  HAL_TIM_Base_Start(&htim5);

  BME280_Config(OSRS_2, OSRS_16, OSRS_1, MODE_NORMAL, T_SB_0p5, IIR_16);
  HAL_Delay(500);

  HAL_TIM_Base_Start_IT(&htim2);

  lwgps_init(&gps);
  HAL_UART_Receive_IT(&huart2, &gps_rx_byte, 1);

  PulseOximeter_Init();

  SSD1306_Clear();
  HAL_Delay(1500);

  start_button_pressed = 0;
  stop_button_pressed = 0;
  logger_state = LOGGER_STATE_IDLE;
  run_sample_period_counter_ms = 0;
  run_elapsed_time_ms = 0;

  HeartRateFilters_Init();
  BME280_Measure();
}

static void App_Run(void)
{
  PulseOximeter_Process();
  DS18B20_UpdateTemperature();
  Logger_Process();
  Display_Update();
}

static void HeartRateFilters_Init(void)
{
  Moving_Average_Init(&heart_rate_filter1);
  Moving_Average_Init(&heart_rate_filter2);
  Moving_Average_Init(&heart_rate_filter3);
}

static void RunFilters_Init(void)
{
  Moving_Average_Init(&velocity_filter1);
  Moving_Average_Init(&velocity_filter2);
  Moving_Average_Init(&velocity_filter3);
}

static void PulseOximeter_Init(void)
{
  max30102_init(&max30102, &hi2c1);
  max30102_reset(&max30102);
  max30102_clear_fifo(&max30102);

  max30102_set_fifo_config(&max30102, max30102_smp_ave_8, 1, 7);
  max30102_set_led_pulse_width(&max30102, max30102_pw_16_bit);
  max30102_set_adc_resolution(&max30102, max30102_adc_2048);
  max30102_set_sampling_rate(&max30102, max30102_sr_800);
  max30102_set_led_current_1(&max30102, 6.2);
  max30102_set_led_current_2(&max30102, 6.2);

  max30102_set_mode(&max30102, max30102_spo2);
  max30102_set_a_full(&max30102, 1);
  max30102_set_die_temp_en(&max30102, 1);
  max30102_set_die_temp_rdy(&max30102, 1);

  uint8_t interrupt_status[2] = {0};
  max30102_read(&max30102, 0x00, interrupt_status, 1);
}

static void PulseOximeter_Process(void)
{
  if(max30102_has_interrupt(&max30102))
  {
    max30102_interrupt_handler(&max30102);
  }
}

static void DS18B20_UpdateTemperature(void)
{
  DS18B20_Start();
  HAL_Delay(1);
  DS18B20_Write(0xCC);
  DS18B20_Write(0x44);

  DS18B20_Start();
  HAL_Delay(1);
  DS18B20_Write(0xCC);
  DS18B20_Write(0xBE);

  ds18b20_temp_lsb = DS18B20_Read();
  ds18b20_temp_msb = DS18B20_Read();
  ds18b20_raw_temperature = (ds18b20_temp_msb << 8) | ds18b20_temp_lsb;
  body_temperature_c = (float)ds18b20_raw_temperature / 16.0f;
}

static void Logger_Process(void)
{
  if(logger_state == LOGGER_STATE_START_REQUESTED)
  {
    Logger_Start();
  }

  if(logger_state == LOGGER_STATE_RUNNING)
  {
    Logger_Record();
  }

  if(Logger_ShouldStop())
  {
    Logger_Stop();
  }
}

static void Logger_Start(void)
{
  BME280_Measure();
  log_sample_index = 0;
  Logger_ResetMeasurements();

  fresult = f_mount(&fs, "", 0);
  if(fresult != FR_OK)
  {
    logger_state = LOGGER_STATE_IDLE;
    return;
  }

  snprintf(log_file_name, sizeof(log_file_name), "%d-%d-%d--%d-%d-%d.txt",
           gps.hours, gps.minutes, gps.seconds, gps.date, gps.month, gps.year);

  fresult = f_open(&fil, log_file_name, FA_OPEN_ALWAYS | FA_WRITE | FA_READ);
  if(fresult != FR_OK)
  {
    f_mount(NULL, "/", 1);
    logger_state = LOGGER_STATE_IDLE;
    return;
  }

  log_file_open = true;
  fresult = f_lseek(&fil, f_size(&fil));
  if(fresult != FR_OK)
  {
    f_close(&fil);
    log_file_open = false;
    f_mount(NULL, "/", 1);
    logger_state = LOGGER_STATE_IDLE;
    return;
  }

  Logger_WriteHeader();

  run_elapsed_time_ms = 0;
  run_sample_period_counter_ms = 0;
  RunFilters_Init();
  logger_state = LOGGER_STATE_RUNNING;
}

static void Logger_Record(void)
{
  if(log_sample_index == 0 && GPS_HasFix())
  {
    start_latitude = gps.latitude;
    start_longitude = gps.longitude;
    log_sample_index = 1;
  }

  distance = CalculateGpsDistance(gps.latitude, gps.longitude,
                             start_latitude, start_longitude);

  if(log_sample_index == 1)
  {
    Logger_WriteSample();
    log_sample_index++;
  }

  if(run_sample_period_counter_ms >= RUN_SAMPLE_PERIOD_MS &&
     distance > previous_distance &&
     log_sample_index > 1)
  {
    Logger_UpdateMotion();
    Logger_WriteSample();
    log_sample_index++;

    previous_distance = distance;
    previous_velocity_stage3 = current_velocity_stage3;
    previous_acceleration = current_acceleration;

    Logger_UpdateStatistics();
    run_sample_period_counter_ms = 0;
  }
}

static bool Logger_ShouldStop(void)
{
  return (logger_state == LOGGER_STATE_STOP_REQUESTED) ||
         (logger_state == LOGGER_STATE_RUNNING &&
          (filtered_distance >= RUN_TARGET_DISTANCE_M ||
           run_elapsed_time_ms > RUN_MAX_DURATION_MS));
}

static void Logger_Stop(void)
{
  if(log_file_open)
  {
    sea_level_adjusted_time_ms =
        run_elapsed_time_ms * (1.03f - (0.03f * expf(-0.000125f * gps.altitude)));

    uint32_t speed_sample_count = (log_sample_index > 1) ? (uint32_t)(log_sample_index - 1) : 1U;
    uint32_t pulse_sample_count = (log_sample_index > 2) ? (uint32_t)(log_sample_index - 2) : 1U;

    average_speed_mps = average_speed_mps / speed_sample_count;
    average_heart_rate_bpm = average_heart_rate_bpm / pulse_sample_count;

    if(average_heart_rate_bpm > (220.0f - age_years) * 0.8f)
    {
      snprintf(log_line_buffer, sizeof(log_line_buffer), "\nAverage Heart Rate High: %d \n", average_heart_rate_bpm);
      f_puts(log_line_buffer, &fil);
    }
    else
    {
      snprintf(log_line_buffer, sizeof(log_line_buffer), "\nAverage Heart Rate Normal: %d \n", average_heart_rate_bpm);
      f_puts(log_line_buffer, &fil);
    }

    snprintf(log_line_buffer, sizeof(log_line_buffer),
             "\nTotal_Time(ms),Sea_Level_Time(ms),Total_Distance(m),Average_Speed(m/s),"
             "Max_Speed(m/s),Max_Speed_Time(ms),Max_Speed_HR,Max_Speed_Body_Temp,SpO2\n");
    f_puts(log_line_buffer, &fil);

    snprintf(log_line_buffer, sizeof(log_line_buffer), "%d,%.0f,%.3f,%.3f,%.3f,%d,%d,%.2f,%.0f\n",
             (int)run_elapsed_time_ms, sea_level_adjusted_time_ms, filtered_distance,
             average_speed_mps, max_speed_mps, (int)max_speed_time_ms, max_speed_heart_rate_bpm,
             max_speed_body_temperature_c, spo2_percent);
    f_puts(log_line_buffer, &fil);

    f_close(&fil);
    log_file_open = false;
  }

  f_mount(NULL, "/", 1);
  Logger_ResetSession();
  SSD1306_Clear();
}

static void Logger_ResetSession(void)
{
  log_sample_index = 0;
  logger_state = LOGGER_STATE_IDLE;
  run_sample_period_counter_ms = 0;
  run_elapsed_time_ms = 0;
  Logger_ResetMeasurements();
}

static void Logger_ResetMeasurements(void)
{
  distance = 0.0f;
  previous_distance = 0.0f;
  filtered_distance = 0.0f;

  raw_velocity = 0.0f;
  filtered_velocity_stage1 = 0.0f;
  filtered_velocity_stage2 = 0.0f;
  current_velocity_stage3 = 0.0f;
  previous_velocity_stage3 = 0.0f;

  current_acceleration = 0.0f;
  previous_acceleration = 0.0f;
  filtered_acceleration = 0.0f;

  start_latitude = 0.0f;
  start_longitude = 0.0f;

  sea_level_adjusted_time_ms = 0.0f;
  average_speed_mps = 0.0f;
  max_speed_mps = 0.0f;
  max_speed_time_ms = 0;
  max_speed_heart_rate_bpm = 0;
  max_speed_body_temperature_c = 0.0f;
  average_heart_rate_bpm = 0;
}

static void Logger_WriteHeader(void)
{
  snprintf(log_line_buffer, sizeof(log_line_buffer), "Ambient_Temp(C),Humidity(RH),Atmospheric_Pressure(kPa),Weight,Age\n");
  f_puts(log_line_buffer, &fil);

  snprintf(log_line_buffer, sizeof(log_line_buffer), "%.0f,%.0f,%.1f,%.0f,%.0f\n",
           ambient_temperature_c, ambient_humidity_rh, ambient_pressure_pa / 1000.0f, weight_kg, age_years);
  f_puts(log_line_buffer, &fil);

  snprintf(log_line_buffer, sizeof(log_line_buffer),
           "\nSample,Time(ms),Latitude,Longitude,Distance(m),Velocity(m/s),Acceleration(m/s^2),"
           "Body_Temp(C),Heart_Rate(bpm)\n");
  f_puts(log_line_buffer, &fil);
}

static void Logger_WriteSample(void)
{
  snprintf(log_line_buffer, sizeof(log_line_buffer), "%d,%d,%.6f,%.6f,%.3f,%.3f,%.3f,%.2f,%d\n",
           log_sample_index, (int)run_elapsed_time_ms, gps.latitude, gps.longitude,
           filtered_distance, filtered_velocity_stage2, filtered_acceleration,
           body_temperature_c, filtered_heart_rate);
  f_puts(log_line_buffer, &fil);
}

static void Logger_UpdateMotion(void)
{
  filtered_distance = (distance + filtered_distance) / 2.0f;

  raw_velocity = (1000.0f * (distance - previous_distance)) / run_sample_period_counter_ms;
  filtered_velocity_stage1 = Moving_Average_Compute(raw_velocity, &velocity_filter1);
  filtered_velocity_stage2 = Moving_Average_Compute(filtered_velocity_stage1, &velocity_filter2);
  current_velocity_stage3 = Moving_Average_Compute(filtered_velocity_stage2, &velocity_filter3);

  current_acceleration = ((current_velocity_stage3 - previous_velocity_stage3) * 1000.0f) /
             run_sample_period_counter_ms;
  filtered_acceleration = (current_acceleration + previous_acceleration) / 2.0f;
}

static void Logger_UpdateStatistics(void)
{
  average_speed_mps = average_speed_mps + filtered_velocity_stage2;
  average_heart_rate_bpm = average_heart_rate_bpm + filtered_heart_rate;

  if(max_speed_mps < filtered_velocity_stage2)
  {
    max_speed_mps = filtered_velocity_stage2;
    max_speed_time_ms = run_elapsed_time_ms;
    max_speed_heart_rate_bpm = filtered_heart_rate;
    max_speed_body_temperature_c = body_temperature_c;
  }
}

static void Display_Update(void)
{
  lcd_spo2_percent = (uint8_t)spo2_percent;
  lcd_body_temperature_c = (uint8_t)body_temperature_c;

  SSD1306_GotoXY(0, 0);
  snprintf(lcd_text_buffer, sizeof(lcd_text_buffer), "HR:%d|O2:%d|BT:%d ",
           filtered_heart_rate, lcd_spo2_percent, lcd_body_temperature_c);
  SSD1306_Puts(lcd_text_buffer, &Font_7x10, 1);

  SSD1306_GotoXY(0, 11);
  SSD1306_Puts("Dist,Speed,Accel", &Font_7x10, 1);

  SSD1306_GotoXY(0, 22);
  snprintf(lcd_text_buffer, sizeof(lcd_text_buffer), "%.2f,%.2f,%.2f",
           filtered_distance, filtered_velocity_stage2, filtered_acceleration);
  SSD1306_Puts(lcd_text_buffer, &Font_7x10, 1);

  SSD1306_GotoXY(0, 44);
  snprintf(lcd_text_buffer, sizeof(lcd_text_buffer), "State:%d;Time:%d",
           logger_state, (int)run_elapsed_time_ms);
  SSD1306_Puts(lcd_text_buffer, &Font_7x10, 1);

  SSD1306_UpdateScreen();
}

static bool GPS_HasFix(void)
{
  return gps.latitude != 0.0f && gps.longitude != 0.0f;
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
  MX_I2C2_Init();
  MX_TIM5_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  App_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    App_Run();

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 100-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 100-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : HEART_RATE_EXTI_Pin */
  GPIO_InitStruct.Pin = HEART_RATE_EXTI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(HEART_RATE_EXTI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DS18B20_Pin */
  GPIO_InitStruct.Pin = DS18B20_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DS18B20_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_CS_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : START_BUTTON_Pin STOP_BUTTON_Pin */
  GPIO_InitStruct.Pin = START_BUTTON_Pin|STOP_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MAX30102_INT_Pin */
  GPIO_InitStruct.Pin = MAX30102_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MAX30102_INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == HEART_RATE_EXTI_Pin)
  {
    current_heart_rate_tick = HAL_GetTick();
    heart_rate_interval_ms = current_heart_rate_tick - previous_heart_rate_tick;

    if(heart_rate_interval_ms > HEART_RATE_MIN_INTERVAL_MS)
    {
      heart_rate_bpm = 60000U / heart_rate_interval_ms;
    }
    previous_heart_rate_tick = current_heart_rate_tick;

    filtered_heart_rate_stage1 = Moving_Average_Compute(heart_rate_bpm, &heart_rate_filter1);
    filtered_heart_rate_stage2 = Moving_Average_Compute(filtered_heart_rate_stage1, &heart_rate_filter2);
    filtered_heart_rate = Moving_Average_Compute(filtered_heart_rate_stage2, &heart_rate_filter3);
  }

  current_start_button_tick = HAL_GetTick();
  if(GPIO_Pin == START_BUTTON_Pin &&
     (current_start_button_tick - previous_start_button_tick) > BUTTON_DEBOUNCE_MS)
  {
    if(logger_state == LOGGER_STATE_IDLE)
    {
      logger_state = LOGGER_STATE_START_REQUESTED;
    }
    previous_start_button_tick = current_start_button_tick;
    start_button_pressed = 1;
  }

  current_stop_button_tick = HAL_GetTick();
  if(GPIO_Pin == STOP_BUTTON_Pin &&
     (current_stop_button_tick - previous_stop_button_tick) > BUTTON_DEBOUNCE_MS)
  {
    logger_state = LOGGER_STATE_STOP_REQUESTED;
    previous_stop_button_tick = current_stop_button_tick;
    stop_button_pressed = 1;
  }
}

static float CalculateGpsDistance(float latitude1, float longitude1, float latitude2, float longitude2)
{
  const double lat1_rad = latitude1 * DEG_TO_RAD;
  const double lon1_rad = longitude1 * DEG_TO_RAD;
  const double lat2_rad = latitude2 * DEG_TO_RAD;
  const double lon2_rad = longitude2 * DEG_TO_RAD;

  const double haversine = pow(sin(0.5 * (lat2_rad - lat1_rad)), 2.0) +
      (cos(lat1_rad) * cos(lat2_rad) * pow(sin(0.5 * (lon2_rad - lon1_rad)), 2.0));
  const double central_angle = 2.0 * asin(MinFloat(1.0f, (float)sqrt(haversine)));

  return (float)(EARTH_RADIUS_M * central_angle);
}

static float MinFloat(float a, float b)
{
  return (a > b) ? b : a;
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  if(htim->Instance == TIM2)
  {
    if(logger_state != LOGGER_STATE_IDLE)
    {
      run_elapsed_time_ms++;
      run_sample_period_counter_ms++;
    }
  }

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

#ifdef  USE_FULL_ASSERT
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
