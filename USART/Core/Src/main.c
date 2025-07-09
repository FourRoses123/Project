#include "main.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "fatfs.h"
#include "gpio.h"
#include "tim.h"
#include "sdcard.h"
#include "user_spi.h"
#include "sdconfig.h"
#include "user_usart.h"

void SystemClock_Config(void);
void wipe_cache(void);
void Resettime(void);

uint8_t rx_buffer[BUF_SIZE];
volatile uint16_t rx_length = 0;
volatile uint16_t data_ready = 0;
FATFS fs;
FIL fil;
UINT bw;
uint8_t res = 0;
SD_Config current_config;
volatile uint8_t open = 0;
volatile uint8_t timing = 0;
volatile uint32_t timer_ms_count = 0;
volatile uint32_t last_trigger_time = 0;
char tx_buffer_ascii[1024]; //测试用，之后删
volatile uint32_t last_send_tick = 0;
volatile uint32_t last_send_time = 0;
const uint32_t interval = 1000;
volatile uint32_t settime = 0;
int retry = 127;
volatile uint16_t start = 0;
static uint16_t rollover_action_done = 0;

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  MX_TIM6_Init();
  MX_TIM2_Init();
  MX_TIM7_Init();
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);
  MX_SPI1_Init();
  UART_Queue_Init();
  if(SD_Init() == SD_OK)
  {
	res=f_mount(&fs,"0:",1);
	if(res != FR_OK)
		Error_Handler();
	if(file_exists(CONFIG_FILE) != FR_OK)
		create_default_config();
	int log_num = get_next_log_number();
	snprintf(current_log_file, sizeof(current_log_file), "%s%05d.txt", LOG_PREFIX, log_num);
	if(f_open(&fil, current_log_file, FA_CREATE_NEW | FA_WRITE) == FR_OK)
	{
		open = 1;
	}
	current_config = load_and_apply_config();
	HAL_UART_Receive_DMA(&huart1, rx_buffer, sizeof(rx_buffer));
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);
  }
  wipe_cache();

  while (1)
  {
	  Resettime();
	  if(data_ready)
	  {
		  data_ready = 0;
		  uint8_t time[14];
		  Timing_Status res = GPS_message_process(time);
		  rp = wp;
		  if(res == Timing_OK)
		  {
			  time_t GPStime = standard_to_stamp(time);
			  set_base_time(GPStime);
			  settime = HAL_GetTick();
			  timing = 6;
			  Toggle_G();
		  }
	  }
	  ASIC_message_read();
	  uint32_t sendtime = HAL_GetTick();
	  if(sendtime - last_send_time >= interval)
	  {
		  last_send_time = sendtime;
		  Data_Consolidation();
		  if(open == 1)
		  {
			  int offset = 0;
			  for (int i = 0; i < tx_length; i++) //测试用，之后删
				  offset += snprintf(tx_buffer_ascii + offset, sizeof(tx_buffer_ascii) - offset, "%02X ", tx_buffer[i]);
			  snprintf(tx_buffer_ascii + offset, sizeof(tx_buffer_ascii) - offset, "\r\n");
			  if(f_write(&fil, tx_buffer_ascii, strlen(tx_buffer_ascii), &bw) == FR_OK)
			  {
				  f_sync(&fil);
			  }
			  /*if(f_write(&fil, tx_buffer, tx_length, &bw) == FR_OK)
				  f_sync(&fil);*/
		  }
	  }
  }
}

void Resettime(void)
{
	uint8_t resetbit = (high_counter >> 32) & 0x01;
	if (resetbit == 0)
	{
		if (rollover_action_done == 0)
		{
		  Toggle_R();
		  start = 1;
		  rollover_action_done = 1;
		}
	}
	else
	{
		rollover_action_done = 0;
	}
	if (start == 1)
	{
		start = 0;
		wrap_count++;
	}
}

void wipe_cache(void)//清除非本次上电的ASIC缓存
{
	  while(retry >=0 && ReadResult() != ASIC_EMPTY)
	  {
		  retry--;
	  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == PPS_Pin)
	{
		uint32_t now = HAL_GetTick();
		if(timing == 6 && now - settime < 2000)
		{
			timing = 7;
			time_t time = base_timestamp + ONESEC;
			set_base_time(time);
			last_trigger_time = HAL_GetTick();
			return;
		}
		else if(timing == 6 && now - settime >= 2000)
		{
			timing = 0;
			return;
		}
		if(now - last_trigger_time > 1002)
		{
			last_trigger_time = now;
			timing = 1;
		}
		else if(now - last_trigger_time >= 999)
		{
			__set_BASEPRI(1 << 4);
			last_trigger_time = now;
			if(timing < 5)
				timing++;
			else if(timing == 7)
			{
				time_t time = base_timestamp + ONESEC;
				set_base_time(time);
			}
			__set_BASEPRI(0);
		}
	}
}

int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, 50);
    return len;
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
	  ON_R();
  }
}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{

}
#endif
