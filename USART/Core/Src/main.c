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
uint8_t rx_buffer[BUF_SIZE];
volatile uint16_t rx_length = 0;
volatile uint16_t data_ready = 0;
FATFS fs;
FIL fil;
UINT bw;
uint8_t res = 0;
SD_Config current_config;
volatile uint8_t open = 0;
volatile uint8_t origin = 1;
volatile uint64_t last_trigger_time = 0;
volatile uint32_t timer_ms_count = 0;

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  MX_SPI1_Init();
  MX_TIM6_Init();
  MX_TIM2_Init();
  MX_TIM7_Init();
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);
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
		open = 1;
	current_config = load_and_apply_config();
  }
  HAL_UART_Receive_DMA(&huart1, rx_buffer, sizeof(rx_buffer));
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  ASIC_CMD(0x01, 250);
  receivercode = 0x0001;
  ON_G();

  while (1)
  {
	  if(data_ready)
	  {
		  time_t receive_time = get_current_systick();
		  uint8_t time[14];
		  Timing_Status res = GPS_message_process(time);
		  if(origin == 1 && res == Timing_OK)
		  {
			  time_t GPStime = standard_to_stamp(time);
			  time_t process_time = get_current_systick();
			  time_t timestamp = GPStime + process_time - receive_time;
			  set_base_time(timestamp);
			  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
			  origin = 0;
		  }
	  }
	  else
		  maintain_processing_buffer();
	  if(sampling_ready == 1)
	  {
	  	  delay_ms_non_blocking(500);
		  if(ReadResult() == HAL_OK)
		  {
			  Send_Data();
			  if(open == 1)
			  {
				  if(f_write(&fil, datatx, transmitlength, &bw) == FR_OK)
				  	  f_sync(&fil);
			  }
		  }
	  }
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == PPS_Pin)
	{
		__disable_irq();
		time_t timestamp = base_timestamp + 0xf4240;
		set_base_time(timestamp);
		__enable_irq();
	}
}

void delay_ms_non_blocking(uint32_t ms)
{
    HAL_TIM_Base_Stop_IT(&htim7);
    __HAL_TIM_SET_COUNTER(&htim7, 0);
    __HAL_TIM_CLEAR_FLAG(&htim7, TIM_FLAG_UPDATE);
    timer_ms_count = ms;
    HAL_TIM_Base_Start_IT(&htim7);
    while (timer_ms_count > 0)
    {
		__WFI();
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
	  SHINE_R();
	  HAL_Delay(1000);
  }
}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{

}
#endif
