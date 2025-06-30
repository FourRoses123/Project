#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"
#include "string.h"
#include "stdio.h"
#include "stdarg.h"
#include "unistd.h"
#include "stdbool.h"
#include "inttypes.h"
#include "time.h"

#define BUF_SIZE 2048

#define LED_R_Pin GPIO_PIN_0
#define LED_R_GPIO_Port GPIOA
#define LED_G_Pin GPIO_PIN_1
#define LED_G_GPIO_Port GPIOA
#define ASIC_RST_Pin GPIO_PIN_4
#define ASIC_RST_GPIO_Port GPIOA
#define AFULL_Pin GPIO_PIN_1
#define AFULL_GPIO_Port GPIOB
#define EMPTY_Pin GPIO_PIN_2
#define EMPTY_GPIO_Port GPIOB
#define S2_Pin GPIO_PIN_10
#define S2_GPIO_Port GPIOB
#define S1_Pin GPIO_PIN_11
#define S1_GPIO_Port GPIOB
#define S3_Pin GPIO_PIN_3
#define S3_GPIO_Port GPIOB
#define PPS_Pin GPIO_PIN_4
#define PPS_GPIO_Port GPIOB
#define PPS_EXTI_IRQn EXTI4_IRQn

#define ON_R()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET) //红灯亮
#define OFF_R()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET) //红灯灭
#define ON_G()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET) //绿灯亮
#define OFF_G()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET) //绿灯灭
#define Toggle_R() HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0) //红灯反转
#define Toggle_G() HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1) //绿灯反转

void Error_Handler(void);
extern uint8_t rx_buffer[BUF_SIZE];
extern volatile uint16_t rx_length;
extern volatile uint16_t data_ready;
extern volatile uint8_t timing;
extern volatile uint32_t timer_ms_count;
extern volatile uint8_t origin;

int _write(int file, char *ptr, int len);
void delay_ms_non_blocking(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif
