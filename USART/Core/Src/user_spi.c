#include "user_spi.h"
#include "usart.h"

uint16_t PEAKTH = 200;
uint16_t ALMSTTH = 32;
uint16_t PKWND = 5000;
uint8_t codeid;
uint16_t peaklevel;
uint64_t peaktime;
uint8_t result[6];
uint64_t pre_peaktime = 0;
volatile uint32_t wrap_count = 0;

uint64_t get_extended_peaktime(uint64_t peaktime);

uint8_t ASIC_TransmitReceive(uint8_t data) //发送接收函数封装
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&hspi1, &data, &rx_data, 1, 100);
    return rx_data;
}

void ASIC_CMD(uint8_t address, uint16_t data) //ASIC发送参数到指定地址
{
	ASIC_CS_LOW();
	ASIC_TransmitReceive(address);
	ASIC_TransmitReceive(0x00);
	ASIC_TransmitReceive(0x00);
	ASIC_TransmitReceive(0x00);
	ASIC_TransmitReceive(0x00);
	ASIC_TransmitReceive((data >> 8) & 0xFF);
	ASIC_TransmitReceive(data & 0xFF);
	ASIC_CS_HIGH();
}

HAL_StatusTypeDef ReadResult(void) //读取相关结果
{
	if(HAL_GPIO_ReadPin(EMPTY_GPIO_Port, EMPTY_Pin) == GPIO_PIN_SET)
	{
		return HAL_ERROR;
	}
	else
	{
		ASIC_CS_LOW();
		uint16_t i;
		uint8_t address = 0x80;
		ASIC_TransmitReceive(address);
		for(i = 0;i < 6;i++)
			result[i] = ASIC_TransmitReceive(0x00);
		codeid = (result[0] >> 3) + 1;
		peaklevel = (result[1] >> 1) | ((result[0] & 0x07 ) << 7);
		peaktime = ((uint64_t)(result[1] & 0x01) << 32) |
		           ((uint64_t)result[2] << 24) |
		           ((uint64_t)result[3] << 16) |
		           ((uint64_t)result[4] << 8)  |
		           (uint64_t)result[5];
		peaktime = get_extended_peaktime(peaktime);
		ASIC_CS_HIGH();
		return HAL_OK;
	}
}

void ASIC_RST(void) //ASIC芯片复位
{
	HAL_GPIO_WritePin(ASIC_RST_GPIO_Port, ASIC_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(ASIC_RST_GPIO_Port, ASIC_RST_Pin, GPIO_PIN_SET);
}

uint64_t get_extended_peaktime(uint64_t peaktime)
{
	if(peaktime < pre_peaktime)
		wrap_count++;
	pre_peaktime = peaktime;
	return peaktime + (wrap_count * PEAKTIME_PERIOD);
}
