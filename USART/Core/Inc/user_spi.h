#ifndef INC_USER_SPI_H_
#define INC_USER_SPI_H_

#include "main.h"
#include "sdcard.h"
#include "spi.h"

#define ASIC_CS_HIGH()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET) //片选拉高
#define ASIC_CS_LOW()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET) //片选拉低

#define PEAKTIME_PERIOD (1ULL << 33)

typedef enum{
	ASIC_OK = 0,
	ASIC_ERROR,
	ASIC_EMPTY,
}ASIC_Status;

extern volatile uint16_t PEAKTH;
extern volatile uint16_t ALMSTTH;
extern volatile uint16_t PKWND;
extern volatile uint8_t codeid;
extern volatile uint16_t peaklevel;
extern volatile uint64_t peaktime;
extern volatile uint8_t result[6];
extern volatile int32_t wrap_count;

uint8_t ASIC_TransmitReceive(uint8_t data);
void ASIC_CMD(uint8_t address, uint16_t data);
ASIC_Status ReadResult(void);
void ASIC_RST(void);

#endif
