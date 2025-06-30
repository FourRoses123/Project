#ifndef INC_USER_SPI_H_
#define INC_USER_SPI_H_

#include "main.h"
#include "sdcard.h"
#include "spi.h"

#define ASIC_CS_HIGH()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET) //片选拉高
#define ASIC_CS_LOW()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET) //片选拉低

#define PEAKTIME_PERIOD (1ULL << 33)

extern uint16_t PEAKTH;
extern uint16_t ALMSTTH;
extern uint16_t PKWND;
extern uint8_t codeid;
extern uint16_t peaklevel;
extern uint64_t peaktime;
extern uint8_t result[6];

uint8_t ASIC_TransmitReceive(uint8_t data);
void ASIC_CMD(uint8_t address, uint16_t data);
HAL_StatusTypeDef ReadResult(void);
void ASIC_RST(void);

#endif
