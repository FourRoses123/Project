#ifndef INC_SDCARD_H_
#define INC_SDCARD_H_

#include "main.h"
#include "stm32l4xx_hal.h"
#include "diskio.h"
#include "ff.h"

#define CMD0   0
#define CMD1   1
#define CMD8   8
#define CMD9   9
#define CMD12  12
#define CMD16  16
#define CMD17  17
#define CMD18  18
#define CMD23  23
#define CMD24  24
#define CMD25  25
#define CMD55  55
#define CMD58  58
#define ACMD41 41
#define ERR     		0x00
#define MMC				0x01
#define V1				0x02
#define V2				0x04
#define V2HC			0x06

#define SD_CS_HIGH() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET) //片选拉高
#define SD_CS_LOW()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET) //片选拉低

extern const char *CONFIG_FILE;
extern const char *LOG_PREFIX;
extern char current_log_file[20];
typedef enum
{
    SD_OK = 0,
	SD_ERROR_CMD_TIMEOUT,
	SD_ERROR_TOKEN_TIMEOUT,
	SD_ERROR_DMA_START,
	SD_ERROR_DMA_TIMEOUT,
	SD_ERROR_CMD12,
	SD_ERROR,

} SD_Status;

SD_Status SD_Init(void);
uint8_t SPI_TransmitReceive(uint8_t data);
uint8_t SD_SendCmd(uint8_t cmd, uint32_t arg, uint8_t crc);
uint8_t SD_ReceiveData(uint8_t *data, uint16_t len);
uint8_t SD_ReadDisk(uint8_t*buf,uint32_t sector,uint8_t cnt);
uint8_t SD_WriteDisk(const uint8_t*buf, uint32_t sector, uint8_t cnt);
uint8_t SD_SendBlock(uint8_t*buf,uint8_t cmd);
uint8_t SD_GETCSD(uint8_t *csd_data);
uint32_t SD_GetSectorCount();
FRESULT file_exists(const char *path);
int get_next_log_number(void);

#endif
