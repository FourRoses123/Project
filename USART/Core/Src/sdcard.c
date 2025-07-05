#include "sdcard.h"
#include "spi.h"

const char *CONFIG_FILE = "config.txt";
const char *LOG_PREFIX = "log";
char current_log_file[20];
static int cached_max_log_num = -1;
uint8_t SD_TYPE = 0;
static volatile uint8_t spi_dma_completed = 0;
static uint8_t spi_dummy_tx[512];


#if defined ( __ICCARM__ )
#pragma data_alignment=4
uint8_t dma_aligned_buffer[512];
#elif defined ( __CC_ARM )
__attribute__((aligned(4))) uint8_t dma_aligned_buffer[512];
#elif defined ( __GNUC__ )
uint8_t dma_aligned_buffer[512] __attribute__((aligned(4)));
#endif


uint8_t SPI_TransmitReceive(uint8_t data)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&hspi2, &data, &rx_data, 1, 100);
    return rx_data;
}

static void SPI_Wait_Not_Busy(void)
{
    uint32_t timeout = HAL_GetTick() + 20;
    while (__HAL_SPI_GET_FLAG(&hspi2, SPI_FLAG_BSY))
        if (HAL_GetTick() > timeout) break;
}

static uint8_t SD_Wait_Ready(void)
{
    uint8_t res;
    uint32_t timeout = HAL_GetTick() + 500;
    do {
        res = SPI_TransmitReceive(0xFF);
        if (res == 0xFF)
            return 0;
    } while (HAL_GetTick() < timeout);
    return 1;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == hspi2.Instance)
        spi_dma_completed = 1;

}
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == hspi2.Instance)
        spi_dma_completed = 1;
}


void SD_PowerOnSeq(void)
{
    SD_CS_HIGH();
    for (uint8_t i = 0; i < 10; i++)
        SPI_TransmitReceive(0xFF);
}

uint8_t SD_SendCmd(uint8_t cmd, uint32_t arg, uint8_t crc)  //发送cmd命令，arg参数及crc校准
{
    uint8_t response, retry = 0;
    while (SPI_TransmitReceive(0xFF) != 0xFF && retry < 200)
    	retry++;

    SPI_TransmitReceive(cmd | 0x40);
    SPI_TransmitReceive((uint8_t)(arg >> 24));
    SPI_TransmitReceive((uint8_t)(arg >> 16));
    SPI_TransmitReceive((uint8_t)(arg >> 8));
    SPI_TransmitReceive((uint8_t)(arg));
    SPI_TransmitReceive(crc);

    retry = 0;
    do
    {
        response = SPI_TransmitReceive(0xFF);
        retry++;
    } while ((response & 0x80) && (retry < 0xFF));
    return response;
}

SD_Status SD_Init(void) //SD卡初始化
{
    uint8_t r1;
	uint8_t buff[6] = {0};
	uint16_t retry;
	uint8_t i;

	SD_PowerOnSeq();
	SD_CS_LOW();

	retry = 100;
	do {
	    r1 = SD_SendCmd(CMD0, 0, 0x95);
	    retry--;
	} while (r1 != 0x01 && retry > 0);

	if (retry == 0) {
	    SD_CS_HIGH();
	    return SD_ERROR;
	}

	SD_TYPE=0;
	r1 = SD_SendCmd(CMD8, 0x1AA, 0x87);
	if(r1==0x01) {
		for(i=0;i<4;i++) buff[i]=SPI_TransmitReceive(0xFF);
		if(buff[2]==0X01&&buff[3]==0XAA) {
			retry=0XFFFE;
			do {
				SD_SendCmd(CMD55,0,0);
				r1=SD_SendCmd(ACMD41,0x40000000,0);
			} while(r1&&retry--);
			r1=SD_SendCmd(CMD58,0,0);
			if(retry&&r1==0) {
				for(i=0;i<4;i++) buff[i]=SPI_TransmitReceive(0XFF);
				if(buff[0]&0x40) SD_TYPE=V2HC;
				else SD_TYPE=V2;
			}
		}
	} else {
		SD_SendCmd(CMD55,0,0);
		r1=SD_SendCmd(ACMD41,0,0);
		if(r1<=1) {
			SD_TYPE=V1;
			retry=0XFFFE;
			do {
				SD_SendCmd(CMD55,0,0);
				r1=SD_SendCmd(ACMD41,0,0);
			} while(r1&&retry--);
		} else {
			SD_TYPE=MMC;
			retry=0XFFFE;
			do {
				r1=SD_SendCmd(CMD1,0,0);
			} while(r1&&retry--);
		}
		if(retry==0||SD_SendCmd(CMD16,512,0)!=0) SD_TYPE=ERR;
	}
	SD_CS_HIGH();

    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    HAL_SPI_Init(&hspi2);

    if (SD_TYPE != ERR)
    {
        memset(spi_dummy_tx, 0xFF, sizeof(spi_dummy_tx));
        return SD_OK;
    }
    return SD_ERROR;
}

static uint8_t SD_ReceiveBlock(uint8_t *buff, uint16_t len) //从SD卡读取指定长度的数据块，len为512时为单块大小
{
    uint8_t r1;
    uint16_t retry = 0;

    do {
        r1 = SPI_TransmitReceive(0xFF);
        retry++;
    } while (r1 != 0xFE && retry < 40000);
    if (retry >= 40000)
    	return SD_ERROR_TOKEN_TIMEOUT;

    spi_dma_completed = 0;
    if (HAL_SPI_TransmitReceive_DMA(&hspi2, spi_dummy_tx, buff, len) != HAL_OK)
        return SD_ERROR_DMA_START;

    uint32_t wait_timeout = HAL_GetTick() + 100;
    while (spi_dma_completed == 0) {
        if (HAL_GetTick() > wait_timeout) {
            HAL_SPI_Abort(&hspi2);
            return SD_ERROR_DMA_TIMEOUT;
        }
    }
    SPI_Wait_Not_Busy();
    SPI_TransmitReceive(0xFF);
    SPI_TransmitReceive(0xFF);

    return SD_OK;
}


static uint8_t SD_SendBlockDMA(const uint8_t*buf, uint8_t cmd) //写入扇区内容，512字节
{
    uint16_t retry = 0;

    do {
        if (SPI_TransmitReceive(0xFF) == 0xFF)
        	break;
        retry++;
    } while (retry < 0xFFFF);
    if (retry >= 0xFFFF)
    	return 1;
    SPI_TransmitReceive(cmd);
    if (cmd != 0xFD)
    {
        spi_dma_completed = 0;
        if (HAL_SPI_Transmit_DMA(&hspi2, (uint8_t*)buf, 512) != HAL_OK)
            return 2;
        uint32_t wait_timeout = HAL_GetTick() + 100;
        while (spi_dma_completed == 0)
        {
            if (HAL_GetTick() > wait_timeout)
            {
                HAL_SPI_Abort(&hspi2);
                return 3;
            }
        }
        SPI_Wait_Not_Busy();
        SPI_TransmitReceive(0xFF);
        SPI_TransmitReceive(0xFF);
        uint8_t r1 = SPI_TransmitReceive(0xFF);
        if ((r1 & 0x1F) != 0x05)
        	return 4;
    }
    return 0;
}


uint8_t SD_ReadDisk(uint8_t*buf, uint32_t sector, uint8_t cnt) //读SD卡，buf缓存区 sector起始扇区 cnt扇区数
{
    uint8_t r1;
    SD_CS_LOW();
    if (SD_TYPE != V2HC) sector *= 512;

    if (cnt == 1)
    {
        r1 = SD_SendCmd(CMD17, sector, 0);
        if (r1 == 0)
            r1 = SD_ReceiveBlock(buf, 512);
    }
    else
    {
        r1 = SD_SendCmd(CMD18, sector, 0);
        if (r1 == 0)
        {
            do {
                r1 = SD_ReceiveBlock(buf, 512);
                buf += 512;
            } while (--cnt && r1 == SD_OK);
            SD_SendCmd(CMD12, 0, 0);
        }
    }
    SD_CS_HIGH();
    return r1;
}

uint8_t SD_WriteDisk(const uint8_t*buf, uint32_t sector, uint8_t cnt) {
    uint8_t r1;
    SD_CS_LOW();
    if (SD_TYPE != V2HC) sector *= 512;
    if (cnt == 1)
    {
        r1 = SD_SendCmd(CMD24, sector, 0);
        if (r1 == 0)
        {
            r1 = SD_SendBlockDMA(buf, 0xFE);
            if (r1 == 0)
            {
                if (SD_Wait_Ready() != 0)
                    r1 = 1;
            }
        }
    }
    else
    {
        if (SD_TYPE != MMC)
        {
            SD_SendCmd(CMD55, 0, 0);
            SD_SendCmd(CMD23, cnt, 0);
        }
        r1 = SD_SendCmd(CMD25, sector, 0);
        if (r1 == 0)
        {
            do {
                r1 = SD_SendBlockDMA(buf, 0xFC);
                if (r1 != 0)
                	break;
                buf += 512;
            } while (--cnt);
            if (SD_SendBlockDMA(0, 0xFD) == 0)
            {
                 if (SD_Wait_Ready() != 0)
                     r1 = 1;
            }
            else
                r1 = 1;
        }
    }
    SD_CS_HIGH();
    return r1;
}
uint8_t SD_GETCSD(uint8_t *csd_data) //获取CSD数据
{
	SD_CS_LOW();
	uint8_t r1;
    if(SD_SendCmd(CMD9,0,0) == 0)
    {
    	r1=SD_ReceiveBlock(csd_data, 16);
    }
	SD_CS_HIGH();
	if(r1)
		return 1;
	else
		return 0;
}

uint32_t SD_GetSectorCount(void) //获取SD卡总扇区数
{
    uint8_t csd[16];
    uint32_t Capacity;
    uint8_t n;
	uint16_t csize;
    if(SD_GETCSD(csd)!=0)
    	return 0;
    if((csd[0]&0xC0)==0x40)
    {
		csize = csd[9] + ((uint16_t)csd[8] << 8) + 1;
		Capacity = (uint32_t)csize << 10;
    }
    else
    {
		n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
		csize = (csd[8] >> 6) + ((uint16_t)csd[7] << 2) + ((uint16_t)(csd[6] & 3) << 10) + 1;
		Capacity= (uint32_t)csize << (n - 9);
    }
    return Capacity;
}

FRESULT file_exists(const char *path) //检查配置文件是否存在
{
    FILINFO fno;
    return f_stat(path, &fno);
}

int get_next_log_number(void) //寻找sd卡中生成的文件最大的数字并返回最大数字+1
{
    if (cached_max_log_num == -1)
    {
        DIR dir;
        FILINFO fno;
        const char *prefix = LOG_PREFIX;
        const size_t prefix_len = strlen(prefix);
        int max_num = -1;

        if (f_opendir(&dir, "/") == FR_OK)
        {
            while (f_readdir(&dir, &fno) == FR_OK && fno.fname[0] != 0)
            {
                char* name = fno.fname;
                size_t len = strlen(name);

                if (len == prefix_len + 9 && strncmp(name, prefix, prefix_len) == 0)
                {
                    int num = 0;
                    for (int i = 0; i < 5; i++)
                    {
                        char c = name[prefix_len + i];
                        if (c < '0' || c > '9') break;
                        num = num * 10 + (c - '0');
                    }

                    if (strcmp(name + prefix_len + 5, ".txt") == 0)
                    {
                        if (num > max_num) max_num = num;
                    }
                }
            }
            f_closedir(&dir);
        }
        cached_max_log_num = (max_num < 0) ? 0 : max_num;
    }
    	return ++cached_max_log_num;
}
