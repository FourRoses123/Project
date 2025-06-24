#include "sdcard.h"
#include "spi.h"

uint8_t SD_TYPE = 0;
const char *CONFIG_FILE = "config.txt";
const char *LOG_PREFIX = "log";
char current_log_file[20];
static int cached_max_log_num = -1;

void SHINE_R(void)
{
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
	HAL_Delay(1000);
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
}

void SHINE_G(void)
{
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
	HAL_Delay(1000);
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
}

uint8_t SPI_TransmitReceive(uint8_t data) //发送接收函数封装
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&hspi2, &data, &rx_data, 1 ,100);
    return rx_data;
}

void SD_PowerOnSeq(void) //80个时钟周期
{
    SD_CS_HIGH();
    for(uint8_t i=0; i<10; i++)
    {
        SPI_TransmitReceive(0xFF);
    }
}

uint8_t SD_SendCmd(uint8_t cmd, uint32_t arg, uint8_t crc)  //发送cmd命令，arg参数及crc校准
{
	uint8_t retry = 0;
	uint8_t response;
	do{
		retry=SPI_TransmitReceive(0xFF);
	}while(retry!=0xFF);  //这段如果不加，发送CMD8返回值会错误

    SPI_TransmitReceive(0x40 | cmd);
    SPI_TransmitReceive((arg >> 24) & 0xFF);
    SPI_TransmitReceive((arg >> 16) & 0xFF);
    SPI_TransmitReceive((arg >> 8)  & 0xFF);
    SPI_TransmitReceive(arg & 0xFF);
    SPI_TransmitReceive(crc | 0x01);
    retry = 0;

	do {
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

	retry = 1000; // 设置超时值
	do {
	    r1 = SD_SendCmd(CMD0, 0, 0x95);
	    retry--;
	} while (r1 != 0x01 && retry > 0);

	if (retry == 0)
	{
	    SD_CS_HIGH();
	    return SD_ERROR; // 超时返回错误
	}

	SD_TYPE=0;
	r1 = SD_SendCmd(CMD8, 0x1AA, 0x87);
	if(r1==0x01)
	{
		for(i=0;i<4;i++)buff[i]=SPI_TransmitReceive(0xFF);
		if(buff[2]==0X01&&buff[3]==0XAA)
		{
			retry=0XFFFE;
			do
			{
				SD_SendCmd(CMD55,0,0);
				r1=SD_SendCmd(ACMD41,0x40000000,0);
			}while(r1&&retry--);
			r1=SD_SendCmd(CMD58,0,0);
			if(retry&&r1==0)
			{
				for(i=0;i<4;i++)buff[i]=SPI_TransmitReceive(0XFF);
				if(buff[0]&0x40){
					SD_TYPE=V2HC;
				}else {
					SD_TYPE=V2;
				}
			}
		}
	}
	else
	{
		SD_SendCmd(CMD55,0,0);
		r1=SD_SendCmd(ACMD41,0,0);
		if(r1<=1)
		{
			SD_TYPE=V1;
			retry=0XFFFE;
			do
			{
				SD_SendCmd(CMD55,0,0);
				r1=SD_SendCmd(ACMD41,0,0);
			}while(r1&&retry--);
		}else
		{
			SD_TYPE=MMC;
			retry=0XFFFE;
			do
			{
				r1=SD_SendCmd(CMD1,0,0);
			}while(r1&&retry--);
		}
		if(retry==0||SD_SendCmd(CMD16,512,0)!=0)SD_TYPE=ERR;
	}
	SD_CS_HIGH();
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    HAL_SPI_Init(&hspi2);

    if(SD_TYPE != ERR)
    	return SD_OK;
    else
    	return SD_ERROR;
}

uint8_t SD_ReceiveData(uint8_t *data, uint16_t len) //从SD卡读取指定长度的数据块，len为512时为单块大小
{
   uint8_t r1;
   SD_CS_LOW();
   do
   {
      r1 = SPI_TransmitReceive(0xFF);
      HAL_Delay(100);
   }while(r1 != 0xFE); // sd卡spi模式单块读取的起始令牌是0xFE
  while(len--)
  {
   *data = SPI_TransmitReceive(0xFF);
   data++;
  }
  SPI_TransmitReceive(0xFF);
  SPI_TransmitReceive(0xFF);
  SD_CS_HIGH();
  return 0;
}

uint8_t SD_SendBlock(uint8_t*buf,uint8_t cmd) //写入扇区内容，512字节
{
	uint16_t i;
	uint8_t r1;
	do{
		r1=SPI_TransmitReceive(0xFF);
	}while(r1!=0xFF);

	SPI_TransmitReceive(cmd);
	if(cmd!=0xFD) //0xFD为多块写入停止令牌
	{
		for(i = 0;i < 512;i++)
			SPI_TransmitReceive(buf[i]);
		SPI_TransmitReceive(0xFF);
		SPI_TransmitReceive(0xFF);
		i=SPI_TransmitReceive(0xFF);
		if((i&0x1F)!=0x05) //低5位为状态码，高3位为保留，应舍弃，0x05表示接收正确
			return 2;
	}
    return 0;
}

uint8_t SD_ReadDisk(uint8_t*buf,uint32_t sector,uint8_t cnt) //读SD卡，buf缓存区 sector起始扇区 cnt扇区数
{
	SD_CS_LOW();
	uint8_t r1;
	if(SD_TYPE!=V2HC)
		sector *= 512;
	if(cnt==1)
	{
		r1=SD_SendCmd(CMD17,sector,0);
		if(r1==0)
			r1=SD_ReceiveData(buf,512);
	}
	else
	{
		r1=SD_SendCmd(CMD18,sector,0);
		do
		{
			r1=SD_ReceiveData(buf,512);
			buf+=512;
		}while(--cnt && r1==0);
		SD_SendCmd(CMD12,0,0);
	}
	SD_CS_HIGH();
	return r1;
}

uint8_t SD_WriteDisk(uint8_t*buf,uint32_t sector,uint8_t cnt) //写SD卡，buf缓存区 sector起始扇区 cnt扇区数
{
	SD_CS_LOW();
	uint8_t r1;
	if(SD_TYPE!=V2HC)
		sector *= 512;
	if(cnt==1)
	{
		r1=SD_SendCmd(CMD24,sector,0);
		if(r1==0)
		{
			r1=SD_SendBlock(buf,0xFE);
		}
	}
	else
	{
		if(SD_TYPE!=MMC)
		{
			SD_SendCmd(CMD55,0,0);
			SD_SendCmd(CMD23,cnt,0);
		}
 		r1=SD_SendCmd(CMD25,sector,0);
		if(r1==0)
		{
			do
			{
				r1=SD_SendBlock(buf,0xFC); //0xFC为多块写入起始令牌
				buf+=512;
			}while(--cnt && r1==0);
			r1=SD_SendBlock(0,0xFD);
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
    	r1=SD_ReceiveData(csd_data, 16);
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
