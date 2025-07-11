#ifdef USE_OBSOLETE_USER_CODE_SECTION_0

#endif

#include <string.h>
#include "ff_gen_drv.h"
#include "spi.h"
#include "sdcard.h"

static volatile DSTATUS Stat = STA_NOINIT;

DSTATUS USER_initialize (BYTE pdrv);
DSTATUS USER_status (BYTE pdrv);
DRESULT USER_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
  DRESULT USER_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
#endif
#if _USE_IOCTL == 1
  DRESULT USER_ioctl (BYTE pdrv, BYTE cmd, void *buff);
#endif

Diskio_drvTypeDef  USER_Driver =
{
  USER_initialize,
  USER_status,
  USER_read,
#if  _USE_WRITE
  USER_write,
#endif
#if  _USE_IOCTL == 1
  USER_ioctl,
#endif
};

DSTATUS USER_initialize (
	BYTE pdrv
)
{
	uint8_t res;
	res = SD_Init();
	if(res)
	{
		hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
		SPI_TransmitReceive(0xFF);
		hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	}
	if(res)
		return  STA_NOINIT;
	else
		return RES_OK;
  /* USER CODE END INIT */
}

DSTATUS USER_status (
	BYTE pdrv
)
{
	switch (pdrv)
	{
		case 0 :
			return RES_OK;
		case 1 :
			return RES_OK;
		default:
			return STA_NOINIT;
	}
}

DRESULT USER_read (
	BYTE pdrv,
	BYTE *buff,
	DWORD sector,
	UINT count
)
{
	uint8_t res;
  if(count == 0)
	  return RES_PARERR;
  switch (pdrv)
  	{
	case 0:
	{
		res=SD_ReadDisk(buff,sector,count);
		if(res == 0)
			return RES_OK;
		else
			return RES_ERROR;
	}
	default:
		return RES_ERROR;
  	}
}

#if _USE_WRITE == 1
DRESULT USER_write (
	BYTE pdrv,
	const BYTE *buff,
	DWORD sector,
	UINT count
)
{
	uint8_t  res;
	if( !count )
	{
		return RES_PARERR;
	}
	switch (pdrv)
	{
		case 0:
			res=SD_WriteDisk((uint8_t *)buff,sector,count);
				if(res == 0)
					return RES_OK;
				else
					return RES_ERROR;
		default:return RES_ERROR;
	}
}
#endif

#if _USE_IOCTL == 1
DRESULT USER_ioctl (
	BYTE pdrv,		/* 物理驱动器号 */
	BYTE cmd,		/* 控制代码 */
	void *buff		/* 发送/接收数据缓冲区指针 */
)
{
	DRESULT res = RES_ERROR;
	DWORD *p_dw = (DWORD*)buff;
	if (pdrv) return RES_PARERR;

	switch (cmd) {
		case CTRL_SYNC :
			res = RES_OK;
			break;
		case GET_SECTOR_COUNT :
			*p_dw = SD_GetSectorCount();
			res = RES_OK;
			break;
		case GET_SECTOR_SIZE :
			*(WORD*)buff = 512;
			res = RES_OK;
			break;
		case GET_BLOCK_SIZE :
			*(DWORD*)buff = 1;
			res = RES_OK;
			break;
		default:
			res = RES_PARERR;
			break;
	}

	return res;
}
#endif

