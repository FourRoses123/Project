#include "user_usart.h"
#include "user_spi.h"
#include "usart.h"
#include "sdcard.h"
#include "sdconfig.h"
#include "tim.h"

uint8_t processing_buffer[BUF_SIZE]; //用于处理接收数据的数组
uint8_t *wp = processing_buffer; //写指针，指向接收数据的末尾
uint8_t *rp = processing_buffer; //读指针
uint8_t version = 0x01;
volatile uint16_t receivercode = 0x0001; //接收机编码
uint16_t cmd[cmd_number] = {0x0001, 0x0003, 0x0004, 0x0005};
uint16_t command; //记录命令码的全局变量
uint8_t *rp1; //在运行过程中将指向包头第一个字节
uint8_t *rp2; //在运行过程中将指向包尾第一个字节
uint8_t *rp3; //在运行过程中将指向数据第一个字节
uint8_t datalength; //记录数据包中数据部分长度的全局变量
uint16_t firmware = 0x0100; //固件版本
volatile uint16_t sampling_ready = 1; //采样标记
volatile uint64_t high_counter = 0; //定时器为32位，高32位需由该变量保存
uint64_t base_timestamp = 0; //记录基准时间戳的全局变量,本地时间
uint64_t base_systick = 0; //记录设置时间基准时的定时器数据
volatile uint8_t state = 0; //设备状态
volatile uint8_t retransmit = 1; //是否重发
uint16_t FREQ = 60; //频率
volatile uint16_t heartcount = 0;
uint16_t transmitlength = 23;
uint8_t datatx[23];
static UART_Queue txQueue;
static uint8_t samplingtx[13];
static uint8_t tx_buffer[32];
static uint8_t time_buffer[20];
static uint8_t hearttx[14];
static uint8_t error_buffer[14];
void Sampling(void);
void Send_time(void);
void Parameterset_query(void);
void Sendheart(void);
uint8_t UART_Send_Data(uint8_t* data, uint16_t len);
void UART_Start_Send(void);

uint16_t read_be16(uint8_t *data) //连续读取两个字节
{
    return (data[0] << 8) | data[1];
}

uint16_t Checksum(uint8_t *data, uint16_t length) //逐字节校检和
{
	uint16_t sum;
    if(length <= 1)
    {
    	sum = (uint16_t)data[0];
    	return sum;
    }
    else
    {
    	sum = (uint16_t)data[length-1]+Checksum(data, length - 1);
    	return sum;
    }
}

CMD_Status CMD_Judge(void) //按照地址从地到高轮询数组，找到最近的包头和包尾进行判断和处理，没有设计处理参数中如果出现包头包尾的方法，目前如果出现这种情况会直接报错误码舍弃
{
	uint32_t rxcode = 0;
	uint16_t time;
	uint16_t checksum;
	uint16_t SUM;
	rp1 = rp;
	rp2 = rp;
	while (wp > rp)
	{
		uint16_t test = (*rp << 8) | *(rp + 1);
		if(test == HEAD)
			rp1 = rp;
		if(test == TAIL)
			rp2 = rp + 1;
		rp++;
		if(rp2 > rp1)
			break;
	}
	if(rp == wp)
	{
		data_ready = 0;
		return CMD_END;
	}
	for(uint16_t i = 3;i <= 4;i++)
		rxcode = (*(rp1 + i) << (32 - 8*i)) | rxcode;
	datalength = *(rp1 + 7);
	for(time = 0;time < cmd_number;time++)
	{
		command = (*(rp1 + 5)<< 8) | *(rp1 + 6);
		if(command == cmd[time])
			break;
	}
	checksum = (*(rp2 - 3) << 8) | *(rp2 - 2);
	rp3 = rp1 + 8;
	SUM = Checksum(rp1 + 2, datalength + 6);
	if(rxcode != receivercode || *(rp1 + 2) != version)
		return CODE_ERROE;
	else if(datalength != rp2 - rp1 - 11)
		return LENGTH_ERROE;
	else if(time >= cmd_number)
		return CMD_ERROR;
	else if(checksum != SUM)
		return CHECK_ERROR;
	else
		return CMD_OK;
}

CMD_Status CMD_Execute(void) //判断命令内容和处理
{
	if(command == cmd1)
	{
		if(datalength == 0x01 && *rp3 == 0x01)
		{
			Sampling();
			sampling_ready = 0;
			return CMD_OK;
		}
		else if(datalength == 0x01 && *rp3 == 0)
		{
			Sampling();
			sampling_ready = 1;
			return CMD_OK;
		}
		else
			return INVALID;
	}
	else if(command == cmd3)
	{
		Parameterset_query();
		return CMD_OK;
	}
	else if(command == cmd4)
	{
		if(datalength != 0x00)
			return INVALID;
		else
		{
			Parameterset_query();
			return CMD_OK;
		}
	}
	else
	{
		Send_time();
		return CMD_OK;
	}
}

void Sampling(void) //发送采样相关的应答命令
{
	uint16_t length =  rp2 - rp1 + 1;
	uint16_t SUM;
	for(uint16_t i = 0;i < length;i++)
	{
		if(i == 5)
			samplingtx[i] = 0x01;
		else if(i == length - 4)
		{
			SUM = Checksum(&samplingtx[2], i - 2);
			samplingtx[i] = (SUM >> 8) & 0xFF;
		}
		else if(i == length - 3)
			samplingtx[i] = SUM & 0xFF;
		else
			samplingtx[i] = *(rp1 + i);
	}
	UART_Send_Data(samplingtx, length);
}

void Parameterset_query(void) //参数设置查询函数
{
	uint16_t SUM;
	if(command == cmd3)
	{
		uint16_t code = (*rp3 << 8) | *(rp3 + 1);
		receivercode = code;
		PEAKTH = read_be16(rp3 + 2);
		ALMSTTH = read_be16(rp3 + 4);
		PKWND = read_be16(rp3 + 6);
		ASIC_CMD(0x01, PEAKTH);
		ASIC_CMD(0x02, ALMSTTH);
		ASIC_CMD(0x04, PKWND);
		char config_buf[64];
		FIL fil;
		UINT bytes_written;
		if(read_config(config_buf, sizeof(config_buf)) == FR_OK)
		{
			sprintf(config_buf, "[svpinger]\n"
		            "Receivercode=%d\n"
		            "PEAKTH=%d\n"
		        	"ALMSTTH=%d\n"
		        	"PKWND=%d", receivercode, PEAKTH, ALMSTTH, PKWND);
			f_open(&fil, CONFIG_FILE, FA_CREATE_ALWAYS | FA_WRITE);
			f_write(&fil, config_buf, strlen(config_buf), &bytes_written);
			f_close(&fil);
		}
		tx_buffer[rp3 - rp1 + 8] = 0x7E;
		tx_buffer[rp3 - rp1 + 9] = 0xFE;
	}
	else
	{
		ReadResult();
		tx_buffer[rp3 - rp1 + 6] = (FREQ >> 8) & 0xFF;
		tx_buffer[rp3 - rp1 + 7] = FREQ & 0xFF;
		tx_buffer[rp3 - rp1 + 8] = (firmware >> 8) & 0xFF;
		tx_buffer[rp3 - rp1 + 9] = firmware & 0xFF;
		tx_buffer[rp3 - rp1 + 12] = 0x7E;
		tx_buffer[rp3 - rp1 + 13] = 0xFE;
	}
	for(uint16_t i = 0;i < rp3 - rp1 + 6;i++)
	{
		if(i < rp3 - rp1)
			tx_buffer[i] = *(rp1 + i);
		if(command == cmd3)
		{
			if(i >= 3 && i <= 4)
				tx_buffer[i] = *(rp3 + i - 3);
			if(i == 7)
				tx_buffer[i] = 0x06;
		}
		if(command == cmd4 && i == 7)
			tx_buffer[i] = 0x0A;
		if(i == 5)
			tx_buffer[i] = 0x01;
		if(i == rp3 - rp1)
		{
			tx_buffer[i] = (PEAKTH >> 8) & 0xFF;
			tx_buffer[i + 1] = PEAKTH & 0xFF;
		}
		if(i == rp3 - rp1 + 2)
		{
			tx_buffer[i] = (ALMSTTH >> 8) & 0xFF;
			tx_buffer[i + 1] = ALMSTTH & 0xFF;
		}
		if(i == rp3 - rp1 + 4)
		{
			tx_buffer[i] = (PKWND >> 8) & 0xFF;
			tx_buffer[i + 1] = PKWND & 0xFF;
		}
	}
	size_t txlen = 12 + tx_buffer[7];
	size_t checklength = 6 + tx_buffer[7];
	SUM = Checksum(&tx_buffer[2], checklength);
	if(command == 3)
	{
		tx_buffer[rp3 - rp1 + 6] = (SUM >> 8) & 0xFF;
		tx_buffer[rp3 - rp1 + 7] = SUM & 0xFF;
	}
	else
	{
		tx_buffer[rp3 - rp1 + 10] = (SUM >> 8) & 0xFF;
		tx_buffer[rp3 - rp1 + 11] = SUM & 0xFF;
	}
	UART_Send_Data(tx_buffer, txlen);
}

uint64_t get_current_systick(void) //获取当前系统计时 (µs)
{
  uint32_t high1, low;
  do
  {
    high1 = (uint32_t)(high_counter >> 32);
    low = TIM2->CNT;
  } while (high1 != (uint32_t)(high_counter >> 32));
  return high_counter + low; // 返回64位，低32位为定时器记录，高32位由high_counter记录
}

void set_base_time(uint64_t timestamp) // 设置时间基准
{
  base_timestamp = timestamp;
  base_systick = get_current_systick();
}

uint64_t get_current_timestamp(void) //获取当前时间戳 (µs)
{
  uint64_t current_systick = get_current_systick();
  return base_timestamp + (current_systick - base_systick);
}

void Send_time(void) // 校时
{
	uint64_t timestamp = 0;
	for(uint16_t i = 0; i < 8;i++)
	timestamp = (timestamp << 8) | *(rp3 + i);
	set_base_time(timestamp);
	for(uint16_t j = 0;j < rp3 - rp1;j++)
	{
		time_buffer[j] = *(rp1 + j);
		if(j == 5)
			time_buffer[j] = 0x01;
	}
	size_t checklength = 6 + time_buffer[7];
	size_t txlen = 12 + time_buffer[7];
	uint64_t current_timestamp = get_current_timestamp();
	for(uint16_t i = 0; i < 8;i++)
		time_buffer[i + 8] = (current_timestamp >> (56 - 8 * i)) & 0xFF;
	uint16_t SUM = Checksum(&time_buffer[2], checklength);
	time_buffer[rp3 - rp1 + 8] = (SUM >> 8) & 0xFF;
	time_buffer[rp3 - rp1 + 9] = SUM & 0xFF;
	uint16_t tail = TAIL;
	time_buffer[rp3 - rp1 + 10] = (tail >> 8) & 0xFF;
	time_buffer[rp3 - rp1 + 11] = tail & 0xFF;
	UART_Send_Data(time_buffer, txlen);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) //5s触发增加一次heartcount记数，每30s发送一次心跳
{
	  if (htim->Instance == TIM6)
		  heartcount++;
	  if(heartcount >= 6)
	  {
		  Sendheart();
		  heartcount = 0;
	  }
}

void Sendheart(void) //发送心跳
{
	  uint16_t length = 14;
	  uint16_t head = HEAD;
	  uint16_t tail = TAIL;
	  hearttx[0] = (head >> 8) & 0xFF;
	  hearttx[1] = head & 0xFF;
	  hearttx[2] = version;
	  hearttx[3] = (receivercode >> 8) & 0xFF;
	  hearttx[4] = receivercode & 0xFF;
	  hearttx[5] = 0x01;
	  hearttx[6] = 0x00;
	  hearttx[7] = 0x02;
	  hearttx[8] = state;
	  hearttx[9] = (uint8_t)sampling_ready;
	  uint16_t SUM = Checksum(&hearttx[2], 8);
	  hearttx[10] = (SUM >> 8) & 0xFF;
	  hearttx[11] = SUM & 0xFF;
	  hearttx[12] = (tail >> 8) & 0xFF;
	  hearttx[13] = tail & 0xFF;
	  UART_Send_Data(hearttx, length);
}

void Send_Data(void) // 数据上送
{
	  uint16_t head = HEAD;
	  uint16_t tail = TAIL;
	  datatx[0] = (head >> 8) & 0xFF;
	  datatx[1] = head & 0xFF;
	  datatx[2] = version;
	  datatx[3] = (receivercode >> 8) & 0xFF;
	  datatx[4] = receivercode & 0xFF;
	  datatx[5] = 0x01;
	  datatx[6] = 0x02;
	  datatx[7] = 0x0B;
	  datatx[8] = codeid;
	  datatx[9] = (peaklevel >> 8) & 0xFF;
	  datatx[10] = peaklevel & 0xFF;
	  uint64_t datatime = base_timestamp + peaktime - base_systick;
	  for(uint16_t i = 0;i < 8;i++)
		  datatx[i + 11] = (datatime >> (56 - 8 * i)) & 0xFF;
	  uint16_t SUM = Checksum(&datatx[2], 17);
	  datatx[19] = (SUM >> 8) & 0xFF;
	  datatx[20] = SUM & 0xFF;
	  datatx[21] = (tail >> 8) & 0xFF;
	  datatx[22] = tail & 0xFF;
	  UART_Send_Data(datatx, transmitlength);
}

void maintain_processing_buffer(void) //存储数据超过512字节且处理数据也超过512字节后前移512字节
{
	uint8_t *address = processing_buffer;
    if (wp - address > BUF_SIZE/2 && rp != processing_buffer)
    {
        uint16_t move_len = rp - processing_buffer;
        __disable_irq();
        memmove(processing_buffer, rp, move_len);
        wp = wp - move_len;
        rp = processing_buffer;
        __enable_irq();
    }
}

void CMD_HANDLE_ERROR(CMD_Status cmdstate) //错误码发送
{
	uint16_t head = HEAD;
	uint16_t tail = TAIL;
	error_buffer[0] = (head >> 8) & 0xFF;
	error_buffer[1] = head & 0xFF;
	error_buffer[2] = version;
	error_buffer[3] = (receivercode >> 8) & 0xFF;
	error_buffer[4] = receivercode & 0xFF;
	error_buffer[5] = 0xFF;
	error_buffer[6] = command & 0xFF;
	error_buffer[7] = 0x02;
	error_buffer[8] = retransmit;
	error_buffer[9] = (uint8_t)cmdstate;
	uint16_t SUM = Checksum(&error_buffer[2], 8);
	error_buffer[10] = (SUM >> 8) & 0xFF;
	error_buffer[11] = SUM & 0xFF;
	error_buffer[12] = (tail >> 8) & 0xFF;
	error_buffer[13] = tail & 0xFF;
	uint16_t length = 14;
	UART_Send_Data(error_buffer, length);
}

void UART_Queue_Init(void)
{
    txQueue.head = 0;
    txQueue.tail = 0;
    txQueue.count = 0;
    txQueue.isSending = 0;
}

uint8_t UART_Send_Data(uint8_t *data, uint16_t len)// 数据入队
{
    if(len == 0 || len > MAX_PACKET_SIZE || txQueue.count >= TX_QUEUE_SIZE)
        return 0;
    UART_Packet *pkt = &txQueue.packets[txQueue.tail];
    memcpy(pkt->data, data, len);
    pkt->length = len;
    txQueue.tail = (txQueue.tail + 1) % TX_QUEUE_SIZE;
    txQueue.count++;
    if(!txQueue.isSending)
    {
        txQueue.isSending = 1;
        UART_Start_Send();
    }
    return 1;
}

void UART_Start_Send(void) // 启动DMA发送
{
    if(txQueue.count == 0)
    {
        txQueue.isSending = 0;
        return;
    }
    UART_Packet *pkt = &txQueue.packets[txQueue.head];
    HAL_UART_Transmit_DMA(&huart1, pkt->data, pkt->length);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance != USART1)
    	return;
    txQueue.head = (txQueue.head + 1) % TX_QUEUE_SIZE;
    txQueue.count--;
    UART_Start_Send();
}
