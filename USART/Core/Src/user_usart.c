#include "user_usart.h"
#include "user_spi.h"
#include "usart.h"
#include "sdcard.h"
#include "sdconfig.h"
#include "tim.h"

uint8_t process_buffer[HANDLE_SIZE]; //用于处理接收数据的数组
volatile uint8_t *wp = process_buffer; //写指针，指向接收数据的末尾
volatile uint8_t *rp = process_buffer; //读指针
uint8_t version = 0x01;
volatile uint16_t receivercode = 0x0001; //接收机编码
uint16_t cmd[cmd_number] = {0x0001, 0x0003, 0x0004, 0x0005};
uint16_t command; //记录命令码的全局变量
volatile uint8_t *rp1 = NULL;
volatile uint8_t *rp2 = NULL;
volatile uint8_t *rp3 = NULL;
uint8_t datalength; //记录数据包中数据部分长度的全局变量
uint16_t firmware = 0x0100; //固件版本
volatile uint16_t sampling_ready = 1; //采样标记
volatile uint64_t high_counter = 0; //定时器为32位，高32位需由该变量保存
time_t base_timestamp = 0; //记录基准时间戳的全局变量,本地时间
uint64_t base_systick = 0; //记录设置时间基准时的定时器数据
volatile uint8_t state = 0; //设备状态
volatile uint8_t retransmit = 1; //是否重发
uint16_t FREQ = 60; //频率
volatile uint16_t heartcount = 0;
volatile uint8_t first_time2_irq = 1;
volatile uint16_t GPS_count = 5;
volatile uint8_t ASIC_count = 0;
uint8_t GPS_buffer[128];
uint16_t GPSlen;
UART_Queue txQueue;
uint8_t tx_buffer[256];
uint16_t tx_length;
volatile uint8_t *wp_asic = &tx_buffer[9];


void Sampling(void);
void Send_time(void);
void Parameterset_query(void);

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
	uint8_t samplingtx[13];
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
}

void Parameterset_query(void) //参数设置查询函数
{
	uint8_t parameter[32];
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
		parameter[rp3 - rp1 + 8] = 0x7E;
		parameter[rp3 - rp1 + 9] = 0xFE;
	}
	else
	{
		ReadResult();
		parameter[rp3 - rp1 + 6] = (FREQ >> 8) & 0xFF;
		parameter[rp3 - rp1 + 7] = FREQ & 0xFF;
		parameter[rp3 - rp1 + 8] = (firmware >> 8) & 0xFF;
		parameter[rp3 - rp1 + 9] = firmware & 0xFF;
		parameter[rp3 - rp1 + 12] = 0x7E;
		parameter[rp3 - rp1 + 13] = 0xFE;
	}
	for(uint16_t i = 0;i < rp3 - rp1 + 6;i++)
	{
		if(i < rp3 - rp1)
			parameter[i] = *(rp1 + i);
		if(command == cmd3)
		{
			if(i >= 3 && i <= 4)
				parameter[i] = *(rp3 + i - 3);
			if(i == 7)
				parameter[i] = 0x06;
		}
		if(command == cmd4 && i == 7)
			parameter[i] = 0x0A;
		if(i == 5)
			parameter[i] = 0x01;
		if(i == rp3 - rp1)
		{
			parameter[i] = (PEAKTH >> 8) & 0xFF;
			parameter[i + 1] = PEAKTH & 0xFF;
		}
		if(i == rp3 - rp1 + 2)
		{
			parameter[i] = (ALMSTTH >> 8) & 0xFF;
			parameter[i + 1] = ALMSTTH & 0xFF;
		}
		if(i == rp3 - rp1 + 4)
		{
			parameter[i] = (PKWND >> 8) & 0xFF;
			parameter[i + 1] = PKWND & 0xFF;
		}
	}
	size_t txlen = 12 + parameter[7];
	size_t checklength = 6 + parameter[7];
	SUM = Checksum(&parameter[2], checklength);
	if(command == 3)
	{
		parameter[rp3 - rp1 + 6] = (SUM >> 8) & 0xFF;
		parameter[rp3 - rp1 + 7] = SUM & 0xFF;
	}
	else
	{
		parameter[rp3 - rp1 + 10] = (SUM >> 8) & 0xFF;
		parameter[rp3 - rp1 + 11] = SUM & 0xFF;
	}
}

time_t get_current_systick(void) //获取当前系统计时 (µs)
{
  uint32_t high1, low;
  do
  {
    high1 = (uint32_t)(high_counter >> 32);
    low = TIM2->CNT;
  } while (high1 != (uint32_t)(high_counter >> 32));
  return high_counter + low; // 返回64位，低32位为定时器记录，高32位由high_counter记录
}

void set_base_time(time_t timestamp) // 设置时间基准
{
  base_timestamp = timestamp;
  base_systick = get_current_systick();
}

time_t get_current_timestamp(void) //获取当前时间戳 (µs)
{
  time_t current_systick = get_current_systick();
  return base_timestamp + (current_systick - base_systick);
}

void Send_time(void) // 校时
{
	uint8_t time_buffer[20];
	time_t timestamp = 0;
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
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) //5s触发增加一次heartcount记数，每30s发送一次心跳
{
	  if (htim->Instance == TIM2)
	  {
		  if (first_time2_irq)
			first_time2_irq = 0;
		  else
			high_counter += 0x100000000;
	  }
	  if (htim->Instance == TIM6)
		  heartcount++;
	  if(heartcount >= 6)
	  {
		  Sendheart();
		  heartcount = 0;
	  }
	  if (htim->Instance == TIM7)
	  {
		  if (timer_ms_count > 0)
			  timer_ms_count--;
	  }
}

void Sendheart(void) //发送心跳
{
	  static uint8_t hearttx[14];
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

void maintain_processing_buffer(void) //存储数据超过1024字节且处理数据也超过1024字节后前移
{
	uint8_t *address = process_buffer;
    if (wp - address > BUF_SIZE/2 && rp != process_buffer)
    {
        uint16_t move_len = rp - process_buffer;
        __set_BASEPRI(1 << 4);
        memmove(process_buffer, rp, move_len);
        wp = wp - move_len;
        rp = process_buffer;
        __set_BASEPRI(0);
    }
}

void CMD_HANDLE_ERROR(CMD_Status cmdstate) //错误码发送
{
	uint8_t error_buffer[14];
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
}

Timing_Status GPS_message_process(uint8_t *time)
{
	while (wp > rp)
	{
		uint8_t* end_of_line = memchr(rp, '\n', wp - rp);
		if (end_of_line == NULL)
			return Timing_ERROR;
		uint8_t* start_of_line = rp;
		uint16_t line_len = end_of_line - start_of_line;
		rp = end_of_line + 1;
		uint8_t* dollar_sign = memchr(start_of_line, '$', line_len);
		if (dollar_sign == NULL)
		{
			rp = end_of_line + 1;
			continue;
		}
		uint8_t str1[] = "$GPRMC";
		uint8_t str2[] = "$GNRMC";
		if(strncmp(dollar_sign, str1, 6) == 0 || strncmp(dollar_sign, str2, 6) == 0)
		{
			uint8_t* p = dollar_sign;
			int comma_count = 0;
			while (p < end_of_line && comma_count < 2)
			{
				if (*p == ',')
					comma_count++;
				p++;
			}
			if(GPS_count >= 5)
			{
				GPSlen = end_of_line - dollar_sign + 1;
				uint16_t head = HEAD;
				uint16_t tail = TAIL;
				GPS_buffer[0] = (head >> 8) & 0xFF;
				GPS_buffer[1] = head & 0xFF;
				GPS_buffer[2] = version;
				GPS_buffer[3] = (receivercode >> 8) & 0xFF;
				GPS_buffer[4] = receivercode & 0xFF;
				GPS_buffer[5] = 0x01;
				GPS_buffer[6] = 0x06;
				GPS_buffer[7] = GPSlen;
				memcpy(&GPS_buffer[8], dollar_sign, GPSlen);
				uint16_t SUM = Checksum(&GPS_buffer[2], GPSlen + 6);
				GPS_buffer[8 + GPSlen] = (SUM >> 8) & 0xFF;
				GPS_buffer[9 + GPSlen] = SUM & 0xFF;
				GPS_buffer[10 + GPSlen] = (tail >> 8) & 0xFF;
				GPS_buffer[11 + GPSlen] = tail & 0xFF;
				GPSlen += 12;
				UART_Send_Data(GPS_buffer, GPSlen);
				GPS_count = 0;
			}
			if(timing == 6)
				return Timing_DONE;
			if(timing == 5)
			{
				if (comma_count == 2 && p < end_of_line && *p == 'A')
				{
					volatile uint16_t count = 0;
					memcpy(&time[6], dollar_sign + 7, 6);
					memcpy(&time[12], dollar_sign + 14, 2);
					uint8_t* date_ptr = dollar_sign;
					while(date_ptr < end_of_line)
					{
						if(*date_ptr == ',')
							count++;
						if(count == 9)
							break;
						date_ptr++;
					}
					if (count == 9)
					{
						memcpy(time, date_ptr + 1, 6);
						return Timing_OK;
					}
					else
						return Timing_ERROR;
				}
			}
		}
	}
	return Timing_ERROR;
}

int calculate(uint16_t *data)
{
    return data[0] * 10 + data[1];
}

time_t standard_to_stamp(uint8_t *time)
{
    struct tm stm;
    int year, mon, mday, hour, min, sec, ms;
    uint16_t number[14];
    for(uint16_t i = 0;i < 14;i++)
        number[i] = time[i] - '0';
    year = calculate(&number[4]);
    mon = calculate(&number[2]);
    mday = calculate(&number[0]);
    hour = calculate(&number[6]);
    min = calculate(&number[8]);
    sec = calculate(&number[10]);
    ms = calculate(&number[12]);
    memset(&stm, 0, sizeof(stm));
    stm.tm_year = year + 100;
    stm.tm_mon = mon - 1;
    stm.tm_mday = mday;
    stm.tm_hour = hour;
    stm.tm_min = min;
    stm.tm_sec = sec;
    time_t systic = mktime(&stm);
    systic = systic*1000000 + ms*10000;
    return systic;
}

void UART_Queue_Init(void)
{
    memset(&txQueue, 0, sizeof(UART_Queue));
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

void ASIC_message_read(void)
{
	if(wp_asic + 15 < &tx_buffer[255])
	{
		if(ReadResult() == ASIC_OK)
		{
			ASIC_count++;
			*wp_asic = codeid;
			wp_asic++;
			*wp_asic = (peaklevel >> 8) & 0xFF;
			*(wp_asic + 1) = peaklevel & 0xFF;
			wp_asic += 2;
			time_t datatime = base_timestamp + peaktime - base_systick;
			for(uint16_t i = 0;i < 8;i++)
				*(wp_asic + i) = (datatime >> (56 - 8 * i)) & 0xFF;
			wp_asic += 8;
		}
	}
}

void Data_Consolidation(void)
{
	uint16_t head = HEAD;
	uint16_t tail = TAIL;
	uint8_t datalength = wp_asic - &tx_buffer[9] + 1;
	tx_buffer[0] = (head >> 8) & 0xFF;
	tx_buffer[1] = head & 0xFF;
	tx_buffer[2] = version;
	tx_buffer[3] = (receivercode >> 8) & 0xFF;
	tx_buffer[4] = receivercode & 0xFF;
	tx_buffer[5] = 0x01;
	tx_buffer[6] = 0x07;
	tx_buffer[7] = datalength;
	tx_buffer[8] = ASIC_count;
	uint16_t checklength = datalength + 6;
	uint16_t SUM = Checksum(&tx_buffer[2], checklength);
	*wp_asic = (SUM >> 8) & 0xFF;
	*(wp_asic + 1) = SUM & 0xFF;
	*(wp_asic + 2) = (tail >> 8) & 0xFF;
	*(wp_asic + 3) = tail & 0xFF;
	tx_length = datalength + 12;
	UART_Send_Data(tx_buffer, tx_length);
	ASIC_count = 0;
	wp_asic = &tx_buffer[9];
}

