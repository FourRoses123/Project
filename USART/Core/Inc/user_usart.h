#ifndef INC_USART_CMD_H_
#define INC_USART_CMD_H_

#include "main.h"

#define HEAD 0xAA55
#define TAIL 0x7EFE
#define cmd1 0x0001 //开始、停止采样
#define cmd3 0x0003 //设置参数
#define cmd4 0x0004 //查询参数
#define cmd5 0x0005 //对时
#define cmd_number  4
#define TX_QUEUE_SIZE 8 //队列最大数量
#define MAX_PACKET_SIZE 256 //单个队列发送的最大数据量

typedef enum{
	CMD_OK = 0,
	CODE_ERROE,
	LENGTH_ERROE,
	CMD_ERROR,
	CHECK_ERROR,
	INVALID,
	CMD_END,
}CMD_Status;

typedef enum{
	Timing_OK = 0,
	Timing_ERROR,
	Timing_DONE,
}Timing_Status;


typedef struct __attribute__((packed)){
    uint8_t data[MAX_PACKET_SIZE];
    uint16_t length;
} UART_Packet;

typedef struct {
    UART_Packet packets[TX_QUEUE_SIZE];
    uint16_t head;
    uint16_t tail;
    uint8_t count;
    uint8_t isSending;
} UART_Queue;

extern uint8_t process_buffer[];
extern volatile uint8_t *wp;
extern volatile uint8_t *rp;
extern volatile uint16_t receivercode;
extern volatile uint16_t sampling_ready;
extern volatile uint16_t txstate;
extern volatile uint16_t samplingstate;
extern uint16_t transmitlength;
extern uint8_t datatx[23];
extern time_t base_timestamp;
extern uint64_t base_systick;
extern volatile uint16_t GPS_count;
extern uint8_t tx_buffer[256];
extern uint16_t tx_length;
extern volatile uint64_t high_counter;

CMD_Status CMD_Judge(void);
CMD_Status CMD_Execute(void);
void Send_Data(void);
void heartreply(void);
void datareply(void);
void maintain_processing_buffer(void);
void CMD_HANDLE_ERROR(CMD_Status state);
void set_base_time(time_t timestamp);
time_t get_current_systick(void);
time_t get_current_timestamp(void);
time_t standard_to_stamp(uint8_t *time);
Timing_Status GPS_message_process(uint8_t *time);
void Sendheart(void);
void UART_Queue_Init(void);
uint8_t UART_Send_Data(uint8_t *data, uint16_t len);
void ASIC_message_read(void);
void Data_Consolidation(void);

#endif
