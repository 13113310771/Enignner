#ifndef _data_packet_H
#define _data_packet_H

#include "stm32f4xx.h"
#include "protocol.h"
#include "data_fifo.h"

typedef enum
{
  CHASSIS_DATA_ID     = 0x0010,
  GIMBAL_DATA_ID      = 0x0011,
  SHOOT_TASK_DATA_ID  = 0x0012,
  INFANTRY_ERR_ID     = 0x0013,
  CONFIG_RESPONSE_ID  = 0x0014,
  CALI_RESPONSE_ID    = 0x0015,
  REMOTE_CTRL_INFO_ID = 0x0016,
  BOTTOM_VERSION_ID   = 0x0017,
	SENTRY_DATA_ID			= 0X0018,
  
  CHASSIS_CTRL_ID     = 0x00A0,
  GIMBAL_CTRL_ID      = 0x00A1,
  SHOOT_CTRL_ID       = 0x00A2,
  ERROR_LEVEL_ID      = 0x00A3,
  INFANTRY_STRUCT_ID  = 0x00A4,
  CALI_GIMBAL_ID      = 0x00A5,
  
  STU_CUSTOM_DATA_ID  = 0x0100,
} infantry_data_id_e;

typedef enum
{
  UART_IDLE_IT     = 0,
  UART_DMA_HALF_IT = 1,
  UART_DMA_FULL_IT = 2,
} uart_it_type_e;

typedef enum
{
  STEP_HEADER_SOF  = 0,//������ʼ�ֽ�
  STEP_LENGTH_LOW  = 1,//���ݵ��ֽ�
  STEP_LENGTH_HIGH = 2,//���ݸ��ֽ�
  STEP_FRAME_SEQ   = 3,//����֡�����
  STEP_HEADER_CRC8 = 4,//֡ͷ CRC8 У��
  STEP_DATA_CRC16  = 5,//CRC16������У��
} unpack_step_e;

typedef enum
{
	CONTROLLER=0,
	NORMAL=1,
}yaokong_type;

typedef struct
{
  DMA_Stream_TypeDef      *dma_stream;
  fifo_s_t                *data_fifo;
  uint16_t                buff_size;
  uint8_t                 *buff[2];
  uint16_t                read_index;
  uint16_t                write_index;
} uart_dma_rxdata_t;

typedef struct
{
  fifo_s_t       *data_fifo;
  frame_header_t *p_header;
  uint16_t       data_len;
  uint8_t        protocol_packet[PROTOCAL_FRAME_MAX_SIZE];//Э�����֡ͷ����
  unpack_step_e  unpack_step;                             //��ѹ���Ĳ���
  uint16_t       index;
} unpack_data_t;

typedef struct
{
	uint8_t *buf;      	    /* ������ */
	uint32_t buf_size;    	/* ��������С */
  uint32_t occupy_size;   /* ��Ч���ݴ�С */
	uint8_t *pwrite;      	/* дָ�� */
	uint8_t *pread;       	/* ��ָ�� */
	void (*lock)(void);	/* �������� */
	void (*unlock)(void);	/* ������� */
}_fifo_t;


typedef struct
{
	uint8_t status;		/* ����״̬ */
	_fifo_t tx_fifo;	/* ����fifo */
	_fifo_t rx_fifo;	/* ����fifo */
	uint8_t *dmarx_buf;	/* dma���ջ��� */
	uint16_t dmarx_buf_size;/* dma���ջ����С*/
	uint8_t *dmatx_buf;	/* dma���ͻ��� */
	uint16_t dmatx_buf_size;/* dma���ͻ����С */
	uint16_t last_dmarx_size;/* dma��һ�ν������ݴ�С */
}uart_device_t;
typedef void (*lock_fun)(void);


void dma_buffer_to_unpack_buffer(uart_dma_rxdata_t *dma_obj, uart_it_type_e it_type);
uint8_t* protocol_packet_pack(uint16_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t sof, uint8_t *tx_buf);
void data_packet_pack(uint16_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t sof);
void unpack_fifo_data(unpack_data_t *p_obj, uint8_t sof,uint8_t control_type);
uint32_t send_packed_fifo_data(fifo_s_t *pfifo, uint8_t sof);
void fifo_register(_fifo_t *pfifo, uint8_t *pfifo_buf, uint32_t size,
                   lock_fun lock, lock_fun unlock);
void uart_dmarx_done_isr(uint8_t uart_id);
void uart_dmarx_idle_isr(uint8_t uart_id);
void uart_dmarx_half_done_isr(uint8_t uart_id);

#endif
