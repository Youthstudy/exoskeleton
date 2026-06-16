/**
 * 串口接收解析模块
 * 配合 Python SerialReceiver 使用
 */

#ifndef SERIAL_PARSER_H
#define SERIAL_PARSER_H

#include <stdint.h>
#include <string.h>
#include "dma.h"
#include "usart.h"

// ==================== 配置 ====================

#define FRAME_START     0x3A
#define FRAME_END_0     0x0D
#define FRAME_END_1     0x0A
#define MAX_PAYLOAD_LEN 512
#define RX_BUFFER_SIZE  512
#define UART_DMA_RX_BUF_SIZE 256

// ==================== 数据结构 ====================

typedef enum {
    PARSE_IDLE,         // 等待帧头
    PARSE_LENGTH,       // 等待长度
    PARSE_PAYLOAD,      // 接收payload
    PARSE_END_0,        // 等待结束符0x0D
    PARSE_END_1         // 等待结束符0x0A
} ParseState;

typedef struct {
    uint8_t buffer[RX_BUFFER_SIZE];
    uint16_t write_idx;
    uint16_t read_idx;
} RingBuffer;

typedef struct {
    ParseState state;
    uint8_t payload[MAX_PAYLOAD_LEN];
    uint8_t payload_len;
    uint8_t payload_idx;
} FrameParser;

int parse_int32s(const uint8_t *payload, uint8_t len, int32_t *out, uint8_t max_count);
int parse_floats(const uint8_t *payload, uint8_t len, float *out, uint8_t max_count) ;
int serial_parse_frame(uint8_t *out_payload, uint8_t *out_len) ;
void serial_rx_irq_handler(uint8_t byte) ;
void serial_parser_init(void) ;
void uart_rx_idle_callback(void);

extern uint8_t uart_dma_rx_buf[UART_DMA_RX_BUF_SIZE];

#endif // SERIAL_PARSER_H

