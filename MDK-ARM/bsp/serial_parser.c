#include "serial_parser.h"


// ==================== 全局变量 ====================

static RingBuffer g_rx_ring;
static FrameParser g_parser;


uint8_t uart_dma_rx_buf[UART_DMA_RX_BUF_SIZE];
uint16_t uart_dma_last_pos = 0;

// ==================== 环形缓冲区 ====================

static inline void ring_init(RingBuffer *rb) {
    rb->write_idx = 0;
    rb->read_idx = 0;
}

static inline uint16_t ring_available(RingBuffer *rb) {
    return (rb->write_idx - rb->read_idx + RX_BUFFER_SIZE) % RX_BUFFER_SIZE;
}

static inline void ring_push(RingBuffer *rb, uint8_t byte) {
    rb->buffer[rb->write_idx] = byte;
    rb->write_idx = (rb->write_idx + 1) % RX_BUFFER_SIZE;
}

static inline uint8_t ring_pop(RingBuffer *rb) {
    uint8_t byte = rb->buffer[rb->read_idx];
    rb->read_idx = (rb->read_idx + 1) % RX_BUFFER_SIZE;
    return byte;
}

// ==================== 初始化 ====================

void serial_parser_init(void) {
    ring_init(&g_rx_ring);
    g_parser.state = PARSE_IDLE;
    g_parser.payload_len = 0;
    g_parser.payload_idx = 0;
}

// ==================== 串口中断调用 ====================

/**
 * 在串口接收中断中调用此函数
 * @param byte 接收到的字节
 */
void serial_rx_irq_handler(uint8_t byte) {
    ring_push(&g_rx_ring, byte);
}

// ==================== 帧解析 ====================

/**
 * 解析一帧数据
 * @param out_payload 输出payload缓冲区
 * @param out_len 输出payload长度
 * @return 1=解析成功, 0=无完整帧
 */
int serial_parse_frame(uint8_t *out_payload, uint8_t *out_len) {
    while (ring_available(&g_rx_ring) > 0) {
        uint8_t byte = ring_pop(&g_rx_ring);
        
        switch (g_parser.state) {
        case PARSE_IDLE:
            if (byte == FRAME_START) {
                g_parser.state = PARSE_LENGTH;
            }
            break;
            
        case PARSE_LENGTH:
            g_parser.payload_len = byte;
            g_parser.payload_idx = 0;
            if (g_parser.payload_len == 0) {
                g_parser.state = PARSE_END_0;
            } else {
                g_parser.state = PARSE_PAYLOAD;
            }
            break;
            
        case PARSE_PAYLOAD:
            g_parser.payload[g_parser.payload_idx++] = byte;
            if (g_parser.payload_idx >= g_parser.payload_len) {
                g_parser.state = PARSE_END_0;
            }
            break;
            
        case PARSE_END_0:
            if (byte == FRAME_END_0) {
                g_parser.state = PARSE_END_1;
            } else {
                g_parser.state = PARSE_IDLE;  // 错误，重新开始
            }
            break;
            
        case PARSE_END_1:
            g_parser.state = PARSE_IDLE;
            if (byte == FRAME_END_1) {
                // 解析成功
                memcpy(out_payload, g_parser.payload, g_parser.payload_len);
                *out_len = g_parser.payload_len;
                return 1;
            }
            break;
        }
    }
    return 0;
}

// ==================== 数据解析辅助函数 ====================

/**
 * 从payload解析float数组（小端）
 */
int parse_floats(const uint8_t *payload, uint8_t len, float *out, uint8_t max_count) {
    int count = len / sizeof(float);
    if (count > max_count) count = max_count;
    
    for (int i = 0; i < count; i++) {
        memcpy(&out[i], payload + i * sizeof(float), sizeof(float));
    }
    return count;
}

/**
 * 从payload解析int32数组（小端）
 */
int parse_int32s(const uint8_t *payload, uint8_t len, int32_t *out, uint8_t max_count) {
    int count = len / sizeof(int32_t);
    if (count > max_count) count = max_count;
    
    for (int i = 0; i < count; i++) {
        memcpy(&out[i], payload + i * sizeof(int32_t), sizeof(int32_t));
    }
    return count;
}

void uart_rx_idle_callback(void)
{
    uint16_t dma_pos;
    uint16_t data_len;

    // DMA 当前写到哪里了
    dma_pos = UART_DMA_RX_BUF_SIZE
            - __HAL_DMA_GET_COUNTER(huart6.hdmarx);

    if (dma_pos >= uart_dma_last_pos)
    {
        data_len = dma_pos - uart_dma_last_pos;
    }
    else
    {
        data_len = UART_DMA_RX_BUF_SIZE
                 - uart_dma_last_pos
                 + dma_pos;
    }

    // 把新收到的数据逐字节丢进你的 ring buffer
    for (uint16_t i = 0; i < data_len; i++)
    {
        uint8_t byte =
            uart_dma_rx_buf[(uart_dma_last_pos + i)
                             % UART_DMA_RX_BUF_SIZE];

        serial_rx_irq_handler(byte);   // ★ 复用你原来的接口
    }

    uart_dma_last_pos = dma_pos;
}


// ==================== 使用示例 ====================

#if 0  // 示例代码，实际使用时改为 1 或复制到你的main.c

#include "serial_parser.h"

// 串口中断示例 (STM32 HAL)
void USART1_IRQHandler(void) {
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)) {
        uint8_t byte = huart1.Instance->DR;
        serial_rx_irq_handler(byte);
    }
}

// 主循环处理
void main_loop(void) {
    uint8_t payload[MAX_PAYLOAD_LEN];
    uint8_t len;
    
    if (serial_parse_frame(payload, &len)) {
        // 收到完整帧，处理数据
        
        // 示例1: 解析为float数组
        float values[8];
        int count = parse_floats(payload, len, values, 8);
        // 使用 values[0], values[1], ...
        
        // 示例2: 根据命令字节处理
        if (len > 0) {
            uint8_t cmd = payload[0];
            switch (cmd) {
            case 0x01:  // 命令1
                // 处理...
                break;
            case 0x02:  // 命令2
                // 处理...
                break;
            }
        }
    }
}
#endif
