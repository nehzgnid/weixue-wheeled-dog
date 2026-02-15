#include "usb_cmd_handler.h"
#include <string.h>

// === 环形缓冲区定义 ===
#define RING_BUF_SIZE 512
static uint8_t rx_ring_buf[RING_BUF_SIZE];
static volatile uint32_t head = 0;
static volatile uint32_t tail = 0;

// 推入数据 (在 USB ISR 中调用)
void USB_RingBuffer_Push(uint8_t* buf, uint32_t len) {
    for(uint32_t i=0; i<len; i++) {
        uint32_t next = (head + 1) % RING_BUF_SIZE;
        if (next != tail) { // 没满
            rx_ring_buf[head] = buf[i];
            head = next;
        } else {
            // 溢出丢弃
            break; 
        }
    }
}

// 弹出数据 (在 RxTask 中调用)
// 返回 1 表示成功读到一个字节，0 表示空
uint32_t USB_RingBuffer_Pop(uint8_t* byte) {
    if (head == tail) return 0; // 空
    
    *byte = rx_ring_buf[tail];
    tail = (tail + 1) % RING_BUF_SIZE;
    return 1;
}

// 这里不再需要 Servo_Process_Loop，逻辑移入 RxTask
