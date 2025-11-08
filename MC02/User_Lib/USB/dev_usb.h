/**
*   @file dev_usb.h
*   @brief 
*   @author Wenxin HU
*   @date 25-11-6
*   @version 1.0
*   @note
*/
#ifndef DEV_USB_H
#define DEV_USB_H
#include "main.h"

typedef struct {
    char start;       // 0 帧头，取 's'
    char datatype;    // 1 消息类型
    char command;     // 2 命令字
    uint32_t data[6]; // 3 - 27 数据域，最多存储6个float类型数据
    // 28 - 30 预留空位
    char end; // 31 帧尾，取 'e'
} SerialPacket_t;


#ifdef __cplusplus
extern "C" {
#endif
/**
 * @brief USB任务函数
 * @note 需要塞到循环中执行，也可以塞在定时器中断中执行
 */
void USB_Task();

void USBData_Process();

void USBData_GetData(uint8_t* Buf, uint32_t* Len);

void USBData_SendData(uint8_t* Buf, uint32_t* Len);

void USBData_init();

void USB_SendMsg(uint32_t *Data);
#ifdef __cplusplus
}
#endif

#endif //USB_TASK_H
