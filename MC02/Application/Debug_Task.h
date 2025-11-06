#ifndef DEBUG_TASK_H
#define DEBUG_TASK_H

#include "stdint.h"

// 计算得到的控制力矩（下->上）
typedef struct {
    char start;          // 0 帧头，取 's'
    char datatype;       // 1 消息类型 0xFE
    float torque[6];     // 2-25 计算得到的前馈力矩
    uint8_t reserved[5]; //26-30 预留空位（填充0）
    char end;            // 31 帧尾，取 'e'
} Controller_package;



#ifdef __cplusplus
extern "C" {
#endif
void App_DebugTask(void const* argument);
#ifdef __cplusplus
}
#endif

#endif //DEBUG_TASK_H
