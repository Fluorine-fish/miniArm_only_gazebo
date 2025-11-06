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

typedef enum {
    light_brighter = 0x10,    //提升光照亮度
    light_dimmer = 0x11,      //降低光照亮度
    temperature_upper = 0x12, //提升色温
    temperature_lower = 0x13, //降低色温
    light_on = 0x14,          //上位机开灯
    light_off = 0x15,         //上位机关灯
    posture_reminder = 0x20,  //上位机坐姿提醒
    eye_reminder = 0x21,      //上位机远眺提醒
    arm_back = 0x02,          //上位机复位
    light_set = 0x16,         //上位机谁当灯光
    arm_forward = 0x30,
    arm_backward = 0x31,          //机械臂前后移动
    arm_left = 0x32,          //机械臂左转
    arm_right = 0x33,         //机械臂右转

    reading_light = 0x50,      //阅读灯模式
    learning_light = 0x51,    //学习灯模式
} Command_enum;

typedef struct {
    // 消息队列类型为位
    uint64_t queue[20];   // 命令消息循环队列
    uint32_t data[20][8]; // 数据域，最多存储8个float类型数据
    uint8_t front;        // 队列头指针
    uint8_t rear;         // 队列尾指针
} Command_FIFO;

typedef struct {
    char start; //0 帧头取 's'
    char type;  //1 消息类型：上->下：0xA3 下->上：0xB3
    char light_on;
    char light_off;
    char light_brighter;
    char light_dimmer;
    char posture_reminder;
    char eye_reminder;
    char end; //31 帧尾取'e'
} usb_msg_t;

typedef struct {
    char light_on;
    char light_off;
    char light_brighter;
    char light_dimmer;
    char posture_reminder;
    char eye_reminder;
} arm_todo_t;

typedef struct {
    char start;       // 0 帧头，取 's'
    char datatype;    // 1 消息类型
    char command;     // 2 命令字
    uint32_t data[8]; // 3 - 26 数据域，最多存储8个float类型数据
    // 27 - 30 预留空位
    char end; // 31 帧尾，取 'e'
} SerialPacket_t;

/***用于操作命令消息队列的函数***/

/** @brief 判断队列是否为空
 * @param q 命令消息队列指针
 * @return 1表示为空,0表示不为空
 */
inline char isEmpty(Command_FIFO* q) {
    if (q->front == q->rear) {
        return 1; // 队列为空
    }
    else {
        return 0; // 队列不为空
    }
};
/**
 * @brief 判断队列是否满
 * @param q 命令消息队列指针
 * @return 1表示为满,0表示不为满
 */
inline char isFull(Command_FIFO* q) {
    if ((q->rear + 1) % 20 == q->front) {
        return 1; // 队列已满
    }
    else {
        return 0; // 队列未满
    }
}

/**
 * @brief 队列入队
 * @param q 命令消息队列指针
 * @param command 入队的命令
 * @return -1表示失败 0表示成功
 */
inline char enQueue(Command_FIFO* q, char command, uint32_t* data) {
    if (isFull(q)) {
        return -1;
    }
    else {
        q->queue[q->rear] = command; // 先写入数据
        if (data != NULL) {
            for (int i = 0; i < 8; i++) {
                q->data[q->rear][i] = data[i]; // 将数据域也写入
            }
        }
        else {
            for (int i = 0; i < 8; i++) {
                q->data[q->rear][i] = 0; // 如果没有数据，填充0
            }
        }
        q->rear = (q->rear + 1) % 20; // 再更新队尾指针
        return 0;                     // 成功入队
    }
}

/**
 * @brief 队列出队
 * @param q 命令队列指针
 * @param command 用于接收读出命令的指针
 * @return -1表示失败,0表示成功
 */
static inline char deQueue(Command_FIFO* q, char* command, uint32_t* data) {
    if (isEmpty(q)) {
        return -1; // 队列为空，无法出队
    }
    else {
        *command = q->queue[q->front]; // 先读取数据
        if (data != NULL) {
            for (int i = 0; i < 8; i++) {
                data[i] = q->data[q->front][i]; // 将数据域也读取出来
            }
        }
        q->front = (q->front + 1) % 20; // 再更新队头指针
        return 0;                       // 成功出队
    }
}

/*********************/

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
#ifdef __cplusplus
}
#endif

#endif //USB_TASK_H
