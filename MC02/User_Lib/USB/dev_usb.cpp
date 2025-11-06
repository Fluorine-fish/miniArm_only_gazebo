/**
*   @file dev_usb.c
*   @brief 
*   @author Wenxin HU
*   @date 25-11-6
*   @version 1.0
*   @note
*/
#include "dev_usb.h"
#include "usbd_cdc_if.h"

Command_FIFO cmd_FIFO;
uint16_t msg_cnt = 0;
uint16_t send_cnt = 0;
uint8_t last_msg[32];

uint16_t Temperature = 0;
uint16_t Light = 0;
uint8_t Enable_flag = 0;

class USB_Data {
public:
    uint8_t CPP_myUSBRxData[64];
    uint16_t CPP_myUSBRxNum;
    uint8_t CPP_myUSBTxData[64];
    uint16_t CPP_myUSBTxNum;
    SerialPacket_t msg_from_minipc;
    SerialPacket_t msg_to_minipc;

    void CPP_USBData_Init(void);
    void CPP_USBData_Process(void);
    void CPP_USBData_GetData(uint8_t* Buf, uint32_t* Len);
    void CPP_USBData_SendData(uint8_t* Buf, uint32_t* Len);
    void CPP_USBData_SendMsg(char isOn,
                                   char isLightOn,
                                   uint32_t Light,
                                   uint32_t Temprature);
    void USBData_FeedBack();
};

/**
 * @brief 初始化接收来自上位机数据的帧格式配置
 */
void USB_Data::CPP_USBData_Init() {
    msg_from_minipc.start = 's';
    msg_from_minipc.end = 'e';
    msg_from_minipc.datatype = 0xA0; // 上位机发送数据类型
    msg_from_minipc.command = 0xFF;  // 命令字，0xFF表示无效命令
    for (int i = 0; i < 8; i++) {
        msg_from_minipc.data[i] = 0; // 初始化数据域 上位机数据域留空
    }

    msg_to_minipc.start = 's';
    msg_to_minipc.end = 'e';
    msg_to_minipc.datatype = 0xB0; // 下位机发送数据类型
    msg_to_minipc.command = 0xFF;  // 命令字，0xFF表示无效命令
    msg_to_minipc.data[0] = 1;     //默认开机
    msg_to_minipc.data[1] = 1;     //初始化后是开灯状态
    msg_to_minipc.data[2] = 500;   //初始亮度
    msg_to_minipc.data[3] = 5300;  //初始色温
    for (int i = 4; i < 8; i++) {
        msg_from_minipc.data[i] = 0; // 初始化数据域 下位机剩余数据域留空
    }

    //FIFO 初始化
    cmd_FIFO.front = 0; // 队列头指针
    cmd_FIFO.rear = 0;  // 队列尾指针
    for (char i = 0; i < 20; i++) {
        cmd_FIFO.queue[i] = 0; // 初始化队列
    }
}

/**
 * @brief 处理数据
 */
void USB_Data::CPP_USBData_Process(void) {
    if (CPP_myUSBRxNum) {
        msg_cnt += 1; //接收到数据计数+1

        if (CPP_myUSBRxData[1] == 0xA0) {
            //满足上位机发送数据协议的处理方法
            msg_from_minipc.start = CPP_myUSBRxData[0];
            msg_from_minipc.datatype = CPP_myUSBRxData[1];
            msg_from_minipc.command = CPP_myUSBRxData[2];
            uint32_t data[8] = {0}; //数据域
            for (int i = 0; i < 8; i++) {
                data[i] = *(uint32_t*)(CPP_myUSBRxData+3+(i*4));
            }
            msg_from_minipc.end = CPP_myUSBRxData[31];

            char command = msg_from_minipc.command;
            if (command != 0xFF) {
                if (Enable_flag != 1) {
                    uint8_t msg_buf[32] = {0};
                    msg_buf[0] = 's'; //Error开头
                    msg_buf[1] = 0xB0;
                    msg_buf[2] = 0xBF; //命令字
                    for (char i = 3; i < 31; i++) {
                        //中间留空
                        msg_buf[i] = 0x00;
                    }
                    msg_buf[31] = 'e'; //帧尾
                    uint32_t msg_len = 32; //加上Error开头的字节
                    CPP_USBData_SendData(msg_buf, &msg_len);
                    return; //如果没有开机就不处理命令
                }
                if (command == 0xAF) {
                    //0xAF用来测试出队
                    char cmd;
                    deQueue(&cmd_FIFO, &cmd, nullptr);
                }else if (command == 0x40){ //上位机要求下位机回传数据
                    uint8_t msg_buf[32] = {0};
                    msg_buf[0] = 's'; //Error开头
                    msg_buf[1] = 0xB0;
                    msg_buf[2] = 0x41; //命令字

                    uint32_t isLight = 1;
                    uint32_t isOpen = 1;
                    if (Light == 0) {
                        isLight = 0; //如果光照强度为0，表示灯光关闭
                    }
                    uint32_t tmpLight = Light;
                    uint32_t tmpTemperature = Temperature;

                    // 使用局部数组存储变量的字节数据
                    uint8_t isLightBytes[4];
                    uint8_t isOpenBytes[4];
                    uint8_t tmpLightBytes[4];
                    uint8_t tmpTemperatureBytes[4];

                    // 将变量拆分为字节存储到数组中
                    memcpy(isLightBytes, &isLight, sizeof(isLight));
                    memcpy(isOpenBytes, &isOpen, sizeof(isOpen));
                    memcpy(tmpLightBytes, &tmpLight, sizeof(tmpLight));
                    memcpy(tmpTemperatureBytes, &tmpTemperature, sizeof(tmpTemperature));

                    // 数据域 1 为是否开机 2 为是否开灯 3 为光照强度 4 为色温
                    msg_buf[3] = isLightBytes[0];
                    msg_buf[4] = isLightBytes[1];
                    msg_buf[5] = isLightBytes[2];
                    msg_buf[6] = isLightBytes[3];
                    msg_buf[7] = isOpenBytes[0];
                    msg_buf[8] = isOpenBytes[1];
                    msg_buf[9] = isOpenBytes[2];
                    msg_buf[10] = isOpenBytes[3];
                    msg_buf[11] = tmpLightBytes[0];
                    msg_buf[12] = tmpLightBytes[1];
                    msg_buf[13] = tmpLightBytes[2];
                    msg_buf[14] = tmpLightBytes[3];
                    msg_buf[15] = tmpTemperatureBytes[0];
                    msg_buf[16] = tmpTemperatureBytes[1];
                    msg_buf[17] = tmpTemperatureBytes[2];
                    msg_buf[18] = tmpTemperatureBytes[3];

                    for (char i = 19; i < 31; i++) {
                        //中间留空
                        msg_buf[i] = 0x00;
                    }
                    msg_buf[31] = 'e'; //帧尾

                    uint32_t msg_len = 32; //加上Error开头的字节
                    for (char i = 0; i < 32; i++) {
                        CPP_USBData_SendData(msg_buf, &msg_len);
                    }
                    for (char i = 0; i < 32; i++) {
                        last_msg[i] = msg_buf[i];
                    }
                    send_cnt += 1;
                }else {
                    if (command != 0x16) {
                        enQueue(&cmd_FIFO, command, nullptr);
                    }else {
                        enQueue(&cmd_FIFO, command, data); //将命令入队
                    }
                }
            }else {
                // uint8_t msg_buf[32] = {0};
                // msg_buf[0] = 'E'; //Error开头
                // for (char i = 0; i < CPP_myUSBRxNum && i < 32; i++) {
                //     msg_buf[i + 1] = CPP_myUSBRxData[i]; //将接收到的数据复制到msg_buf
                // }
                // uint32_t msg_len = CPP_myUSBRxNum + 1; //加上Error开头的字节
                // CPP_USBData_SendData(msg_buf, &msg_len);
            }

            memset(CPP_myUSBRxData, 0, 64); //数据处理后清空缓存区
            CPP_myUSBRxNum = 0;             //有利于判断，置0
        }
        else {
            //如果不是协议约定的格式 就原样将数据发回，并且以Error开头
            uint8_t msg_buf[32] = {0};
            msg_buf[0] = 'E'; //Error开头
            for (char i = 0; i < CPP_myUSBRxNum && i < 32; i++) {
                msg_buf[i + 1] = CPP_myUSBRxData[i]; //将接收到的数据复制到msg_buf
            }
            uint32_t msg_len = CPP_myUSBRxNum + 1; //加上Error开头的字节
            CPP_USBData_SendData(msg_buf, &msg_len);
        }
        memset(CPP_myUSBRxData, 0, 64); //数据处理后清空缓存区
        CPP_myUSBRxNum = 0;             //有利于判断，置0
    }
}

void USB_Data::CPP_USBData_GetData(uint8_t* Buf, uint32_t* Len) {
    memset(CPP_myUSBRxData, 0, 64);     //清空缓存区
    memcpy(CPP_myUSBRxData, Buf, *Len); //接收的数据复制到缓存区
    CPP_myUSBRxNum = *Len;              //复制字节数
}

void USB_Data::CPP_USBData_SendMsg(char isOn,
                                   char isLightOn,
                                   uint32_t Light,
                                   uint32_t Temprature) {
    msg_to_minipc.start = 's';
    msg_to_minipc.end = 'e';
    msg_to_minipc.datatype = 0xB0; // 下位机发送数据类型
    msg_to_minipc.data[0] = isOn; //是否开机
    msg_to_minipc.data[1] = isLightOn; //是否开灯
    msg_to_minipc.data[2] = Light; //光照强度
    msg_to_minipc.data[3] = Temprature; //色温
    for (int i = 4; i < 8; i++) {
        msg_to_minipc.data[i] = 0; //下位机剩余数据域留空
    }

    CPP_myUSBTxNum = 32;
    CPP_myUSBTxData[0] = msg_to_minipc.start;
    CPP_myUSBTxData[1] = msg_to_minipc.datatype;

    // 将数据域转换为字节数组
    for (char i = 0; i < 8; i++) {
        for (char j = 0; j < 4; j++) {
            CPP_myUSBTxData[i * 4 + j + 2] = *((char*)(&msg_to_minipc.data[i]) + j);
        }
    }

    for (char i = 27; i < 30; i++) {
        //中间留空
        CPP_myUSBTxData[i] = 0x00;
    }
    CPP_myUSBTxData[31] = msg_to_minipc.end;

    CDC_Transmit_HS(CPP_myUSBTxData, CPP_myUSBTxNum);
    memset(CPP_myUSBTxData, 0, 64); //数据处理后清空缓存区
    CPP_myUSBTxNum = 0;             //有利于判断，置0
}


void USB_Data::CPP_USBData_SendData(uint8_t* Buf, uint32_t* Len) {
    memset(CPP_myUSBTxData, 0, 64);
    memcpy(CPP_myUSBTxData, Buf, *Len);
    CPP_myUSBTxNum = *Len;
    CDC_Transmit_HS(Buf, *Len);
    memset(CPP_myUSBTxData, 0, 64);
}

/**
 * @brief 发送反馈数据到上位机
 */
void USB_Data::USBData_FeedBack() {
    // 发送数据到上位机
    CPP_USBData_SendMsg(msg_to_minipc.data[0], msg_to_minipc.data[1],
                        msg_to_minipc.data[2], msg_to_minipc.data[3]);
}

USB_Data USB_Data1;

void CPP_USB_Task() {
    USB_Data1.CPP_USBData_Process();
    // USB_Data1.CPP_USBData_SendMsg(_todo.posture_reminder, _todo.eye_reminder);
}

extern "C" {
void USB_Task() {
    CPP_USB_Task();
}

void USBData_Process() {
    USB_Data1.CPP_USBData_Process();
}

void USBData_GetData(uint8_t* Buf, uint32_t* Len) {
    USB_Data1.CPP_USBData_GetData(Buf, Len);
}

void USBData_SendData(uint8_t* Buf, uint32_t* Len) {
    USB_Data1.CPP_USBData_SendData(Buf, Len);
}

void USBData_init() {
    USB_Data1.CPP_USBData_Init();
}
}
