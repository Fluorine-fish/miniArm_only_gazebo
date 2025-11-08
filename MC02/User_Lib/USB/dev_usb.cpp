/**
*   @file dev_usb.c
*   @brief 
*   @author Wenxin HU
*   @date 25-11-6
*   @version 1.0
*   @note
*/
#include "dev_usb.h"

#include "bsp_log.h"
#include "usbd_cdc_if.h"

uint16_t msg_cnt = 0;
uint16_t send_cnt = 0;
uint8_t last_msg[32];

float FeedBack_Velocity[6] = {};
float FeedBack_Position[6] = {};
float JointTorque[6] = {};

uint16_t success_cnt = 0;

class USB_Data {
public:
    uint8_t CPP_myUSBRxData[32];
    uint16_t CPP_myUSBRxNum;
    uint8_t CPP_myUSBTxData[32];
    uint16_t CPP_myUSBTxNum;
    SerialPacket_t msg_from_minipc;
    SerialPacket_t msg_to_minipc;

    void CPP_USBData_Init(void);
    void CPP_USBData_Process(void);
    void CPP_USBData_GetData(uint8_t* Buf, uint32_t* Len);
    void CPP_USBData_SendData(uint8_t* Buf, uint32_t* Len);
    void CPP_USBData_SendMsg(uint32_t* Data);
};

/**
 * @brief 初始化接收来自上位机数据的帧格式配置
 */
void USB_Data::CPP_USBData_Init() {
    msg_from_minipc.start = 's';
    msg_from_minipc.end = 'e';
    msg_from_minipc.datatype = 0xA0; // 上位机发送数据类型
    msg_from_minipc.command = 0xFF;  // 命令字，0xFF表示无效命令
    for (int i = 0; i < 6; i++) {
        msg_from_minipc.data[i] = 0; // 初始化数据域 上位机数据域留空
    }

    msg_to_minipc.start = 's';
    msg_to_minipc.end = 'e';
    msg_to_minipc.datatype = 0xB0; // 下位机发送数据类型
    msg_to_minipc.command = 0xFF;  // 命令字，0xFF表示无效命令
    for (int i = 0; i < 6; i++) {
        msg_to_minipc.data[i] = 0; // 初始化数据域 下位机剩余数据域留空
    }
}

/**
 * @brief 处理数据
 */
void USB_Data::CPP_USBData_Process(void) {
    if (CPP_myUSBRxNum) {
        msg_cnt += 1; //接收到数据计数+1

        if (CPP_myUSBRxData[1] == 0xA6) {
            //满足上位机发送数据协议的处理方法
            msg_from_minipc.start = CPP_myUSBRxData[0];
            msg_from_minipc.datatype = CPP_myUSBRxData[1];
            msg_from_minipc.command = CPP_myUSBRxData[2];
            uint32_t data[6] = {0}; //数据域
            for (int i = 0; i < 6; i++) {
                data[i] = *(uint32_t*)(CPP_myUSBRxData+3+(i*4));
            }
            msg_from_minipc.end = CPP_myUSBRxData[31];

            char command = msg_from_minipc.command;
            if (command != 0xFF) {
                switch (command) {
                    case 0x01: // 位置信息
                        FeedBack_Position[0] = (float)data[0];
                        FeedBack_Position[1] = (float)data[1];
                        FeedBack_Position[2] = (float)data[2];
                        FeedBack_Position[3] = (float)data[3];
                        FeedBack_Position[4] = (float)data[4];
                        FeedBack_Position[5] = (float)data[5];
                        break;
                    case 0x02: // 速度信息
                        FeedBack_Velocity[0] = (float)data[0];
                        FeedBack_Velocity[1] = (float)data[1];
                        FeedBack_Velocity[2] = (float)data[2];
                        FeedBack_Velocity[3] = (float)data[3];
                        FeedBack_Velocity[4] = (float)data[4];
                        FeedBack_Velocity[5] = (float)data[5];
                        break;
                    default:
                        uint8_t msg_buf[32] = {0};
                        msg_buf[0] = 'E'; //Error开头
                        for (char i = 0; i < CPP_myUSBRxNum && i < 32; i++) {
                            msg_buf[i + 1] = CPP_myUSBRxData[i]; //将接收到的数据复制到msg_buf
                        }
                        uint32_t msg_len = CPP_myUSBRxNum + 1; //加上Error开头的字节
                        CPP_USBData_SendData(msg_buf, &msg_len);
                }
            }
            memset(CPP_myUSBRxData, 0, 32); //数据处理后清空缓存区
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
        memset(CPP_myUSBRxData, 0, 32); //数据处理后清空缓存区
        CPP_myUSBRxNum = 0;             //有利于判断，置0
    }
}

void USB_Data::CPP_USBData_GetData(uint8_t* Buf, uint32_t* Len) {
    memset(CPP_myUSBRxData, 0, 32);     //清空缓存区
    memcpy(CPP_myUSBRxData, Buf, *Len); //接收的数据复制到缓存区
    CPP_myUSBRxNum = *Len;              //复制字节数
}

void USB_Data::CPP_USBData_SendMsg(uint32_t *Data) {
    msg_to_minipc.start = 's';
    msg_to_minipc.end = 'e';
    msg_to_minipc.datatype = 0xC2; // 下位机发送数据类型

    msg_to_minipc.data[0] = Data[0];
    msg_to_minipc.data[1] = Data[1];
    msg_to_minipc.data[2] = Data[2];
    msg_to_minipc.data[3] = Data[3];
    msg_to_minipc.data[4] = Data[4];
    msg_to_minipc.data[5] = Data[5];

    CPP_myUSBTxNum = 32;
    CPP_myUSBTxData[0] = msg_to_minipc.start;
    CPP_myUSBTxData[1] = msg_to_minipc.datatype;
    // 将数据域转换为字节数组
    for (char i = 0; i < 6; i++) {
        for (char j = 0; j < 4; j++) {
            CPP_myUSBTxData[i * 4 + j + 2] = *((char*)(&msg_to_minipc.data[i]) + j);
        }
    }
    for (char i = 28; i < 30; i++) {
        //中间留空
        CPP_myUSBTxData[i] = 0x00;
    }
    CPP_myUSBTxData[31] = msg_to_minipc.end;

    if (CDC_Transmit_HS(CPP_myUSBTxData, CPP_myUSBTxNum) == USBD_OK) success_cnt++;
    memset(CPP_myUSBTxData, 0, 32); //数据处理后清空缓存区
    CPP_myUSBTxNum = 0;
}


void USB_Data::CPP_USBData_SendData(uint8_t* Buf, uint32_t* Len) {
    memset(CPP_myUSBTxData, 0, 32);
    memcpy(CPP_myUSBTxData, Buf, *Len);
    CPP_myUSBTxNum = *Len;
    CDC_Transmit_HS(Buf, *Len);
    memset(CPP_myUSBTxData, 0, 32);
}

USB_Data USB_Data1;

void CPP_USB_Task() {
    USB_Data1.CPP_USBData_Process();
    USB_SendMsg((uint32_t*)JointTorque);
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

void USB_SendMsg(uint32_t *Data) {
    USB_Data1.CPP_USBData_SendMsg(Data);
}

void USBData_init() {
    USB_Data1.CPP_USBData_Init();
}
}
