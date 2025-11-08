#include "Debug_Task.h"
#include "cmsis_os.h"
#include "dev_usb.h"
#include "usb_device.h"

void App_DebugTask(void const * argument) {

    // USB设备初始化
    MX_USB_DEVICE_Init();
    USBData_init();

    while (1) {
        USB_Task();
        osDelay(1);
    }
}
