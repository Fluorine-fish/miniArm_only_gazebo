#include "Debug_Task.h"
#include "cmsis_os.h"
#include "dev_usb.h"
#include "usb_device.h"

void App_DebugTask(void const * argument) {

    MX_USB_DEVICE_Init();

    while (1) {
        // USB_Task();
        osDelay(1);
    }
}
