#include "Debug_Task.h"
#include "cmsis_os.h"
#include "dev_usb.h"

void App_DebugTask(void const * argument) {

    while (1) {
        USB_Task();
        osDelay(1);
    }
}
