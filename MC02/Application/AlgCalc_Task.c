#include "AlgCalc_Task.h"
#include "cmsis_os.h"
#include "Alg_calc.h"
#include "stm32h7xx_hal.h"
#include "core_cm7.h"
#include "alg_pid.h"
#include "bsp_log.h"

// DWT
static inline void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 允许 DWT
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // 启用 cycle counter
}

static inline uint32_t DWT_GetCycle(void) {
    return DWT->CYCCNT;
}

static inline float DWT_GetMicroseconds(uint32_t start, uint32_t end) {
    uint32_t cycles = end - start;
    return (float)cycles / (HAL_RCC_GetHCLKFreq() / 1e6f);
}

// DWT

PidInstance_s* Joint_Velocity_PidInstance[6];
PidInstance_s* Joint_Position_PidInstance[6];

PidInitConfig_s Joint_Position_PidConfig[6] = {
    {
        .kp = 1.0f,
        .ki = 0.5f,
        .kd = 0.0f,
        .i_max = 5.0f,
        .out_max = 20.0f,
    },
    {
        .kp = 1.0f,
        .ki = 0.5f,
        .kd = 0.0f,
        .i_max = 5.0f,
        .out_max = 20.0f,
    },
    {
        .kp = 1.0f,
        .ki = 0.5f,
        .kd = 0.0f,
        .i_max = 5.0f,
        .out_max = 20.0f,
    },
    {
        .kp = 1.0f,
        .ki = 0.5f,
        .kd = 0.0f,
        .i_max = 5.0f,
        .out_max = 20.0f,
    },
    {
        .kp = 1.0f,
        .ki = 0.5f,
        .kd = 0.0f,
        .i_max = 5.0f,
        .out_max = 20.0f,
    },
    {
        .kp = 1.0f,
        .ki = 0.5f,
        .kd = 0.0f,
        .i_max = 5.0f,
        .out_max = 20.0f,
    },
};
PidInitConfig_s Joint_Velocity_PidConfig[6] = {
    {
        .kp = 1.0f,
        .ki = 0.5f,
        .kd = 0.0f,
        .i_max = 5.0f,
        .out_max = 20.0f,
    },
    {
        .kp = 2.0f,
        .ki = 0.5f,
        .kd = 0.0f,
        .i_max = 5.0f,
        .out_max = 20.0f,
    },
    {
        .kp = 2.0f,
        .ki = 0.5f,
        .kd = 0.0f,
        .i_max = 5.0f,
        .out_max = 20.0f,
    },
    {
        .kp = 0.5f,
        .ki = 0.1f,
        .kd = 0.0f,
        .i_max = 3.0f,
        .out_max = 7.0f,
    },
    {
        .kp = 0.5f,
        .ki = 0.1f,
        .kd = 0.0f,
        .i_max = 3.0f,
        .out_max = 7.0f,
    },
    {
        .kp = 0.5f,
        .ki = 0.1f,
        .kd = 0.0f,
        .i_max = 3.0f,
        .out_max = 7.0f,
    },
};

void Arm_Init() {
    for (uint8_t i = 0; i < 6; i++) {
        Joint_Velocity_PidInstance[i] = NULL;
        Joint_Position_PidInstance[i] = NULL;
    }

    for (uint8_t i = 0; i < 6; i++) {
        while (Joint_Position_PidInstance[i] == NULL) {
            Joint_Position_PidInstance[i] = Pid_Register(&Joint_Position_PidConfig[i]);
        }
        while (Joint_Velocity_PidInstance[i] == NULL) {
            Joint_Velocity_PidInstance[i] = Pid_Register(&Joint_Velocity_PidConfig[i]);
        }
    }
}

float us;
extern float FeedBack_Velocity[6];
extern float FeedBack_Position[6];
extern float JointTorque[6];
float FeedForward_Torque[6];
UBaseType_t highwater = 0;

void App_AlgCalc(void const* argument) {
    DWT_Init();

    Arm_Init();

    while (1) {
        uint32_t start = DWT_GetCycle();
        // FeedForward_Torque = miniArm.rne(FeedBack_Position);
        Rne(FeedBack_Position, NULL, NULL, NULL, FeedForward_Torque);
        uint32_t end = DWT_GetCycle();

        us = DWT_GetMicroseconds(start, end);

        for (uint8_t i = 0; i < 6; i++) {
            JointTorque[i] = FeedForward_Torque[i];
        }

        highwater = uxTaskGetStackHighWaterMark(NULL);
        osDelay(1);
    }
}
