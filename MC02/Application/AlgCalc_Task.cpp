#include "AlgCalc_Task.h"
#include "matrix.h"
#include "robotics.h"
#include "cmsis_os.h"

#include "stm32h7xx_hal.h"
#include "core_cm7.h"
#include <random>

#include "bsp_log.h"

// DWT
static inline void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 允许 DWT
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // 启用 cycle counter
}

static inline uint32_t DWT_GetCycle(void) {
    return DWT->CYCCNT;
}

static inline float DWT_GetMicroseconds(uint32_t start, uint32_t end) {
    uint32_t cycles = end - start;
    return (float)cycles / (HAL_RCC_GetHCLKFreq() / 1e6f);
}
// DWT

float randomFloat(float min, float max, int seed) {
    static std::mt19937 gen(seed);
    std::uniform_real_distribution<float> dist(min, max);
    return dist(gen);
}

float mess[6] = {1.32f,0.649f,0.642f, 0.493f, 0.488f, 0.009};
// link质心位置原始数据
float RC[18] = {-0.011f, -0.027f, -0.012f, 0.0f, 0.0f, 0.0f,
                0.038f, 0.0f, 0.008f, -0.02f, -0.08, 0.0f,
                0.008f, -0.069f, -0.015f, -0.005f, 0.05f, 0.012f};
Matrixf<3,6> centroid(RC);
Matrixf<3,3> I[6] = {
    matrixf::eye<3,3>() *1.0f,
    matrixf::eye<3,3>() *1.0f,
    matrixf::eye<3,3>() *1.0f,
    matrixf::eye<3,3>() *1.0f,
    matrixf::eye<3,3>() *1.0f,
    matrixf::eye<3,3>() *1.0f};
robotics::Link links[6] = {
    robotics::Link(0, 0.09955, 0.0, -PI/2, robotics::R, 0.0,
                    -PI, PI, mess[0], centroid.col(0), I[0]),
    robotics::Link(0, 0.0,    0.14, PI,   robotics::R, -PI/2,
                    -PI/2, PI/2, mess[1], centroid.col(1), I[1]),
    robotics::Link(0, 0.0,    0.0,  -PI/2, robotics::R, -PI/2,
                    -2.7925, 0.0, mess[2], centroid.col(2), I[2]),
    robotics::Link(0, 0.153,  0.0,  PI/2,  robotics::R, 0.0,
                    -PI/2, PI/2, mess[3], centroid.col(3), I[3]),
    robotics::Link(0, 0.0,    0.0,  -PI/2, robotics::R, 0.0,
                    -2.3561, 2.3561, mess[4], centroid.col(4), I[4]),
    robotics::Link(0, 0.0875, 0.0,  0.0,   robotics::R, 0.0,
                       -PI, PI, mess[5], centroid.col(5), I[5]),
    };
robotics::Serial_Link<6> miniArm(links);
Matrixf<6,1> FeedForward_Torque;

float us;
extern float FeedBack_Velocity[6];
extern float FeedBack_Position[6];
extern float JointTorque[6];
UBaseType_t highwater = 0;

void App_AlgCalc(void const * argument) {

    DWT_Init();

    while (1) {

        // 生成随机序列
        FeedBack_Position[0] = randomFloat(-3.1415, 3.1415, 0);
        FeedBack_Position[1] = randomFloat(-0.3, 1.1570, 1);
        FeedBack_Position[2] = randomFloat(-2.6, 1.1570, 2);
        FeedBack_Position[3] = randomFloat(-2.6, 0.0, 3);
        FeedBack_Position[4] = randomFloat(-1.2, 1.2, 4);
        FeedBack_Position[5] = randomFloat(-3.1415, 3.1415, 5);

        uint32_t start = DWT_GetCycle();
        FeedForward_Torque = miniArm.rne(FeedBack_Position);
        uint32_t end = DWT_GetCycle();

        us = DWT_GetMicroseconds(start, end);

        for (uint8_t i = 0; i < 6; i++) {
            JointTorque[i] = FeedForward_Torque[i][0];
        }

        highwater = uxTaskGetStackHighWaterMark(NULL);
        osDelay(1);
    }
}

