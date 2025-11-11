/**
*   @file Alg_calc.cpp
*   @brief 本文件为c源的Task封装CPP函数后提供接口
*   @author Wenxin HU
*   @date 25-11-9
*   @version 1.0
*   @note
*/
#include "Alg_calc.h"

#include <sys/types.h>

#include "matrix.h"
#include "robotics.h"

float mess[6] = {1.32f, 0.649f, 0.642f, 0.493f, 0.488f, 0.009};
// link质心位置原始数据
float RC[18] = {
    -0.011f, -0.027f, -0.012f, 0.0f, 0.0f, 0.0f,
    0.038f, 0.0f, 0.008f, -0.02f, -0.08, 0.0f,
    0.008f, -0.069f, -0.015f, -0.005f, 0.05f, 0.012f
};
Matrixf<3, 6> centroid(RC);
Matrixf<3, 3> I[6] = {
    matrixf::eye<3, 3>() * 1.0f,
    matrixf::eye<3, 3>() * 1.0f,
    matrixf::eye<3, 3>() * 1.0f,
    matrixf::eye<3, 3>() * 1.0f,
    matrixf::eye<3, 3>() * 1.0f,
    matrixf::eye<3, 3>() * 1.0f
};
robotics::Link links[6] = {
    robotics::Link(0, 0.09955, 0.0, -PI / 2, robotics::R, 0.0,
                   -PI, PI, mess[0], centroid.col(0), I[0]),
    robotics::Link(0, 0.0, 0.14, PI, robotics::R, -PI / 2,
                   -PI / 2, PI / 2, mess[1], centroid.col(1), I[1]),
    robotics::Link(0, 0.0, 0.0, -PI / 2, robotics::R, -PI / 2,
                   -2.7925, 0.0, mess[2], centroid.col(2), I[2]),
    robotics::Link(0, 0.153, 0.0, PI / 2, robotics::R, 0.0,
                   -PI / 2, PI / 2, mess[3], centroid.col(3), I[3]),
    robotics::Link(0, 0.0, 0.0, -PI / 2, robotics::R, 0.0,
                   -2.3561, 2.3561, mess[4], centroid.col(4), I[4]),
    robotics::Link(0, 0.0875, 0.0, 0.0, robotics::R, 0.0,
                   -PI, PI, mess[5], centroid.col(5), I[5]),
};
robotics::Serial_Link<6> miniArm(links);

void Rne(float* _q,
         float* _qv,
         float* _qa,
         float* _he,
         float* _torque) {
    Matrixf<6,1>q = matrixf::zeros<6,1>();
    Matrixf<6,1>qv = matrixf::zeros<6,1>();
    Matrixf<6,1>qa = matrixf::zeros<6,1>();
    Matrixf<6,1>he = matrixf::zeros<6,1>();
    // 输入转换
    if (_q != NULL) {
        q = _q;
    } else {
        return;
    }
    if (_qv != NULL) qv = _qv;
    if (_qa != NULL) qa = _qa;
    if (_he != NULL) he = _he;
    if (_torque == NULL) return;

    Matrixf<6, 1>torque = miniArm.rne(q,qv, qa, he);

    for (uint8_t i = 0; i < 6; i++) {
        _torque[i] = torque[i][0];
    }
}
