#include "alg_calc/matrix.h"
#include "alg_calc/robotics.h"
#include "alg_calc/utils.h"
#include "alg_calc/user_function.hpp"

#include <cmath>
#include <iostream>

int main() {
    float mess[6] = {1.32f,0.649f,0.642f, 0.493f, 0.488f, 0.009};

    float RC[18] = {-0.011f, -0.027f, -0.012f, 0.0f, 0.0f, 0.0f,
                          0.038f, 0.0f, 0.008f, -0.02f, -0.08, 0.0f,
                          0.008f, -0.069f, -0.015f, -0.005f, 0.05f, 0.012f};
    Matrixf<3, 6> centroid(RC);

    Matrixf<3,3> I[6];
    I[0] = matrixf::eye<3, 3>() * 1.0f;
    I[1] = I[0];
    I[2] = I[0];
    I[3] = I[0];
    I[4] = I[0];
    I[5] = I[0];

    // DH计算出的theta需要写入offset中
    robotics::Link links[6];
    links[0] = robotics::Link(0, 0.09955, 0, -PI/2, robotics::R, 0);
    links[1] = robotics::Link(0,0,0.14,PI,robotics::R,-PI/2);
    links[2] = robotics::Link(0,0,0,-PI/2,robotics::R,-PI/2);
    links[3] = robotics::Link(0,0.153,0,PI/2, robotics::R, 0);
    links[4] = robotics::Link(0,0,0,-PI/2,robotics::R,0);
    links[5] = robotics::Link(0,0.0875,0,0,robotics::R,0);

    robotics::Serial_Link<6> miniArm(links);

    float q[6] = {0.0f, 0.1f, 0.2f, 0.3f, 0.4f, 0.5f};
    Matrixf<4, 4> T = miniArm.fkine(q);

    PrintMatrix(T, "T");

    return 0;
}
