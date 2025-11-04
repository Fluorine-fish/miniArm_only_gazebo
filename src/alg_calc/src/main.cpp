#include "alg_calc/matrix.h"
#include "alg_calc/robotics.h"
#include "alg_calc/utils.h"

#include <iostream>

int main() {
    // /* Example 1: PUMA560 ------------------------------------------------------------------------*/
    // float m[6] = {0.2645f, 0.17f, 0.1705f, 0.0f, 0.0f, 0.0f};

    // /* 使用具名的 C 数组，避免 compound-literal */
    // float rc_data[18] = {
    //     0.0f, -8.5e-2f, 0.0f, 0.0f, 0.0f, 0.0f,
    //     13.225e-2f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    //     0.0f, 3.7e-2f, 8.525e-2f, 0.0f, 0.0f, 0.0f
    // };
    // Matrixf<3, 6> rc(rc_data);

    // Matrixf<3, 3> I[6];
    // float I0_data[3] = {1.542e-3f, 0.0f, 1.542e-3f};
    // float I1_data[3] = {0.0f, 0.409e-3f, 0.409e-3f};
    // float I2_data[3] = {0.413e-3f, 0.413e-3f, 0.0f};
    // I[0] = matrixf::diag<3, 3>(I0_data);
    // I[1] = matrixf::diag<3, 3>(I1_data);
    // I[2] = matrixf::diag<3, 3>(I2_data);
    // I[3] = matrixf::eye<3, 3>() * 3.0f;
    // I[4] = matrixf::eye<3, 3>() * 2.0f;
    // I[5] = matrixf::eye<3, 3>() * 1.0f;

    // robotics::Link links[6];
    // links[0] = robotics::Link(0, 26.45e-2f, 0, -PI / 2, robotics::R, 0, 0, 0, m[0], rc.col(0), I[0]);
    // links[1] = robotics::Link(0, 5.5e-2f, 17e-2f, 0, robotics::R, 0, 0, 0, m[1], rc.col(1), I[1]);
    // links[2] = robotics::Link(0, 0, 0, -PI / 2, robotics::R, 0, 0, 0, m[2], rc.col(2), I[2]);
    // links[3] = robotics::Link(0, 17.05e-2f, 0, PI / 2, robotics::R, 0, 0, 0, m[3], rc.col(3), I[3]);
    // links[4] = robotics::Link(0, 0, 0, -PI / 2, robotics::R, 0, 0, 0, m[4], rc.col(4), I[4]);
    // links[5] = robotics::Link(0, 0, 0, 0, robotics::R, 0, 0, 0, m[5], rc.col(5), I[5]);
    // robotics::Serial_Link<6> p560(links);

    // float q[6] = {0.2f, -0.5f, -0.3f, -0.6f, 0.5f, 0.2f};
    // float qv[6] = {1.0f, 0.5f, -1.0f, 0.3f, 0.0f, -1.0f};
    // float qa[6] = {0.2f, -0.3f, 0.1f, 0.0f, -1.0f, 0.0f};
    // float he[6] = {1.0f, 2.0f, -3.0f, -0.5f, -2.0f, 1.0f};

    // Matrixf<4, 4> T = p560.fkine(q);
    // Matrixf<6, 6> J = p560.jacob(q);

    // /* 用具名数组作为初始猜测，避免临时数组取地址问题 */
    // float ik_guess[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f};
    // Matrixf<6, 1> q_ikine = p560.ikine(T, Matrixf<6, 1>(ik_guess));
    // Matrixf<6, 1> torq = p560.rne(q, qv, qa, he);

    // /* Example 2: UR -----------------------------------------------------------------------------*/
    // // ...（保持原注释示例）...

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

    robotics::Link links[6];
    links[0] = robotics::Link(0, 0.09955f, 0.0f, -PI/2, robotics::R,
                0.0, -PI, PI, mess[0], centroid.col(0), I[0]);
    links[1] = robotics::Link(-PI/2, 0.0f, 0.14f, PI, robotics::R,
                0.0, -1.157, 1.157, mess[1], centroid.col(1), I[1]);
    links[2] = robotics::Link(-PI/2, 0.0f, 0.0f, -PI/2, robotics::R,
                0.0, 0.0f, 0.0f, mess[2], centroid.col(2), I[2]);
    links[3] = robotics::Link(0, 0.153f, 0.0f, PI/2, robotics::R,
                0.0, -PI/2, PI/2, mess[3], centroid.col(3), I[3]);
    links[4] = robotics::Link(0, 0.0f, 0.0f, -PI/2, robotics::R,
                0.0, -2.3562, 2.3561, mess[4], centroid.col(4), I[4]);
    links[5] = robotics::Link(0, 0.0875f, 0.0f, 0.0f, robotics::R,
                0.0, -PI, PI, mess[5], centroid.col(5), I[5]);
    robotics::Serial_Link<6> miniArm(links);

    float q[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    Matrixf<4, 4> T = miniArm.fkine(q);

    std::cout << "Matrix T: \n";
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            std::cout << T[i][j] << " ";
        }
        std::cout << "\n";
    }

    return 0;
}
