/**
*   @file Alg_calc.h
*   @brief 本文件为c源的Task封装CPP函数后提供接口
*   @author Wenxin HU
*   @date 25-11-9
*   @version 1.0
*   @note
*/
#ifndef ALG_CALC_H
#define ALG_CALC_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 适配C使用的牛顿欧拉动力学解算
 * @param _q 长度为6的float数组，当前关节位置
 * @param _qv 长度为6的float数组，当前关节速度
 * @param _qa 长度为6的float数组，当前关节加速度
 * @param _he 长度为6的float数组，末端负载
 * @param torque 长度为6的float数组，接收计算出的关节力矩
 */
void Rne(float* _q,
         float* _qv,
         float* _qa,
         float* _he,
         float* _torque);

#ifdef __cplusplus
}
#endif

#endif //ALG_CALC_H
