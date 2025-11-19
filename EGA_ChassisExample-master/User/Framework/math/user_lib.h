//
// Created by zhangzhiwen on 25-10-21.
//

#ifndef USER_LIB_H
#define USER_LIB_H
#include <math.h>

/**
 * @brief   RPM转为rad/s
 */
inline float _rpmToRad(float rpm)
{
    return (float)(rpm * 2 * M_PI / 60.0f);
}

/**
 * @brief   °/s转为rad/s
 */
inline float _angleToRad(float angle)
{
    return (float)(angle * M_PI / 180.0f);
}

/**
 * @brief   rad/s转为RPM
 */
inline float _radToRpm(float rad)
{
    return (float)(rad / (2 * M_PI) * 60.0f);
}

/**
 * @brief   弧度转为角度
 */
inline float _radToAngle(float rad)
{
    return (float)(rad * 180.0f / M_PI);
}

/**
 * @brief   检测速度是否超限
 * @details 传入一个四维数组，返回其是否超限
 * @retval  是否超限(true/false)
 */
inline bool _isSpdOutRange(float s[], float max_spd)
{
    return (fabsf(s[0]) > max_spd || fabsf(s[1]) > max_spd ||
        fabsf(s[2]) > max_spd || fabsf(s[3]) > max_spd);
}

#endif //USER_LIB_H
