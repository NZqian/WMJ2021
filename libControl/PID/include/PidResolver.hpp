#pragma once
#include <iostream>
#include "../../../libBase/include/common.h"

namespace wmj
{
    class PidResolver
    {
    public:
        PidResolver();
        ~PidResolver() {}
        void setParam();
        GimbalPose resolve(GimbalPose cur_error);

        int m_debug;
        GimbalPose m_cur_error   = 0;       // 当前的偏差
        GimbalPose m_last_error  = 0;       // 上一帧的偏差
        GimbalPose m_llast_error = 0;       // 上上帧的偏差
        GimbalPose m_last_output = 0;       // 上一次计算得出的控制量

    private:
        float m_error_threshold;
        float m_yaw_kp;  // first
        float m_yaw_ki;
        float m_yaw_kd;
        float m_pit_kp;
        float m_pit_ki;
        float m_pit_kd;
        float m_yaw_kp2; // second
        float m_yaw_ki2;
        float m_yaw_kd2;
        float m_pit_kp2;
        float m_pit_ki2;
        float m_pit_kd2;

        float deltaPitch(float kp, float ki, float kd);
        float deltaYaw(float kp, float ki, float kd);
    };
}
