#pragma once
#include "PidResolver.hpp"
#include <opencv2/opencv.hpp>

namespace wmj
{

    class SpeedResolver
    {
    public:
        SpeedResolver();
        ~SpeedResolver() {}
        void setParam();
        void clearError();
        GimbalPose resolve(GimbalPose, GimbalPose, double);

    private:
        double m_last_timestamp;
        double m_cur_timestamp;
        GimbalPose  m_last_pose = 0;
        GimbalPose  m_cur_pose  = 0;
        PidResolver m_pid;
    };

}
