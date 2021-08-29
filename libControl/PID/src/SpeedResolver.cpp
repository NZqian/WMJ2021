#include "../include/SpeedResolver.hpp"

namespace wmj
{
    SpeedResolver::SpeedResolver()
    {
        setParam();
    }

    void SpeedResolver::setParam()
    {
        m_pid.setParam();
    }

    void SpeedResolver::clearError()
    {
        m_pid.m_llast_error = 0;
        m_pid.m_last_error  = 0;
        m_pid.m_cur_error   = 0;
        m_pid.m_last_output = 0;
        if(m_pid.m_debug)
        {
            std::cout << _lightcyan("上次输出 : ") << m_pid.m_last_output << "\n";
            std::cout << _warning("清零误差") << "\n";
        }
    }

    GimbalPose SpeedResolver::resolve(GimbalPose target_pose, GimbalPose cur_pose, double cur_time)
    {
        m_last_pose  = m_cur_pose;
        m_cur_pose   = target_pose;
        m_last_timestamp = m_cur_timestamp;
        m_cur_timestamp  = cur_time;

        GimbalPose base_speed = (m_cur_pose - m_last_pose) / (m_cur_timestamp - m_last_timestamp) ;

        if(m_pid.m_debug)
        {
            std::cout << _warning("基准速度") << base_speed << "\n";
        }
        GimbalPose aug_speed = m_pid.resolve(target_pose - cur_pose) ;
        if(m_pid.m_debug)
        {
            std::cout << _reverse(_warning("增量速度")) << aug_speed << "\n";
        }

        return base_speed / 1.f + aug_speed / 1.f;
        // return aug_speed / 1.f;
    }
}
