#include "../include/PidResolver.hpp"

namespace wmj
{
    PidResolver::PidResolver()
    {
        setParam();
    }

    void PidResolver::setParam()
    {
        cv::FileStorage fs(PID_CFG, cv::FileStorage::READ);
        fs["debug"]           >> m_debug;
        fs["error_threshold"] >> m_error_threshold;
        fs["yaw_kp"]          >> m_yaw_kp;
        fs["yaw_ki"]          >> m_yaw_ki;
        fs["yaw_kd"]          >> m_yaw_kd;
        fs["pit_kp"]          >> m_pit_kp;
        fs["pit_ki"]          >> m_pit_ki;
        fs["pit_kd"]          >> m_pit_kd;
        fs["yaw_kp2"]         >> m_yaw_kp2;
        fs["yaw_ki2"]         >> m_yaw_ki2;
        fs["yaw_kd2"]         >> m_yaw_kd2;
        fs["pit_kp2"]         >> m_pit_kp2;
        fs["pit_ki2"]         >> m_pit_ki2;
        fs["pit_kd2"]         >> m_pit_kd2;
        fs.release();
    }

    GimbalPose PidResolver::resolve(GimbalPose cur_error)
    {
        m_llast_error = m_last_error;
        m_last_error  = m_cur_error;
        m_cur_error   = cur_error;
        GimbalPose delta_pose{};

        if(std::fabs(cur_error.pitch) > std::fabs(m_error_threshold))
        {
            delta_pose.pitch = deltaPitch(m_pit_kp, m_pit_ki, m_pit_kd);
        }
        else
        {
            delta_pose.pitch = deltaPitch(m_pit_kp2, m_pit_ki2, m_pit_kd2);
        }

        if(std::fabs(cur_error.yaw) > std::fabs(m_error_threshold))
        {
            if(m_debug)
            {
                std::cout << "First pid thres is : " << m_error_threshold << "  " << cur_error.yaw << std::endl;
            }
            delta_pose.yaw = deltaYaw(m_yaw_kp, m_yaw_ki, m_yaw_kd);
        }
        else
        {
            if(m_debug)
            {
                std::cout << "Senond pid thres is : " << m_error_threshold << "  " << cur_error.yaw << std::endl;
            }
            delta_pose.yaw = deltaYaw(m_yaw_kp2, m_yaw_ki2, m_yaw_kd2);
        }

        m_last_output = delta_pose;
        return delta_pose;
    }

    float PidResolver::deltaPitch(float kp, float ki, float kd)
    {
        return  kp * (m_cur_error.pitch - m_last_error.pitch)
                + ki * m_cur_error.pitch
                + kd * (m_cur_error.pitch - m_last_error.pitch * 2.f + m_llast_error.pitch) + m_last_output.pitch;
    }

    float PidResolver::deltaYaw(float kp, float ki, float kd)
    {
        return  kp * (m_cur_error.yaw - m_last_error.yaw)
                + ki * m_cur_error.yaw
                + kd * (m_cur_error.yaw - m_last_error.yaw * 2.f + m_llast_error.yaw) + m_last_output.yaw;
    }
}
