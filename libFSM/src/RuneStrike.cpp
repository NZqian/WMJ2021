#include "../include/StateMachine.h"

/*
 * @brief 能量机关状态
 * 
 * @param state为true则大符 state为false则小符
 */
void StateMachine::runeStrike(bool state)
{
    bool found_rune = false;
    wmj::GimbalPose center_pose, refer_pose;
    std::vector<wmj::MatWithTime> frames;

    if (*m_usb_capture >> frames)
    {
        std::cout << "statemachine " << _warning("大符未采集到图像 !!!") + "\n";
        return;
    }

    for (int i = 0; i < m_usb_capture->m_device_number; ++i)
    {
        if (frames[i].m_img.empty())
        {
            std::cout << "statemachine " << _warning("大符获取到空图像 !!!") + "\n";
            return;
        }
    }

    double shoot_speed = m_robot_control->getShootSpeedValue();
    std::cout << _lightblue("shoot speed :") << shoot_speed << std::endl;
    double predict_time = 7.0 / std::cos(0.21) / shoot_speed + 0.11;
    std::cout << _lightblue("predict time :") << predict_time << std::endl;
    m_rune_detector->setPredictTime(predict_time);

    // wmj::ROBO_ENERGY energy_status = m_robot_control->getEnergyStatus();
    // energy_status = wmj::ENERGY_BIG;

    wmj::ROBO_ENERGY energy_status;
    if (state)
        energy_status = wmj::ENERGY_BIG;
    else
        energy_status = wmj::ENERGY_SMALL;
    if (m_debug)
        std::cout << _blue("能量机关状态 :") << energy_status << std::endl;
    if (energy_status == 0 || energy_status == 3)
    {
        std::cout << _warning("能量机关不在状态") << std::endl;
        return;
    }

    timeval tv;
    gettimeofday(&tv, NULL);
    long time_start = tv.tv_sec * 1000000 + tv.tv_usec;
    m_final_armor.m_position = m_rune_detector->findTarget(frames, energy_status);
    found_rune = !(m_final_armor.m_position.x == -1 && m_final_armor.m_position.y == -1 && m_final_armor.m_position.z == -1);
    gettimeofday(&tv, NULL);
    long time_end = tv.tv_sec * 1000000 + tv.tv_usec;

    if (found_rune)
    {
        m_robot_control->SwitchBaseCoor(1);

        if (m_debug)
            std::cout << "statemachine " << _cyan("识别消耗时间 : ") + _yellow(std::to_string(time_end - time_start)) + "\n";

        m_cur_pose = m_robot_control->GetGimbalAngle();
        if (m_debug)
            std::cout << _lightblue("当前位姿 : ") << m_cur_pose << "\n";

        center_pose = m_angle_solver->getAngle(m_rune_detector->m_center_position, m_cur_pose);
        refer_pose.yaw = m_cur_pose.yaw - center_pose.yaw;
        refer_pose.pitch = m_cur_pose.pitch;
        cv::Point3f center_abs = m_angle_solver->cam2abs(m_rune_detector->m_center_position, refer_pose);
        cv::Point3f armor_abs = m_angle_solver->cam2abs(m_rune_detector->m_armor_position, refer_pose);
        if (m_debug)
        {
            std::cout << _purple("abs center :") << center_abs << std::endl;
            std::cout << _cyan("abs actual :") << armor_abs << std::endl;
        }

        // double predict_time = center_abs.x / std::cos(center_pose.pitch) / shoot_speed + 0.13;
        // m_rune_detector->recomputePredictAngle(predict_time);

        float predict_angle = (m_rune_detector->m_left_predict_angle + m_rune_detector->m_right_predict_angle) / 2;
        if (m_debug)
            std::cout << _blue("predict angle: ") << m_rune_detector->m_left_predict_angle << " " << m_rune_detector->m_right_predict_angle << '\n'
                      << predict_angle << std::endl;
        cv::Point3f predict_abs;
        predict_abs.x = armor_abs.x;
        predict_abs.y = (armor_abs.y - center_abs.y) * cos(-predict_angle * CV_PI / 180) - (armor_abs.z - center_abs.z) * sin(-predict_angle * CV_PI / 180) + center_abs.y;
        predict_abs.z = (armor_abs.y - center_abs.y) * sin(-predict_angle * CV_PI / 180) + (armor_abs.z - center_abs.z) * cos(-predict_angle * CV_PI / 180) + center_abs.z;
        if (m_debug)
            std::cout << _green("abs predict :") << predict_abs << std::endl;

        m_final_armor.m_position = m_angle_solver->abs2cam(predict_abs, refer_pose);

        m_target_pose = m_angle_solver->getAngle(m_final_armor.m_position, m_cur_pose, shoot_speed);
        m_target_pose.pitch += m_rune_detector->m_pitch_off;

        if (m_debug)
            std::cout << _lightcyan("解算位姿 : ") << m_target_pose << "\n";

        m_robot_control->SetGimbalAngle(m_target_pose.pitch, m_target_pose.yaw);
        std::cout << _warning("************************************") + "\n";
    }
    else
    {
        if (m_debug)
        {
            std::cout << "#########################################" << std::endl;
            std::cout << _red("未识别到大符!!!") << std::endl;
            std::cout << "#########################################" << std::endl;
        }
        m_robot_control->SetGimbalSpeed(0.f, 0.f);
    }
    m_rate_control.sleep();
}
