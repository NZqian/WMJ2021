#include "../include/StateMachine.h"

/*
 * @brief 自瞄状态 右键进入
 */
void StateMachine::armorStrike()
{
    int armor_id = -1;
    int continue_flag = false;
    int detect_num = 0;
    int undetect_num = 0;
    bool found_armor = false;
    double pitch_diff;
    double yaw_diff;
    int shootInterval = 0;
    int top_interval = 0;

    bool set_time = false;
    int FPS = 0;
    double tb, te;

    // wmj::Armor left_armor, right_armor;
    wmj::Armors armors;
    std::vector<wmj::MatWithTime> frames;
    m_armor_detector -> init();
    while (true)
    {
        // 测fps
        //if(!set_time)
        //{
        //set_time = true;
        //tb = wmj::now();
        //}

        //te = wmj::now();
        //FPS++;

        //if(1000.f*(te - tb) > 1000)
        //{
        //set_time = false;
        //std::cout << "FPS : " << FPS <<"\n";
        //FPS = 0;
        //}

        // 是否进行自动曝光
        if(m_robot_control->m_auto_exposure)
        {
            std::cout << "exposure true\n";
            StateMachine::setArmorExposure();
        }

        // 是否使用数字识别 本版本已删除
        //if(m_robot_control->m_use_number_detect && m_armor_detector->i_state == 4)
        //{
        //    m_armor_detector->i_state = 2;
        //    m_usb_capture->cameraMode("armor");
        //    if(m_auto_exposure_finished)
        //        m_usb_capture->setExposureTime(m_auto_exposure_value, wmj::MANUAL);
        //}
        //if(!m_robot_control->m_use_number_detect && m_armor_detector->i_state == 2)
        //{
        //    m_armor_detector->i_state = 4;
        //    m_usb_capture->cameraMode("dart");
        //}

        // 是否进入位姿预测模式
        if(m_robot_control->m_use_pose_predict)
        {
            m_predict_mod = StateMachine::POSE;
        }
        else
        {
            m_predict_mod = StateMachine::POINT;
        }

        // 是否使用默认曝光值
        if(m_robot_control->m_use_default_exposure)
        {
            m_usb_capture->cameraMode("armor");
            m_auto_exposure_finished = false;
        }

        if (!m_filter_init && (m_current_state == wmj::ROBO_STATE::STATE_ARMOR
                               || m_current_state == wmj::ROBO_STATE::STATE_DARK
                               || m_current_state == wmj::ROBO_STATE::STATE_TOP))
        {
            m_speed_resolver = std::make_shared<wmj::SpeedResolver>();
            m_motion_predictor = std::make_shared<wmj::MotionPredict>();
            m_pose_predictor = std::make_shared<wmj::PosePredict>();
            std::cout << "statemachine " << "Restart filter !!!" << std::endl;
            m_filter_init = true;
        }
        else if (m_current_state == wmj::ROBO_STATE::STATE_ARMOR || m_current_state == wmj::ROBO_STATE::STATE_TOP)
        {
            armor_id = m_armor_last.m_id ? m_armor_last.m_id : armor_id;
            std::cout  << "statemachine " << "当前机器人id: " << armor_id << std::endl;
        }
        else if (m_current_state == wmj::ROBO_STATE::STATE_RUNE)
        {
            std::cout  << "statemachine " << _cyan("自瞄线程启动失败，大符函数执行中") + "\n";
            return;
        }
        else
        {
            std::cout << "statemachine " << _cyan("自瞄线程启动失败") + "\n";
            return;
        }
        if (*m_usb_capture >> frames)
        {
            std::cout  << "statemachine " << _warning("自瞄未采集到图像 !!!") + "\n";
            return;
        }
        for (int i = 0; i < m_usb_capture->m_device_number; ++i)
        {
            if (frames[i].m_img.empty())
            {
                std::cout  << "statemachine " << _warning("自瞄获取到空图像 !!!") + "\n";
                continue_flag = true;
                continue;
            }
        }
        if (continue_flag)
        {
            continue_flag = 0;
            continue;
        }

        //for (int i = 0; i < m_usb_capture->m_device_number; ++i)
        //{
        //    cv::cvtColor(frames[i].m_img, frames[i].m_img, cv::COLOR_RGB2BGR);
        //}

        timeval tv;
        gettimeofday(&tv, NULL);
        long time_start = tv.tv_sec * 1000000 + tv.tv_usec;
        m_final_armor   = m_armor_detector->DetectArmorDouble(frames, armors);
        //m_final_armor   = m_armor_detector->DetectArmorDouble(frames, left_armor, right_armor);
        found_armor      = m_final_armor.m_armor_type != wmj::ARMOR_NONE;
        // if(m_detectID == -1 && found_armor)
        // {
        //     m_detectID = m_final_armor.m_id;
        // }
        // else if(found_armor && m_detectID != m_final_armor.m_id)
        // {
        //     found_armor = !found_armor;
        // }
        std::cout << "m_detectID:" << m_armor_detector->m_ArmorSingles[0].m_detectnum << std::endl;
        gettimeofday(&tv, NULL);
        long time_end = tv.tv_sec * 1000000 + tv.tv_usec;
        m_final_armor.m_position.x /= 1000;
        m_final_armor.m_position.y /= 1000;
        m_final_armor.m_position.z /= 1000;

        if (found_armor)
        {
            m_armor_last.m_id = m_final_armor.m_id;
            m_robot_control->SwitchBaseCoor(1);
            if(++detect_num > 10) undetect_num = 0; // 识别到后清零未识别计数
            if(m_debug)
            {
                std::cout << "statemachine " << _lightblue("识别到装甲板类型 :  ") + _yellow(std::to_string(m_final_armor.m_armor_type));
                std::cout << "statemachine " << _cyan("识别消耗时间 : ") + _yellow(std::to_string(time_end - time_start)) + "\n";
            }

            m_cur_pose = m_robot_control->GetGimbalAngle();
            if(m_debug)
            {
                std::cout << _lightblue("当前位姿 : ") << m_cur_pose << "\n";
            }

            // 云台系坐标转地面系
            m_armor_abs_point = m_angle_solver->cam2abs(m_final_armor.m_position, m_robot_control->GetGimbalAngle());
            if(m_debug)
            {
                std::cout << _yellow("装甲绝对坐标 : ") << m_armor_abs_point << std::endl;
            }

            if (m_current_state == wmj::ROBO_STATE::STATE_ARMOR
                    || m_current_state == wmj::ROBO_STATE::STATE_DARK
                    || m_current_state == wmj::ROBO_STATE::STATE_TOP)
            {
                if(m_robot_control->TopMode)
                {
                    // 解算位姿
                    m_top_aimer->setBulletSpeed(m_robot_control->getShootSpeedValue());
                    m_target_pose = m_top_aimer->getFinalPose(m_cur_pose, m_final_armor);
                    if(m_debug)
                    {
                        std::cout << _lightcyan("解算位姿 : ") << m_target_pose << "\n";
                    }

                    // 自动击发
                    if(m_robot_control->m_enable_shoot && m_top_aimer->m_shoot_flag && abs(m_cur_pose.yaw - m_target_pose.yaw) < 0.1)
                    {
                        std::cout << _lightpurple("<<< On Attack!!! >>>") << std::endl;
                        // control->ShootOnce();
                        top_interval--;
                        if (top_interval < 0)
                        {
                            std::cout << _purple("<<< Open fire!!! >>>") << std::endl;
                            m_robot_control->ShootOnce();
                            top_interval = 5;
                        }
                    }
                    else
                    {
                        m_robot_control->ShootNone();
                        top_interval = 0;
                    }
                }
                else
                {
                    // 预测位置
                    if(m_predict_mod == StateMachine::POINT)
                    {
                        m_final_armor.m_position = m_motion_predictor->predict(m_armor_abs_point);
                        if(m_debug)
                        {
                            std::cout << _yellow("滤波绝对坐标 : ") << m_final_armor.m_position << std::endl;
                        }
                        m_final_armor.m_position = m_angle_solver->abs2cam(m_final_armor.m_position, m_robot_control->GetGimbalAngle());
                    }
                    else // 预测位姿
                    {
                        m_target_pose = m_pose_predictor->predict(m_target_pose);
                        if(m_debug)
                        {
                            std::cout << _yellow("滤波位姿 : ") << m_target_pose << std::endl;
                        }
                    }

                    m_target_pose = m_angle_solver->getAngle(m_final_armor.m_position, m_cur_pose, m_robot_control->getShootSpeedValue());

                    // 射子弹
                    pitch_diff = abs(m_target_pose.pitch - m_cur_pose.pitch);
                    yaw_diff = abs(m_target_pose.yaw - m_cur_pose.yaw);
                    if(m_robot_control->m_enable_shoot)
                        shootInterval++;
                    else
                        shootInterval = 0;
                    if(yaw_diff < 0.04 && shootInterval >= 30)
                    {
                        m_robot_control->ShootSome(5);
                    }
                    else if(shootInterval == 1)
                    {
                        m_robot_control->ShootSome(1);
                    }
                    else
                    {
                        m_robot_control->ShootNone();
                    }
                }

                m_gm_speed = m_speed_resolver->resolve(m_target_pose, m_cur_pose, wmj::now());
                if(m_debug)
                {
                    std::cout << _yellow("解算速度 : ") << m_gm_speed << "\n";
                }

                m_robot_control->SetGimbal_YawSpeed_PitchAngle(m_target_pose.pitch, m_gm_speed.yaw);
                // m_robot_control->SetGimbalSpeed(m_gm_speed.pitch, m_gm_speed.yaw);
                // m_robot_control->SetGimbalAngle(m_target_pose.pitch, m_target_pose.yaw);

            }
            std::cout << _warning("************************************") + "\n";
        } // 未识别到
        else
        {
            top_interval = 0;
            undetect_num++;
            if(m_debug)
            {
                std::cout << "#########################################" << std::endl;
                std::cout << _red("未识别到装甲板!!!") << std::endl;
                std::cout << "#########################################" << std::endl;
            }

            m_cur_pose = m_robot_control->GetGimbalAngle();
            if(undetect_num > 3)
                detect_num = 0;
            if(undetect_num > 8)
                m_robot_control->SetGimbalAngle(m_cur_pose.pitch, m_cur_pose.yaw);
        }
        m_rate_control.sleep();
    }
}
