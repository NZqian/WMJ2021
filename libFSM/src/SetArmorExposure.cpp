#include "../include/StateMachine.h"
#include <opencv2/core/persistence.hpp>
#include <string>

/*
 * @brief 自动曝光 右键+Z进入
 */
void StateMachine::setArmorExposure()
{
    if(m_auto_exposure_finished)
        return;
    m_armor_detector->m_ArmorSingles[0].m_param.m_enemy_color = !m_armor_detector->m_ArmorSingles[0].m_param.m_enemy_color;
    m_armor_detector->m_ArmorSingles[1].m_param.m_enemy_color = !m_armor_detector->m_ArmorSingles[1].m_param.m_enemy_color;
    cv::FileStorage fsWrite("AutoExposure_" + to_string(m_repeat_exposure_time) + ".yaml", cv::FileStorage::WRITE);
    m_auto_exposure_value = 100;
    m_usb_capture->setExposureTime(m_auto_exposure_value, wmj::MANUAL);
    usleep(10000);
    bool flag =  false;
    bool conti_flag = false;
    bool first_detected = false;
    bool start_check = false;
    wmj::Armors leftArmors;
    wmj::Armors rightArmors;
    wmj::Armor left_lastArmor;
    wmj::Armor right_lastArmor;
    int add_exposure = 100;
    int success_count = 0;
    Rect left_rect = Rect(Point2f(0, 0), Point2f(1280, 1024));
    Rect right_rect = Rect(Point2f(100, 100), Point2f(1280, 1024));
    left_lastArmor.m_rect = left_rect;
    right_lastArmor.m_rect = right_rect;
    m_armor_detector->m_ArmorSingles[0].m_state = wmj::TRACKING;
    m_armor_detector->m_ArmorSingles[1].m_state = wmj::TRACKING;
    m_armor_detector->m_ArmorSingles[0].m_last_armor = left_lastArmor;
    m_armor_detector->m_ArmorSingles[1].m_last_armor = right_lastArmor;

    while(1)
    {
        if(*m_usb_capture >> m_frames)
        {
            std::cout << "获取图像失败" << endl;
        }
        for (int i = 0; i < m_usb_capture->m_device_number; ++i)
        {
            if (m_frames[i].m_img.empty())
            {
                std::cout << "Not load empty !!!\n";
                conti_flag = true;
                continue;
            }
        }
        if(conti_flag)
        {
            conti_flag = 0;
            continue;
        }
        m_armor_detector->m_ArmorSingles[0].m_src = m_frames[0].m_img;
        m_armor_detector->m_ArmorSingles[1].m_src = m_frames[1].m_img;
        m_armor_detector->m_ArmorSingles[0].ArmorSingleDetector();
        m_armor_detector->m_ArmorSingles[1].ArmorSingleDetector();

        if(m_armor_detector->m_ArmorSingles[0].m_armors.size() > 0 && m_armor_detector->m_ArmorSingles[1].m_armors.size() > 0)
        {
            if(success_count >= 10)
            {
                flag = true;
                break;
            }
            success_count++;
            start_check = true;
        }
        else
        {
            success_count = 0;
            start_check = false;
        }
        if(m_auto_exposure_value > 12000)
        {
            break;
        }
        std::cout << _yellow(">>>>>>>>>>>>>>>current exposure : ") << m_auto_exposure_value << std::endl;
        if(!start_check)
            m_auto_exposure_value += add_exposure;
        m_usb_capture->setExposureTime(m_auto_exposure_value, wmj::MANUAL);
    }

    if(flag)
    {
        m_usb_capture->setExposureTime(m_auto_exposure_value, wmj::MANUAL);
        std::cout << "exposure ok 找到目标曝光值为：" << m_auto_exposure_value << std::endl;
        fsWrite << "if_auto_exposured" << 1;
        fsWrite << "final_exposure" << m_auto_exposure_value;
        fsWrite.release();
        m_repeat_exposure_time++;
        m_auto_exposure_finished = true;
    }
    else
    {
        m_usb_capture->cameraMode("armor");
        std::cout << "exposure no 未找到目标曝光值" << std::endl;
        fsWrite << "if_auto_exposured" << 1;
        fsWrite << "final_exposure" << m_auto_exposure_value;
        fsWrite.release();
        m_repeat_exposure_time++;
        m_auto_exposure_finished = true;
    }
    m_armor_detector->m_ArmorSingles[0].m_param.m_enemy_color = !m_armor_detector->m_ArmorSingles[0].m_param.m_enemy_color;
    m_armor_detector->m_ArmorSingles[1].m_param.m_enemy_color = !m_armor_detector->m_ArmorSingles[1].m_param.m_enemy_color;

    return;
}
