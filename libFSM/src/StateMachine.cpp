#include "../include/StateMachine.h"

/*
 * @brief 状态机初始化函数
 */
StateMachine::StateMachine()
{
    m_robot_control    = std::make_shared<wmj::WMJRobotControl>();
    m_usb_capture      = std::make_shared<wmj::UsbCapture>();
    m_angle_solver     = std::make_shared<wmj::AngleSolver>();
    m_speed_resolver   = std::make_shared<wmj::SpeedResolver>();
    m_motion_predictor = std::make_shared<wmj::MotionPredict>();
    m_armor_detector   = std::make_shared<wmj::ArmorDetectorDouble>();
    m_rune_detector    = std::make_shared<wmj::ArmorTrigger>();
    m_top_aimer        = std::make_shared<wmj::AngleAimer>();

    m_usb_capture->cameraMode("armor");
    m_recorders.resize(m_usb_capture->m_device_number);

    timeval tv;
    gettimeofday(&tv, NULL);
    m_time = tv.tv_sec * 1000000 + tv.tv_usec;

    if(m_usb_capture->m_device_number == 2)
    {
        int left;
        if(m_usb_capture->m_orientation[0] == "left")
        {
            left = 0;
        }
        else
        {
            left = 1;
        }
        //        m_recorders[left] = cv::VideoWriter("../video/left"+to_string(m_time)+".avi", CV_FOURCC('M', 'J', 'P', 'G'), 60, cv::Size(1280, 1024), true);
        //        m_recorders[(left+1)%2] = cv::VideoWriter("../video/right"+to_string(m_time)+".avi", CV_FOURCC('M', 'J', 'P', 'G'), 60, cv::Size(1280, 1024), true);
    }
    else
    {
        //        m_recorders[0] = cv::VideoWriter("../video/single"+to_string(m_time)+".avi", CV_FOURCC('M', 'J', 'P', 'G'), 60, cv::Size(1280, 1024), true);
    }

    this->m_sm_status = true;
    m_robot_control->SwitchBaseCoor(true);
    this->setParam();
}

/*
 * @brief 设置各项参数
 */
void StateMachine::setParam()
{
    return;
}

/*
 * @brief 状态机循环
 */
void StateMachine::stateMachineLoop()
{
    bool armorReady = false;
    bool runeReady = false;
    bool darkReady = false;
    usleep(500000);

    while (m_sm_status)
    {
        m_robo_state = m_robot_control->GetRobotStatus();
        //m_robo_state = wmj::ROBO_STATE::STATE_ARMOR;

        switch (m_robo_state)
        {

            case wmj::ROBO_STATE::STATE_ARMOR:
            {
                m_robot_control->SwitchBaseCoor(true);
                m_current_state = wmj::ROBO_STATE::STATE_ARMOR;
                std::cout << _lightgreen("sm 自瞄控制状态") + "\n";

                if (!armorReady)
                {
                    m_usb_capture->cameraMode("armor");
                    //std::thread(&StateMachine::videoRecoder ,this).detach();
                    m_armor_detector->i_state = wmj::State::Double;
                    std::thread(&StateMachine::armorStrike, this).detach();
                    armorReady = true;
                }
                break;
            }
            case wmj::ROBO_STATE::STATE_RUNE:
            {
                m_robot_control->SwitchBaseCoor(true);
                m_current_state = wmj::ROBO_STATE::STATE_RUNE;
                std::cout << _lightgreen("sm 小符控制状态") + "\n";
                if (!runeReady)
                {
                    m_usb_capture->cameraMode("rune");
                    m_robot_control->SetGimbalSpeed(0, 0);
                    runeReady = true;
                }
                runeStrike(0);
                break;
            }
            case wmj::ROBO_STATE::STATE_RUNE_SINE:
            {
                m_robot_control->SwitchBaseCoor(true);
                m_current_state = wmj::ROBO_STATE::STATE_RUNE_SINE;
                std::cout << _lightgreen("sm 大符控制状态") + "\n";
                if (!runeReady)
                {
                    m_usb_capture->cameraMode("rune");
                    m_robot_control->SetGimbalSpeed(0, 0);
                    runeReady = true;
                }
                runeStrike(1);
                break;
            }
            //删除无数字识别模式 右键+X给位姿预测模式
            //case wmj::ROBO_STATE::STATE_DARK:
            //{
            //    m_robot_control->SwitchBaseCoor(true);
            //    m_current_state = wmj::ROBO_STATE::STATE_DARK;
            //    std::cout << _lightgreen("sm 无数字识别自瞄控制状态") + "\n";
            //    if(!darkReady)
            //    {
            //        m_usb_capture->cameraMode("dart");
            //        m_armor_detector->i_state = wmj::State::SentryAttack;
            //        std::thread(&StateMachine::armorStrike, this).detach();
            //        darkReady = true;
            //    }
            //    break;
            //}
            case wmj::ROBO_STATE::STATE_TOP:
            {
                m_robot_control->SwitchBaseCoor(true);
                m_current_state = wmj::ROBO_STATE::STATE_TOP;
                std::cout << _lightgreen("sm 反陀螺自瞄状态") + "\n";
                if(!armorReady)
                {
                    m_usb_capture->cameraMode("armor");
                    m_armor_detector->i_state = wmj::State::Double;
                    std::thread(&StateMachine::armorStrike, this).detach();
                    armorReady = true;
                }
                break;
            }
            default:
            {
                m_armor_detector->i_state = wmj::State::Double;
                m_current_state = wmj::ROBO_STATE::STATE_IDLE;

                int exposure_time = (m_exposure + 3) * 100;
                m_exposure = (m_exposure + 1) % 48;
                m_usb_capture->setExposureTime(exposure_time, wmj::MANUAL);

                if (*m_usb_capture >> m_frames)
                {
                    std::cout  << "statemachine " << _warning("未采集到图像 !!!") + "\n";
                    continue;
                }

                //               for(int i = 0; i < m_usb_capture->m_device_number; i++)
                //               {
                //	          putText(m_frames[i].m_img,std::to_string(exposure_time),cv::Point(50,50),cv::FONT_HERSHEY_PLAIN,2,cv::Scalar(0,0,255));
                //                   m_recorders[i].write(m_frames[i].m_img);
                //                   std::cout<<_lightblue("Recording!!")<<std::endl;
                //               }
                m_filter_init   = false;
                runeReady       = false;
                armorReady      = false;
                //m_detectID      = -1;
                std::cout << _lightblue("sm 待机状态") + "\n";
                break;
            }
        }
        m_rate_smach.sleep();
    }
    m_robot_control->SetGimbalAngle(0, 0);
}
