/**
 * 自瞄测试文件
 */
#include "../libFSM/include/StateMachine.h"

// #define SAVEPOINT
#define AIMERTYPE 0 // 0 for AngleAimer, 1 for StaticAimer

int main(int argc, char **argv)
{
    bool use_control = true;
    bool use_speed = false;
    bool use_aimer = true;
    switch (argc)
    {
    case 2:
        use_speed = true;
        break;
    default:
        break;
    }

    char c = 'f';
    std::thread(monitorKeyboard, &c).detach();
    std::shared_ptr<wmj::UsbCapture> camera = std::make_shared<wmj::UsbCapture>();
    camera->cameraMode("armor");

    std::shared_ptr<wmj::WMJRobotControl>     control;
    control = std::make_shared<wmj::WMJRobotControl>();
    std::shared_ptr<wmj::AngleSolver>         angle       = std::make_shared<wmj::AngleSolver>();
    std::shared_ptr<wmj::SpeedResolver>       speed       = std::make_shared<wmj::SpeedResolver>();
    std::shared_ptr<wmj::MotionPredict>       predictor   = std::make_shared<wmj::MotionPredict>();
    std::shared_ptr<wmj::ArmorDetectorDouble> detector    = std::make_shared<wmj::ArmorDetectorDouble>();
#if (AIMERTYPE == 0)
    std::shared_ptr<wmj::Aimer>               aimer       = std::make_shared<wmj::AngleAimer>();
#else
    std::shared_ptr<wmj::Aimer>               aimer       = std::make_shared<wmj::StaticAimer>();
#endif

    timeval tv;
    int conti_flag = 0;
    int detect_num = 0;
    int undetect_num = 0;
    int time_interval = 0;
    std::vector<wmj::MatWithTime> frames;
    wmj::Rate rate_detect(100);
    wmj::GimbalPose target_pose, cur_pose, gm_speed;
#ifdef SAVEPOINT
    std::ofstream ofs("./Record.txt");
#endif

    // 装甲识别+测距+自瞄
    cv::Mat leftMat, rightMat;
    wmj::Armor final_armor, left_armor, right_armor;
    bool found_armor = false;
    bool auto_attack = false;

    while (true)
    {
        double time_begin = wmj::now();
        if (*camera >> frames)
        {
            std::cout << "get images wrong!!!\n";
            continue;
        }

        for (int i = 0; i < camera->m_device_number; ++i)
        {
            if (frames[i].m_img.empty())
            {
                std::cout << "Not load empty !!!\n";
                conti_flag = 1;
                continue;
            }
        }
        if(conti_flag)
        {
            conti_flag = 0;
            continue;
        }
        double time_stop = wmj::now();
        std::cout << "TopTest" << "getImage time cost is " << 1000.f * (time_stop - time_begin) << std::endl;


        // 识别装甲板
        time_begin = wmj::now();
        std::vector<wmj::Armor> armors;
        final_armor = detector->DetectArmorDouble(frames, armors);
        found_armor = (armors[0].m_armor_type  != wmj::ARMOR_NONE) && (armors[1].m_armor_type != wmj::ARMOR_NONE);
        time_stop = wmj::now();
        std::cout << "TopTest" << "ArmorDetect time cost is " << 1000.f * (time_stop - time_begin) << std::endl;
        final_armor.m_position.x /= 1000;
        final_armor.m_position.y /= 1000;
        final_armor.m_position.z /= 1000;
        cv::Point3f point = angle->cam2abs(final_armor.m_position, control->GetGimbalAngle());
#ifdef SAVEPOINT
                ofs << point.x << " " << point.y << " ";
#endif


        if(found_armor)
        {
            control->SwitchBaseCoor(1); // 切换地面系
            if(++detect_num > 10)
                undetect_num = 0; // 识别到后清零未识别计数
            std::cout << _lightgreen("装甲坐标 : ") << final_armor.m_position  << endl;

            // 获取当前位姿
            cur_pose = control->GetGimbalAngle();
            std::cout << _lightblue("当前位姿 : ") << cur_pose << "\n";

            // 计算目标位姿
            // wmj::GimbalPose target_pose_new;
            if(use_aimer)
            {
                double time_start = wmj::now();
                aimer->setTrackMode(wmj::Aimer::FOLLOW);
                target_pose = aimer->getFinalPose(cur_pose, final_armor, control->getShootSpeedValue());
                // std::cout << "shoot flag " << aimer->m_shoot_flag << std::endl;
                std::cout<<"time cost: "<<(wmj::now() - time_start) * 1000 << std::endl;
            }
            else
            {
                target_pose = angle->getAngle(final_armor.m_position, cur_pose, control->getShootSpeedValue());
            }
            std::cout << _lightcyan("解算位姿 : ") << target_pose << "\n";

            if(use_control)
            { 
                if(use_speed)
                {
                    wmj::GimbalPose gm_speed = speed->resolve(target_pose, cur_pose, wmj::now());
                    std::cout << _yellow("解算速度 : ") << gm_speed << "\n";

                    // control->SetGimbalSpeed(gm_speed.pitch, gm_speed.yaw);
                    control->SetGimbal_YawSpeed_PitchAngle(target_pose.pitch, gm_speed.yaw);
                }
                else
                {
                    control->SetGimbalAngle(target_pose.pitch, target_pose.yaw);
                }

                if(c == 'a')
                {
                    auto_attack = !auto_attack;
                    control->ShootNone();
                }

                if(auto_attack)
                {
                    // 自动发射
                    std::cout << _lightred("<<<<<< Auto Mode >>>>>>") << std::endl;
                    double yaw_diff = std::abs(cur_pose.yaw - target_pose.yaw);
                    std::cout << "yaw_diff: " << yaw_diff << std::endl;
                    if(aimer->shootable() && yaw_diff < 0.1)
                    {
                        std::cout << _lightpurple("<<< On Attack!!! >>>") << std::endl;
                        // control->ShootOnce();
                        time_interval--;
                        if (time_interval < 0)
                        {
                            std::cout << _purple("<<< Open fire!!! >>>") << std::endl;
                            control->ShootOnce();
                            time_interval = 5;
                        }
                    }
                    else
                    {
                        control->ShootNone();
                        time_interval = 0;
                    }
                }
                else
                {
                    // 手动发射
                    std::cout << _lightblue("<<<<<< Manual Mode >>>>>>") << std::endl;
                    switch(c)
                    {
                    case '1':
                        control->ShootOnce();
                        break;
                    case '2':
                        control->ShootSome(4);
                        break;
                    case '3':
                        control->KeepShoot();
                        break;
                    case '4':
                        control->ShootNone();
                        break;
                    case '5':
                        control->openBox();
                        break;
                    default:
                        break;
                    }
                }
                c = 'f';
                usleep(1000);
            }
            std::cout << _warning("************************************") + "\n";
        }
        else
        {
            undetect_num++;
            {
                std::cout << "#########################################" << std::endl;
                std::cout << _red("未识别到装甲板!!!") << std::endl;
                std::cout << "#########################################" << std::endl;
            }
            switch(c)
            {
            case '4':
                control->StopShoot();
                break;
            case '5':
                control->openBox();
                break;
            default:
                break;
            }

            c = 'f';
            time_interval = 0;
            usleep(1000);
            if(undetect_num > 3)
                detect_num = 0;
            if(undetect_num > 5)
                control->SetGimbalSpeed(0, 0);
        }
        rate_detect.sleep();
    }
#ifdef SAVEPOINT
    ofs.close();
#endif
    return 0;
}
