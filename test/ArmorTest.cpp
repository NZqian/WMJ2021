/**
 * 自瞄测试文件
 */
#include "../libFSM/include/StateMachine.h"
#include <fstream>
#define SAVEPOINT
#define ENABLEMOVE
#define ENABLEPREDICT
#define USEMONO

int main(int argc, char **argv)
{
    bool use_speed  = false;
    bool auto_shoot = false;

#ifdef SAVEPOINT
    bool save_point = true;
#endif
#ifndef SAVEPOINT
    bool save_point = false;
#endif
    switch (argc)
    {
        case 2:
            use_speed = true;
            break;
        default:
            break;
    }
    bool use_pose   = false; // 预测模式

    char c = 'f';
    std::thread(monitorKeyboard, &c).detach();
    std::shared_ptr<wmj::UsbCapture> camera = std::make_shared<wmj::UsbCapture>();
    camera->cameraMode("armor");

    std::shared_ptr<wmj::WMJRobotControl>     control        = std::make_shared<wmj::WMJRobotControl>();
    std::shared_ptr<wmj::AngleSolver>         angle          = std::make_shared<wmj::AngleSolver>();
    std::shared_ptr<wmj::SpeedResolver>       speed          = std::make_shared<wmj::SpeedResolver>();
    std::shared_ptr<wmj::MotionPredict>       predictor      = std::make_shared<wmj::MotionPredict>();
    std::shared_ptr<wmj::PosePredict>         pose_predictor = std::make_shared<wmj::PosePredict>();
    std::shared_ptr<wmj::ArmorDetectorDouble> detector       = std::make_shared<wmj::ArmorDetectorDouble>();
#ifdef USEMONO
        detector->i_state = 1;
#endif

    timeval tv;
    int conti_flag = 0;
    int detect_num = 0;
    int undetect_num = 0;
    std::vector<wmj::MatWithTime> frames;
    wmj::Rate rate_detect(100);
    wmj::GimbalPose target_pose, cur_pose, gm_speed;
    std::ofstream ofs("./Record.txt");

    bool set_time = false;
    int FPS = 0;
    double tb, te;
    // 装甲识别+测距+自瞄
    wmj::Armor final_armor;
    std::vector<wmj::Armor> armors;
    bool found_armor = false;
    detector -> init();
#ifdef SAVEPOINT
    int loopnum = 1000;
    while (loopnum-- >= 0)
#endif

#ifndef SAVEPOINT

        //detector->i_state = wmj::State::SentryAttack;
        while(1)
#endif
        {
            if(!set_time)
            {
                set_time = true;
                tb = wmj::now();
            }
            te = wmj::now();
            FPS++;
            if(1000.f * (te - tb) > 1000)
            {
                set_time = false;
                std::cout << "FPS : " << FPS << "\n";
                FPS = 0;
            }

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
            std::cout << "getImage time cost is " << 1000.f * (time_stop - time_begin) << std::endl;
            // 识别装甲板
            gettimeofday(&tv, NULL);
            long time_start = tv.tv_sec * 1000000 + tv.tv_usec;
            final_armor = detector->DetectArmorDouble(frames, armors);
            found_armor = armors[0].m_armor_type != wmj::ARMOR_NONE;
            gettimeofday(&tv, NULL);
            long time_end = tv.tv_sec * 1000000 + tv.tv_usec;
            final_armor.m_position.x /= 1000;
            final_armor.m_position.y /= 1000;
            final_armor.m_position.z /= 1000;
            // if(m_detectID == -1 && found_armor)
            // {
            //     m_detectID = final_armor.id;
            // }
            // else if(found_armor && m_detectID != final_armor.id)
            // {
            //     found_armor = !found_armor;
            // }
            if(found_armor)
            {
                control->SwitchBaseCoor(1); // 切换地面系
                if(++detect_num > 10) undetect_num = 0; // 识别到后清零未识别计数
                std::cout << _yellow("识别耗时 : ")     << time_end - time_start << _yellow("识别次数 ") << detect_num << "\n";
                std::cout << _lightgreen("装甲坐标 : ") << final_armor.m_position  << endl;
                // 获取当前位姿
                cur_pose = control->GetGimbalAngle();
                std::cout << _lightblue("当前位姿 : ") << cur_pose << "\n";
                cv::Point3f point = angle->cam2abs(final_armor.m_position, control->GetGimbalAngle());
                std::cout << _yellow("绝对坐标 ") << point << std::endl;
                if(use_speed)
                {
#ifdef ENABLEPREDICT
                    if(!use_pose) //预测绝对坐标
                    {
                        if(save_point)
                            ofs << point.x << " " << point.y << " ";
                        final_armor.m_position = predictor->predict(point);
                        std::cout << _lightgreen("滤波绝对坐标 : ") << final_armor.m_position << "\n";
                        if(save_point)
                            ofs << final_armor.m_position.x << " " << final_armor.m_position.y << std::endl;

                        final_armor.m_position = angle->abs2cam(final_armor.m_position, control->GetGimbalAngle());
                        target_pose = angle->getAngle(final_armor.m_position, cur_pose, control->getShootSpeedValue());
                        std::cout << _lightcyan("解算位姿 : ") << target_pose << "\n";

                        gm_speed = speed->resolve(target_pose, cur_pose, wmj::now());
                        std::cout << _yellow("解算速度 : ") << gm_speed << "\n";
                    }
                    else // 预测云台位姿
                    {
                        if(save_point)
                            ofs << control->GetGimbalAngle().pitch << " " << control->GetGimbalAngle().yaw << std::endl;
                        //final_armor.position = predictor->predict(point);
                        //std::cout << _lightgreen("滤波绝对坐标 : ") << final_armor.position << "\n";
                        //final_armor.position = angle->abs2gun(final_armor.position, control->GetGimbalAngle());

                        target_pose = angle->getAngle(final_armor.m_position, cur_pose, control->getShootSpeedValue());
                        std::cout << _lightcyan("解算位姿 : ") << target_pose << "\n";
                        target_pose = pose_predictor->predict(target_pose);
                        std::cout << _lightgreen("滤波云台位姿 : ") << target_pose << std::endl;
                        if(save_point)
                            ofs << control->GetGimbalAngle().pitch << " " << control->GetGimbalAngle().yaw << std::endl;

                        gm_speed = speed->resolve(target_pose, cur_pose, wmj::now());
                        std::cout << _yellow("解算速度 : ") << gm_speed << "\n";
                    }
#endif

#ifndef ENABLEPREDICT
                        target_pose = angle->getAngle(final_armor.position, cur_pose, control->getShootSpeedValue());
                        std::cout << _lightcyan("解算位姿 : ") << target_pose << "\n";

                        gm_speed = speed->resolve(target_pose, cur_pose, wmj::now());
                        std::cout << _yellow("解算速度 : ") << gm_speed << "\n";
#endif

#ifdef ENABLEMOVE
                    //control->SetGimbalSpeed(gm_speed.pitch, gm_speed.yaw);
                    control->SetGimbal_YawSpeed_PitchAngle(target_pose.pitch, gm_speed.yaw);
                    //control->SetGimbalAngle(target_pose.pitch, target_pose.yaw);
#endif
                    // 发射子弹
                    double pitch_diff = abs(target_pose.pitch - cur_pose.pitch);
                    double yaw_diff = abs(target_pose.yaw - cur_pose.yaw);
                    if(yaw_diff < 0.04 && auto_shoot)
                    {
                        std::cout << _red("<<<<<<<<<<<<auto shoot state<<<<<<<<<<<<<\n");
                        control->ShootSome(1);
                    }
                    else if(yaw_diff >= 0.04 || !auto_shoot)
                    {
                        control->ShootNone();
                    }

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
                            control->StopShoot();
                            break;
                        case '5':
                            control->openBox();
                            break;
                        case 'a':
                            auto_shoot = !auto_shoot;
                            break;
                        default:
                            break;
                    }
                    c = 'f';
                    usleep(1000);
                }
                else
                {
#ifdef ENABLEPREDICT
                    if(!use_pose) //预测绝对坐标
                    {
                        if(save_point)
                            ofs << point.x << " " << point.y << " ";
                        final_armor.m_position = predictor->predict(point);
                        std::cout << _lightgreen("滤波绝对坐标 : ") << final_armor.m_position << "\n";
                        if(save_point)
                            ofs << final_armor.m_position.x << " " << final_armor.m_position.y << std::endl;

                        final_armor.m_position = angle->abs2cam(final_armor.m_position, control->GetGimbalAngle());
                        target_pose = angle->getAngle(final_armor.m_position, cur_pose, control->getShootSpeedValue());
                        std::cout << _lightcyan("解算位姿 : ") << target_pose << "\n";
                        //target_pose = pose_predictor->predict(target_pose);
                        //std::cout << _lightcyan("滤波位姿 : ") << target_pose << "\n";
                    }
                    else // 预测云台位姿
                    {
                        if(save_point)
                            ofs << control->GetGimbalAngle().pitch << " " << control->GetGimbalAngle().yaw << std::endl;
                        target_pose = angle->getAngle(final_armor.m_position, cur_pose, control->getShootSpeedValue());
                        std::cout << _lightcyan("解算位姿 : ") << target_pose << "\n";
                        target_pose = pose_predictor->predict(target_pose);
                        std::cout << _lightgreen("滤波云台位姿 : ") << target_pose << std::endl;
                        if(save_point)
                            ofs << control->GetGimbalAngle().pitch << " " << control->GetGimbalAngle().yaw << std::endl;
                    }
#endif

#ifndef ENABLEPREDICT
                        target_pose = angle->getAngle(final_armor.position, cur_pose, control->getShootSpeedValue());
                        std::cout << _lightcyan("解算位姿 : ") << target_pose << "\n";
#endif

                    control->SetGimbalAngle(target_pose.pitch, target_pose.yaw);
                    switch (c)
                    {
                        case '1':
                            control->ShootOnce();
                            break;
                        case '2':
                            control->ShootSome(7);
                            break;
                        case '3':
                            control->KeepShoot();
                            break;
                        case '4':
                            control->StopShoot();
                        case 'a':
                            auto_shoot = !auto_shoot;
                            break;
                        case '5':
                            control->openBox();
                            break;
                        default:
                            control->ShootNone();
                            break;
                    }
                    c = 'f' ;
                    usleep(1000);
                }

                std::cout << _warning("************************************") + "\n";
            }
            else
            {
                control->ShootNone();
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
                usleep(1000);
                if(undetect_num > 3) detect_num = 0;
#ifndef SAVEPOINT
                if(undetect_num > 8) 
                {
                    control->SetGimbalSpeed(0, 0);
                }
#endif
            }
            rate_detect.sleep();
        }
    ofs.close();
    return 0;
}
