#include "../libFSM/include/StateMachine.h"

int main(int argc, char **argv)
{
    char c = 'f';
    std::thread keyThread(monitorKeyboard, &c);
    keyThread.detach();

    std::shared_ptr<wmj::UsbCapture> camera = std::make_shared<wmj::UsbCapture>();
    std::shared_ptr<wmj::WMJRobotControl> control = std::make_shared<wmj::WMJRobotControl>();
    std::shared_ptr<wmj::AngleSolver> angle = std::make_shared<wmj::AngleSolver>();
    std::shared_ptr<wmj::SpeedResolver> speed = std::make_shared<wmj::SpeedResolver>();
    std::shared_ptr<wmj::MotionPredict> predictor = std::make_shared<wmj::MotionPredict>();
    std::shared_ptr<wmj::ArmorTrigger> rune = std::make_shared<wmj::ArmorTrigger>();

    camera->cameraMode("rune");

    timeval tv;
    int conti_flag = 0;
    int detectNum = 0;
    int undetectNum = 0;
    int auto_flag = 1;
    long shoot_time = 0;
    std::vector<wmj::MatWithTime> frames;
    wmj::Rate rateRune(100);
    wmj::GimbalPose targetPose, curPose, gmSpeed, referPose;
    wmj::Armor finalArmor;
    bool foundRune = false;
    bool use_speed = true;

    while (c != 27)
    {
        // gettimeofday(&tv, NULL);
        // long time_start1 = tv.tv_sec * 1000000 + tv.tv_usec;
        // std::cout << _yellow("angle start:") << time_start1 << std::endl;
        curPose = control->GetGimbalAngle();
        // gettimeofday(&tv, NULL);
        // long time_end1 = tv.tv_sec * 1000000 + tv.tv_usec;
        // std::cout << _yellow("angle end:") << time_end1 << std::endl;
        // std::cout << _yellow("angle duration:") << time_end1 - time_start1 << std::endl;

        // gettimeofday(&tv, NULL);
        // long time_start0 = tv.tv_sec * 1000000 + tv.tv_usec;
        // std::cout << _yellow("frame start:") << time_start0 << std::endl;
        if (*camera >> frames)
        {
            std::cout << "get images wrong!!!\n";
            continue;
        }
        // gettimeofday(&tv, NULL);
        // long time_end0 = tv.tv_sec * 1000000 + tv.tv_usec;
        // std::cout << _yellow("frame end:") << time_end0 << std::endl;
        // std::cout << _yellow("frame duration left:")<<time_start1 << (frames[0].m_time_stamp - time_start1)/1000 << std::endl;
        // std::cout << _yellow("frame duration right:") << (frames[1].m_time_stamp - time_start1)/1000 << std::endl;

        // curPose = control->GetGimbalAngle();

        for (int i = 0; i < camera->m_device_number; ++i)
        {
            if (frames[i].m_img.empty())
            {
                std::cout << "Not load empty !!!\n";
                conti_flag = 1;
                continue;
            }
        }
        if (conti_flag)
        {
            conti_flag = 0;
            continue;
        }

        double shoot_speed = control->getShootSpeedValue();
        double predict_time = 7.5 / std::cos(0.21) / shoot_speed + 0.13;
        rune->setPredictTime(predict_time);

        wmj::ROBO_ENERGY energy_status = control->getEnergyStatus();
        std::cout << _blue("energy status:") << energy_status << std::endl;

        gettimeofday(&tv, NULL);
        long time_start = tv.tv_sec * 1000000 + tv.tv_usec;
        finalArmor.m_position = rune->findTarget(frames, 1);
        cv::Point3f position = rune->m_armor_position;
        foundRune = !(finalArmor.m_position.x == -1 && finalArmor.m_position.y == -1 && finalArmor.m_position.z == -1);
        gettimeofday(&tv, NULL);
        long time_end = tv.tv_sec * 1000000 + tv.tv_usec;

        if (foundRune)
        {
            if (++detectNum > 10)
                undetectNum = 0;
            std::cout << _yellow("[rune] 识别到了!!!!!识别到装甲板耗时 : ") << (time_end - time_start) / 1000 << "ms\n";
            std::cout << _blue("[rune] 识别到坐标 :") << finalArmor.m_position << endl;
            control->SwitchBaseCoor(1); //ture 地面系

            std::cout << "[rune] 当前 pitch : " << curPose.pitch << "\t当前 yaw : " << curPose.yaw << "\n";

            referPose = angle->getAngle(rune->m_center_position, curPose, control->getShootSpeedValue());
            referPose.yaw = curPose.yaw - referPose.yaw;
            referPose.pitch = curPose.pitch;
            cv::Point3f center_abs = angle->cam2abs(rune->m_center_position, referPose);
            cv::Point3f actual_abs = angle->cam2abs(rune->m_armor_position, referPose);

            std::cout << _purple("abs center :") << center_abs << std::endl;
            std::cout << _cyan("abs actual :") << actual_abs << std::endl;
            float predict_angle = (rune->m_left_predict_angle + rune->m_right_predict_angle) / 2;
            std::cout << _blue("predict angle: ") << rune->m_left_predict_angle << " " << rune->m_right_predict_angle << '\n'
                      << predict_angle << std::endl;
            cv::Point3f predict_abs;
            predict_abs.x = actual_abs.x;
            predict_abs.y = -(actual_abs.y - center_abs.y) * cos(-predict_angle * CV_PI / 180) + (actual_abs.z - center_abs.z) * sin(-predict_angle * CV_PI / 180) + center_abs.y;
            predict_abs.z = -(actual_abs.y - center_abs.y) * sin(-predict_angle * CV_PI / 180) - (actual_abs.z - center_abs.z) * cos(-predict_angle * CV_PI / 180) + center_abs.z;
            std::cout << _green("abs predict :") << predict_abs << std::endl;

            finalArmor.m_position = angle->abs2cam(predict_abs, referPose);

            std::cout << _lightblue("final position :") << finalArmor.m_position << std::endl;
            targetPose = angle->getAngle(finalArmor.m_position, curPose, shoot_speed);

            targetPose.pitch += rune->m_pitch_off;

            std::cout << "[rune] 解算 pitch : " << targetPose.pitch << "\t解算 yaw : " << targetPose.yaw << "\n";
            // control->SetGimbalAngle(targetPose.pitch, targetPose.yaw);
            if (!rune->m_auto_shoot)
            {
                switch (c)
                {
                case 'j':
                    control->ShootOnce();
                    break;
                case 'k':
                    control->StopShoot();
                    break;
                case 'l':
                    control->openBox();
                    break;
                default:
                    break;
                }
                c = 'f';
                usleep(1000);
            }
            else
            {
                curPose = control->GetGimbalAngle();
                double pose_diff = abs(targetPose.pitch - curPose.pitch) + abs(targetPose.yaw - curPose.yaw);
                gettimeofday(&tv, NULL);
                long now_time = tv.tv_sec * 1000000 + tv.tv_usec;
                if ((now_time - shoot_time) > 1e6 && pose_diff < 0.1)
                {
                    control->ShootOnce();
                    shoot_time = now_time;
                }
            }
            std::cout << "[rune] ************************************\n";
        }
        else
        {
            std::cout << _warning("未识别到装甲板或双目不匹配") << std::endl;
            undetectNum++;
            if (undetectNum > 15)
                detectNum = 0;
            if (undetectNum > 80)
            {
                control->SetGimbalSpeed(0, 0);
                control->StopShoot();
            }
            switch (c)
            {
            case 'l':
                control->openBox();
                break;
            default:
                break;
            }
            c = 'f';
        }
        rateRune.sleep();
    }
    control->SetGimbalSpeed(0, 0);
    control->StopShoot();
    return 0;
}
