#include "../libHardware/UsbCapture/include/UsbCapture.h"
#include "../libVision/Armor/include/ArmorSingle.hpp"
#include <fcntl.h>
#include <opencv2/highgui.hpp>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <typeinfo>
#include <iostream>
std::shared_ptr<wmj::UsbCapture> camera = std::make_shared<wmj::UsbCapture>();
std::shared_ptr<wmj::ArmorSingle> detectorLeft = std::make_shared<wmj::ArmorSingle>();
std::shared_ptr<wmj::ArmorSingle> detectorRight = std::make_shared<wmj::ArmorSingle>();
std::vector<wmj::MatWithTime> frames;
using namespace std;
using namespace wmj;
using namespace cv;
int main()
{
    camera->setExposureTime(100,wmj::MANUAL);

    int targetExposure = 100;
    bool flag =  false;
    bool conti_flag = false;
    Armors leftArmors;
    Armors rightArmors;
    Armor left_lastArmor;
    Armor right_lastArmor;
    int add_exposure = 100;
    Rect left_rect = Rect(Point2f(0,0),Point2f(1280,1024));
    Rect right_rect = Rect(Point2f(100,100),Point2f(1280,1024));
    left_lastArmor.m_rect = left_rect;
    right_lastArmor.m_rect = right_rect;
    detectorLeft->m_state = wmj::TRACKING;
    detectorRight->m_state = wmj::TRACKING;
    detectorLeft->m_last_armor = left_lastArmor;
    detectorRight->m_last_armor = right_lastArmor;
    float exposure_coe = 1.5;
    while(1)
    {
        if(*camera >> frames)
        {
            cout<<"获取图像失败"<<endl;
        }
        for (int i = 0; i < camera->m_device_number; ++i)
        {
            if (frames[i].m_img.empty())
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
        detectorLeft->m_src = frames[0].m_img;
        detectorRight->m_src = frames[1].m_img;
        //std::thread threadRight(detectorRight->ArmorSingleDetector());
        detectorLeft->ArmorSingleDetector();
        detectorRight->ArmorSingleDetector();
        // if(detectorLeft->complete == true)
        // {
        //     threadRight.join();
        // }
        if(detectorRight->m_armors.size()>0&&detectorLeft->m_armors.size()>0)
        {
            flag = true;
	        cv::cvtColor(frames[0].m_img, frames[0].m_img, cv::COLOR_RGB2BGR);
	        cv::putText(frames[0].m_img, "successfully found!!!", cv::Point(100, 10), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 0), 2);
	        cv::putText(frames[0].m_img, "current exposure : " + std::to_string(targetExposure), cv::Point(100, 80), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 255), 2);
	        cv::putText(frames[0].m_img, "coe : " + std::to_string(exposure_coe), cv::Point(100, 160), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 255), 2);
	        cv::putText(frames[0].m_img, "final exposure : " + std::to_string((int)targetExposure * exposure_coe), cv::Point(100, 240), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 255), 2);

	        cv::cvtColor(frames[1].m_img, frames[1].m_img, cv::COLOR_RGB2BGR);
	        cv::putText(frames[1].m_img, "successfully found!!!", cv::Point(100, 10), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 0), 2);
	        cv::putText(frames[1].m_img, "current exposure : " + std::to_string(targetExposure), cv::Point(100, 80), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 255), 2);
	        cv::putText(frames[1].m_img, "coe : " + std::to_string(exposure_coe), cv::Point(100, 160), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 255), 2);
	        cv::putText(frames[1].m_img, "final exposure : " + std::to_string((int)targetExposure * exposure_coe), cv::Point(100, 240), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 255), 2);

	        cv::imshow("left_img",  frames[0].m_img);
	        cv::imshow("right_img", frames[1].m_img);
	        if(cv::waitKey(0) == 'q')
                    break;
        }
        if(targetExposure > 12000)
        {
	        cv::cvtColor(frames[0].m_img, frames[0].m_img, cv::COLOR_RGB2BGR);
	        cv::putText(frames[0].m_img, "not found!!!", cv::Point(100, 10), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 0, 0), 2);
	        cv::putText(frames[0].m_img, "current exposure : " + std::to_string(targetExposure), cv::Point(100, 80), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 255), 2);
	        cv::cvtColor(frames[1].m_img, frames[1].m_img, cv::COLOR_RGB2BGR);
	        cv::putText(frames[1].m_img, "not found!!!", cv::Point(100, 10), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 0, 0), 2);
	        cv::putText(frames[1].m_img, "current exposure : " + std::to_string(targetExposure), cv::Point(100, 80), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 255), 2);
	        cv::imshow("left_img",  frames[0].m_img);
	        cv::imshow("right_img", frames[1].m_img);
	        if(cv::waitKey(0) == 'q')
                    break;
        }
	    cv::cvtColor(frames[0].m_img, frames[0].m_img, cv::COLOR_RGB2BGR);
	    cv::putText(frames[0].m_img, "current exposure : " + std::to_string(targetExposure), cv::Point(100, 80), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 255), 2);
	    cv::cvtColor(frames[1].m_img, frames[1].m_img, cv::COLOR_RGB2BGR);
	    cv::putText(frames[1].m_img, "current exposure : " + std::to_string(targetExposure), cv::Point(100, 80), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 255), 2);
	    cv::imshow("left_img",  frames[0].m_img);
	    cv::imshow("right_img", frames[1].m_img);
	    std::cout << _yellow(">>>>>>>>>>>>>>>current exposure : ") << targetExposure << std::endl;
        if(cv::waitKey(100) == 'q')
            break;
        targetExposure += add_exposure;
        camera->setExposureTime(targetExposure,wmj::MANUAL);
    }
    if(flag)
    {
        targetExposure *= exposure_coe;
        cout << "找到目标曝光值为：" << targetExposure << endl;
    }
    else
    {
        cout << "未找到目标曝光值" << endl;
    }

    return 0;
    
}
