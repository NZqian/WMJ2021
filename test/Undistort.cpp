/**
 * 图像矫正测试
 * 用于双目标定后检查标定参数的可靠度
 */
#include "../libHardware/UsbCapture/include/UsbCapture.h"
#include <fcntl.h>

int main(int argc, char **argv)
{
    std::shared_ptr<wmj::UsbCapture> camera = std::make_shared<wmj::UsbCapture>();
    camera->cameraMode("calib");

    int fd = open("./Binocular.yaml", O_EXCL, 0666);
    cv::FileStorage fs;
    if(fd < 0) // 读取Binocular里的配置文件
        fs = cv::FileStorage("../libVision/Bino/Binocular.yaml", cv::FileStorage::READ);
    else // 读取新标定出的配置文件
        fs = cv::FileStorage("Binocular.yaml", cv::FileStorage::READ);
    close(fd);

    cv::Mat camMatLeft, camMatRight;
    cv::Mat distCoeLeft, distCoeRight;
    fs["M1"]  >> camMatLeft;
    fs["M2"] >> camMatRight;
    fs["D1"] >> distCoeLeft;
    fs["D2"] >> distCoeRight;

    std::vector<wmj::MatWithTime> frames;
    cv::Mat leftMat, rightMat, mapLeft1, mapLeft2, mapRight1, mapRight2;
    cv::Mat leftCalib, rightCalib;
    cv::Size imageSize = cv::Size(1280, 1024);

    while (true)
    {
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
                continue;
            }
        }

        for (int i = 0; i < camera->m_device_number; ++i)
        {
            cv::cvtColor(frames[i].m_img, frames[i].m_img, cv::COLOR_RGB2BGR);
            cv::putText(frames[i].m_img, camera->m_serial_number[i], cv::Point(400, 50), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar::all(255), 1);
            cv::putText(frames[i].m_img, frames[i].m_orientation, cv::Point(800, 50), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar::all(255), 1);
            if(frames[i].m_orientation == "left")
                leftMat = frames[i].m_img;
            else if(frames[i].m_orientation == "right")
                rightMat = frames[i].m_img;
        }

        cv::initUndistortRectifyMap(camMatLeft,  distCoeLeft, cv::Mat(), cv::getOptimalNewCameraMatrix(camMatLeft, distCoeLeft, imageSize, 1, imageSize, 0), imageSize, CV_16SC2, mapLeft1, mapLeft2);
        cv::initUndistortRectifyMap(camMatRight,  distCoeRight, cv::Mat(), cv::getOptimalNewCameraMatrix(camMatRight, distCoeRight, imageSize, 1, imageSize, 0), imageSize, CV_16SC2, mapRight1, mapRight2);

        cv::remap(leftMat, leftCalib, mapLeft1, mapLeft2, cv::INTER_LINEAR);
        cv::remap(rightMat, rightCalib, mapRight1, mapRight2, cv::INTER_LINEAR);

        cv::namedWindow("left", cv::WINDOW_NORMAL);
        cv::imshow("left", leftMat);

        cv::namedWindow("leftCalib", cv::WINDOW_NORMAL);
        cv::imshow("leftCalib", leftCalib);

        cv::namedWindow("right", cv::WINDOW_NORMAL);
        cv::imshow("right", rightMat);

        cv::namedWindow("rightCalib", cv::WINDOW_NORMAL);
        cv::imshow("rightCalib", rightCalib);

        char c = cv::waitKey(1);
        if (c == 27 || c == 'q')
            break;
    }
    return 0;
}
