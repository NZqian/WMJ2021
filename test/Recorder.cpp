/**
 * 视频录制程序
 */
#include "../libHardware/UsbCapture/include/UsbCapture.h"

int main(int argc, char **argv)
{
    std::string count;
    if(argc >1)
    {
        count = argv[1];
    }
    
    int fps = 60;
    std::shared_ptr<wmj::UsbCapture> camera = std::make_shared<wmj::UsbCapture>();
    camera->cameraMode("armor");

    std::vector<wmj::MatWithTime> frames;
    cv::Mat leftMat, rightMat;
    
    int hour;
    // recorder
    std::vector<cv::VideoWriter> recorders(camera->m_device_number);
    if(camera->m_device_number == 2)
    {
        if(camera->m_orientation[0] == "left")
        {
            recorders[0] = cv::VideoWriter("../video/left"+count+".avi", CV_FOURCC('M', 'J', 'P', 'G'), fps, cv::Size(1280, 1024), true);
            recorders[1] = cv::VideoWriter("../video/right"+count+".avi", CV_FOURCC('M', 'J', 'P', 'G'), fps, cv::Size(1280, 1024), true);
           
        }
        else
        {
            recorders[0] = cv::VideoWriter("../video/right"+count+".avi", CV_FOURCC('M', 'J', 'P', 'G'), fps, cv::Size(1280, 1024), true);
            recorders[1] = cv::VideoWriter("../video/left"+count+".avi", CV_FOURCC('M', 'J', 'P', 'G'), fps, cv::Size(1280, 1024), true);
        }
    }
    else
        recorders[0] = cv::VideoWriter("../video/single"+count+".avi", CV_FOURCC('M', 'J', 'P', 'G'), fps, cv::Size(1280, 1024), true);

    int conti_flag = 0;
    while (true)
    {
        conti_flag = 0;
        if (*camera >> frames)
        {
            std::cout << "get images wrong!!!\n";
            continue;
        }

        for (int i = 0; i < camera->m_device_number; ++i)
            if (frames[i].m_img.empty())
            {
                std::cout << "Not load empty !!!\n";
                conti_flag = 1;
                break;
            }
        if(conti_flag) continue;

        for (int i = 0; i < camera->m_device_number; ++i)
        {
            cv::cvtColor(frames[i].m_img, frames[i].m_img, cv::COLOR_RGB2BGR);
            if(frames[i].m_orientation == "left")
                leftMat = frames[i].m_img;
            else if(frames[i].m_orientation == "right")
                rightMat = frames[i].m_img;
        }

        for(int i = 0; i < camera->m_device_number; i++)
        {
            cv::namedWindow(camera->m_orientation[i], cv::WINDOW_NORMAL);
            cv::imshow(camera->m_orientation[i], frames[i].m_img);
            recorders[i].write(frames[i].m_img);
        }

        char c = cv::waitKey(1);
        if (c == 'q' || c == 27)
            break;
    }

    for(auto i : recorders)
        i.release();
    return 0;
}
