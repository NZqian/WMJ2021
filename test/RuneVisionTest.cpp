#include "../libVision/Rune/include/ArmorTrigger.hpp"
#include "../libHardware/UsbCapture/include/UsbCapture.h"

int main(int argc, char **argv)
{
    std::shared_ptr<::wmj::ArmorTrigger> m_RuneDetector;
    std::shared_ptr<::wmj::UsbCapture> m_VideoCapture;
    bool conti_flag = 0;

    m_VideoCapture = std::make_shared<wmj::UsbCapture>(); // create camera
    m_VideoCapture->cameraMode("rune");
    m_RuneDetector = std::make_shared<::wmj::ArmorTrigger>(); // create ArmorTrigger

    while (true)
    {
        std::vector<wmj::MatWithTime> frame;
        if (*m_VideoCapture >> frame)
        {
            std::cout << "Wrong Get Image!!!" << std::endl;
            continue;
        }
        for (int i = 0; i < m_VideoCapture->m_device_number; ++i)
        {
            if (frame[i].m_img.empty())
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

        cout << "图片数" << frame.size() << endl;

        double time_begin = wmj::now();
        cv::Point3f position = m_RuneDetector->findTarget(frame, 1);
        double time_end = wmj::now();
        cout << _blue("final position :") << position << endl;
        std::cout << "Time cost is : " << 1000 * (time_end - time_begin) << "ms" << std::endl;

        cv::imshow("left", frame[0].m_img);
        
        char c = cv::waitKey(1);
        if (c == 'q' || c == 'Q' || c == 27)
            break;
    }
    return 0;
}
