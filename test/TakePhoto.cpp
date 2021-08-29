/**
 * 拍照程序
 */
#include "../libHardware/UsbCapture/include/UsbCapture.h"

int main(int argc, char **argv)
{
    std::vector<wmj::MatWithTime> frames;
    std::shared_ptr<wmj::UsbCapture> camera = std::make_shared<wmj::UsbCapture>();

    camera->cameraMode("calib");

    int count = 0;
    int limit = 20;
    int conti_flag = 0;
    std::cout << "输入要拍的照片张数: ";
    std::cin >> limit;
    std::cout << "请按s键拍照 " << limit << " 次\n";
    while (true)
    {
        if (camera->getImg(frames) != 0)
        {
            std::cout << "未获得图像!!!" << std::endl;
            continue;
        }

        for(int i = 0; i < camera->m_device_number; i++)
        {
            if (frames[i].m_img.empty())
            {
                std::cout << "空图像" << "\n";
                conti_flag = 1;
                break;
            }
            conti_flag = 0;
        }

        if(conti_flag)
            continue;

        for(int i = 0; i < camera->m_device_number; i++)
        {
            cv::cvtColor(frames[i].m_img, frames[i].m_img, cv::COLOR_RGB2BGR);
            cv::namedWindow(camera->m_orientation[i], cv::WINDOW_NORMAL);
            cv::imshow(camera->m_orientation[i], frames[i].m_img);
        }

        char c = cv::waitKey(1);
        if (c == 'q' || c == 27)
            break;
        else if(c == 's')
        {
            for (int i = 0; i < camera->m_device_number; i++)
                cv::imwrite(frames[i].m_orientation + "_" + std::to_string(count) + ".jpg", frames[i].m_img);
            count++;
            std::cout << "拍了 " << count << " 次\n";
        }
        if(count == limit)
            return 0;
    }
    return 0;
}
