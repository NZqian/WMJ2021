/**
 * 相机测试文件
 * 运行时随便加个参数查看对应相机序列号，例如 ./CameraTest param
 * 不加参数则显示拍摄到的图像，如果跑不起来，先运行./CameraTest param查看相机序列号
 * 然后在编译目录下的UsbCapture.yaml里修改对应的序列号指定左右相机
 */
#include "../libHardware/UsbCapture/include/UsbCapture.h"

int main(int argc, char **argv)
{

    std::shared_ptr<wmj::UsbCapture> camera = std::make_shared<wmj::UsbCapture>();
    camera->cameraMode("armor");

    if (argc > 1)
    {
        camera->getCameraInfo();
        std::cout << "想看视频就憋加参数了，不然指腚妹你好果汁吃！\n";
        exit(0);
    }

    std::vector<wmj::MatWithTime> frames;

    for(int i = 0; i < camera->m_device_number; i++)
        std::cout << "串口号 : " << camera->m_serial_number[i] << "\n";

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
        {
            if (frames[i].m_img.empty())
            {
                std::cout << "空图像 !!!\n";
                conti_flag = 1;
                break;
            }
        }
        if(conti_flag) continue;

        for (int i = 0; i < camera->m_device_number; ++i)
        {
            cv::cvtColor(frames[i].m_img, frames[i].m_img, cv::COLOR_RGB2BGR);
            cv::putText(frames[i].m_img, frames[i].m_orientation, cv::Point(800, 50), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar::all(255), 1);
        }

        if(camera->m_device_number == 1)
            for (int i = 0; i < camera->m_device_number; ++i)
                cv::imshow(camera->m_serial_number[i], frames[i].m_img);
        else if(camera->m_device_number == 2)
        {
            cv::imshow("left", frames[0].m_img);
            cv::imshow("right", frames[1].m_img);
        }

        char c = cv::waitKey(1);
        if (c == 27 || c == 'q') exit(0);
    }
    return 0;
}
