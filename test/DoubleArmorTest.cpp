/**
 * 装甲板识别测试程序
 */
#include "../libHardware/UsbCapture/include/UsbCapture.h"
#include "../libVision/Armor/include/ArmorDetector.hpp"
#include <fcntl.h>
#include <opencv2/highgui.hpp>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <typeinfo>
#include <iostream>

void initialize();  //初始化文字函数
void checkCamera(); //检测相机文件
void checkVideo();

std::shared_ptr<wmj::UsbCapture> camera = std::make_shared<wmj::UsbCapture>();
std::shared_ptr<wmj::ArmorDetectorDouble> detector = std::make_shared<wmj::ArmorDetectorDouble>();
// cv::VideoCapture Video_left(0);
// cv::VideoCapture Video_right(1);
std::vector<cv::VideoCapture> video_test(2);
wmj::Rate rate_detect(200);
int video_number = 0;
int videoOpen[2];
enum State
{
    Camera = 0,
    Video = 1
}; //各种状态
State state;
class Parameter
{
public:
    cv::FileStorage fs_test_armor;
    std::string video_path_left;
    // std::string video_path_right;
    std::vector<std::string> video_path;
    cv::Point3f XYZ;
    int waitKey_time;

public:
    Parameter()
    {
        //在这里初始化定义似乎无效
    }
    void readParameter()
    {
        cv::FileStorage fs_test_armor(
            "../test/test_armor.yaml",
            cv::FileStorage::READ); //导入test_armor文件，在yaml文件中操作

        video_path.resize(2);
        std::cout << "成功读取配置文件\n";
        fs_test_armor["Test_Armor"]["video_path_left"] >> video_path[0];
        fs_test_armor["Test_Armor"]["video_path_right"] >> video_path[1];
        fs_test_armor["Test_Armor"]["waitKey_time"] >> waitKey_time;
    }
};


Parameter test_armor;
int main(int argc, char **argv)
{
    int detectstate;
    if(argc == 2)
    {
        detectstate = argv[1][0]-'0';
        detector->i_state = detectstate;
    }
    //std::cout<<detectstate<<std::endl;
    if(detector->i_state >= 4)
    {
        camera->cameraMode("dart");
    }
    else
    {
        camera->cameraMode("armor");
    }
    
    initialize();
    checkCamera();
    checkVideo();
    
    std::vector<wmj::MatWithTime> frames;
    frames.clear();
    timeval tv;
    //std::vector<bool> getflags;
    //wmj::Armors armors;
    wmj::Armor finalArmor;
    bool getImage;
    //detector->SetValue(2);
    // detector->m_state = wmj::State::Single; //单目的测试初始化，日后可以再配置文件里面设置
    while (true)
    {
        long time_start;
        long time_end;
        getImage=1;
        if(state==Camera) 
        {
            gettimeofday(&tv, NULL);
            time_start = tv.tv_sec * 1000000 + tv.tv_usec;
            if (*camera >> frames)
            {
                std::cout << "未获得图像!!!\n";
                continue;
            }
            for (int i = 0; i < camera->m_device_number; ++i)
            {
                if (frames[i].m_img.empty())
                {
                    getImage=0;
                }
            }

            video_number = camera->m_device_number;
            if(getImage==0)
            {
                std::cout << "get image  error" << std::endl;
                continue;
            }
            gettimeofday(&tv, NULL);
            time_end = tv.tv_sec * 1000000 + tv.tv_usec;
            std::cout << _yellow("test 读图耗时 : ")     << ((double)time_end - (double)time_start)/1000.0  << "\n";
            
        }   
        else if(state==Video) 
        {
            frames.resize(video_number);
            if(video_number==2)
            {
                if (video_test[0].read(frames[0].m_img) == 0||video_test[1].read(frames[1].m_img) == 0)
                {
                    getImage=0;
                }  
            }
            else
            {
                for(int i=0;i<2;i++)
                {
                    if(videoOpen[i]&&video_test[i].read(frames[0].m_img) == 0)
                    {
                            getImage=0;
                            break;
                    }
                }
            }    
           
            if(getImage==0)
            {
                std::cout << "get image  error" << std::endl;
                break;
            }
        }
        std::vector<wmj::Armor> final_armors(2);
        //final_armors.clear();
        
        //识别主体
        if (video_number)
        {
            

            gettimeofday(&tv, NULL);
            time_start = tv.tv_sec * 1000000 + tv.tv_usec;
            
            finalArmor = detector->DetectArmorDouble(frames, final_armors);
            
            gettimeofday(&tv, NULL);

            time_end = tv.tv_sec * 1000000 + tv.tv_usec;
        }
        else
        {
            std::cout << "两个全为空,退出操作" << "\n";
            exit(0);
        }
        //读取识别状态
        bool detected=final_armors[0].m_armor_type != wmj::ARMOR_NONE ? true : false;
        int frames_number=frames.size();
        

        //如果识别到装甲板，就把装甲板框出来，输出识别时间
        if (detected)
        {
            if(final_armors[0].m_detectedtype==wmj::DETECT_DOUBLE)   
            {
                for (int j = 0; j < 4; ++j)
                {
                    cv::line(frames[0].m_img, final_armors[0].m_vertices[j % 4], final_armors[0].m_vertices[(j + 1) % 4], cv::Scalar::all(255), 2);
                    cv::line(frames[1].m_img, final_armors[1].m_vertices[j % 4], final_armors[1].m_vertices[(j + 1) % 4], cv::Scalar::all(255), 2);
                }
            }
            else if(final_armors[0].m_detectedtype==wmj::DETECT_LEFT)
            {
                for (int j = 0; j < 4; ++j)
                {
                    cv::line(frames[0].m_img, final_armors[0].m_vertices[j % 4], final_armors[0].m_vertices[(j + 1) % 4], cv::Scalar::all(255), 2);
                }
            }
            else
            {
                if(frames_number==2)
                {
                    for (int j = 0; j < 4; ++j)
                    {
                        cv::line(frames[1].m_img, final_armors[0].m_vertices[j % 4], final_armors[0].m_vertices[(j + 1) % 4], cv::Scalar::all(255), 2);
                    } 
                }
                else
                {
                    for (int j = 0; j < 4; ++j)
                    {
                        cv::line(frames[0].m_img, final_armors[0].m_vertices[j % 4], final_armors[0].m_vertices[(j + 1) % 4], cv::Scalar::all(255), 2);
                    } 
                } 
            }
            std::cout << _yellow("test 识别耗时 : ")     << ((double)time_end - (double)time_start)/1000.0 << "\n";
            std::cout << _lightgreen("test 装甲坐标 : ") << finalArmor.m_position << std::endl;
            
        }
        else
        {      
            std::cout <<_red("test 识别失败!!!")<<std::endl;  
        } 
        std::cout << "test ************************************\n";

        
        //将位置输出在图片上
        float armorDis = std::sqrt(std::pow(finalArmor.m_position.x, 2) + std::pow(finalArmor.m_position.y, 2) + std::pow(finalArmor.m_position.z, 2));
        char position[128] = "";
        char eulaDis[128] = "";
        sprintf(position, "Position :[ %f, %f, %f]", finalArmor.m_position.x, finalArmor.m_position.y, finalArmor.m_position.z);
        sprintf(eulaDis, "EulaDis: %f", armorDis);
        if(detector->m_thread_number==2)
        {
            cv::putText(frames[0].m_img, position, cv::Point2f(100, 80), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 255), 2);
            cv::putText(frames[0].m_img, eulaDis, cv::Point2f(200, 160), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 255), 2);
            cv::putText(frames[1].m_img, position, cv::Point2f(100, 80), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 255), 2);
            cv::putText(frames[1].m_img, eulaDis, cv::Point2f(200, 160), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 255), 2);
            cv::namedWindow("result_left",cv::WINDOW_FREERATIO);
            cv::imshow("result_left", frames[0].m_img); 
            cv::namedWindow("result_right",cv::WINDOW_FREERATIO);
            cv::imshow("result_right", frames[1].m_img); 
        }
        else if(frames_number==2)
        {
            if(detector->m_singleChoose==0)

            {
                cv::putText(frames[0].m_img, position, cv::Point2f(100, 80), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 255), 2);
                cv::putText(frames[0].m_img, eulaDis, cv::Point2f(200, 160), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 255), 2);
                cv::namedWindow("result",cv::WINDOW_FREERATIO);
                cv::imshow("result", frames[0].m_img); 
            }
            else
            {
                cv::putText(frames[1].m_img, position, cv::Point2f(100, 80), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 255), 2);
                cv::putText(frames[1].m_img, eulaDis, cv::Point2f(200, 160), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 255), 2);
                cv::namedWindow("result",cv::WINDOW_FREERATIO);
                cv::imshow("result", frames[1].m_img); 
            }  
        }
        else
        {
            cv::putText(frames[0].m_img, position, cv::Point2f(100, 80), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 255), 2);
            cv::putText(frames[0].m_img, eulaDis, cv::Point2f(200, 160), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 255), 2);
            cv::namedWindow("result",cv::WINDOW_FREERATIO);
            cv::imshow("result", frames[0].m_img); 
        }

        rate_detect.sleep();

        char c = cv::waitKey(test_armor.waitKey_time);
        if (c == 27 || c == 'q')
        {
            break;

        } 
        else if (c == 's')
        {
            std::vector<cv::Mat> images = detector->m_ArmorSingles[detector->m_singleChoose].m_numbers;
            for (int i = 0; i < images.size(); ++i)
            {

                cv::imwrite("./image/" + std::to_string(cv::getTickCount()) + "_" + ".jpg",images[i]);
                std::cout << _red(std::to_string(cv::getTickCount()) + ".jpg"+"已存储") << std::endl;
            }
        }
    }
    
    std::cout << "over " << std::endl;
    return 0;
}


void initialize()
{
    std::cout << "欢迎来到双目自瞄代码识别程序\n";
    std::cout << "相机检测部分\n";
    checkCamera();
    std::cout << "相机检查完毕\n";
    state = State::Camera; //默认是相机
    test_armor.readParameter();
}
void checkCamera()
{
    camera->getCameraInfo();
    //for (int i = 0; i < camera->m_device_number; i++)
        //std::cout << "相机序列： " << i << "\n"
                  //<< "串口号 : " << camera->m_serial_number[i] << "\n";
}
void checkVideo()
{
    if (camera->m_device_number == 0)
    {
        std::cout << "没有相机，开启视频模式\n";
        state = State::Video;
       
        for (int i = 0; i < 2; i++)
        {
            videoOpen[i]=access(test_armor.video_path[i].data(), F_OK);
            videoOpen[i] = !videoOpen[i];
            if (access(test_armor.video_path[i].data(), F_OK) == 0)
            {
                //存在该路径
                video_test[i].open(test_armor.video_path[i]);
                video_number++;
                std::cout <<i+1<<":"<< test_armor.video_path[i] << " : 有效\n";
            }
            else
            {
                std::cout <<i+1<<":"<< test_armor.video_path[i] << " : 无效\n";
            }
        }
    }
}
