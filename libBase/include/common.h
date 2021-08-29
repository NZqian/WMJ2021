#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <unistd.h>
#include <sys/time.h>
#include <thread>
#include <termio.h>

#define PI 3.14159265358
#define HINT std::cout << _warning("<<<<<\t" + std::string(__FILE__) + ":" + std::to_string(__LINE__) + "\t>>>>>") + "\n";

namespace wmj
{
    unsigned int getTimeUSecNow();

    double now();

    class Rate
    {
    public:
        Rate();
        Rate(unsigned int);
        double m_start_time_with_usec;

        void sleep();
    private:
        int m_LoopRate;
    };

    class MatWithTime
    {
    public:
        MatWithTime();
        MatWithTime(cv::Mat, double, std::string);
        ~MatWithTime();

        double m_time_stamp;
        std::string m_orientation;
        cv::Mat m_img;
    };

    struct GimbalPose
    {
        float  pitch;
        float  yaw;
        float  roll;
        double timestamp;

        GimbalPose(float pitch = 0.0, float yaw = 0.0, float roll = 0.0)
        {
            this->pitch     = pitch;
            this->yaw       = yaw;
            this->roll      = roll;
        }

        GimbalPose operator=(const GimbalPose& gm)
        {
            this->pitch     = gm.pitch;
            this->yaw       = gm.yaw;
            this->roll      = gm.roll;
            this->timestamp = gm.timestamp;
            return *this;
        }

        GimbalPose operator=(const float init_value)
        {
            this->pitch     = init_value;
            this->yaw       = init_value;
            this->roll      = init_value;
            this->timestamp = wmj::now();
            return *this;
        }

        friend GimbalPose operator-(const GimbalPose& gm1, const GimbalPose gm2)
        {
            GimbalPose temp{};
            temp.pitch     = gm1.pitch - gm2.pitch;
            temp.yaw       = gm1.yaw   - gm2.yaw;
            temp.roll      = gm1.roll  - gm2.roll;
            temp.timestamp = wmj::now();
            return temp;
        }

        friend GimbalPose operator+(const GimbalPose& gm1, const GimbalPose gm2)
        {
            GimbalPose temp{};
            temp.pitch     = gm1.pitch + gm2.pitch;
            temp.yaw       = gm1.yaw   + gm2.yaw;
            temp.roll      = gm1.roll  + gm2.roll;
            temp.timestamp = wmj::now();
            return temp;
        }

        friend GimbalPose operator*(const GimbalPose& gm, const float k)
        {
            GimbalPose temp{};
            temp.pitch     = gm.pitch * k;
            temp.yaw       = gm.yaw   * k;
            temp.roll      = gm.roll  * k;
            temp.timestamp = wmj::now();
            return temp;
        }

        friend GimbalPose operator*(const float k, const GimbalPose& gm)
        {

            GimbalPose temp{};
            temp.pitch     = gm.pitch * k;
            temp.yaw       = gm.yaw   * k;
            temp.roll      = gm.roll  * k;
            temp.timestamp = wmj::now();
            return temp ;
        }

        friend GimbalPose operator/(const GimbalPose& gm, const float k)
        {
            GimbalPose temp{};
            temp.pitch     = gm.pitch / k;
            temp.yaw       = gm.yaw   / k;
            temp.roll      = gm.roll  / k;
            temp.timestamp = wmj::now();
            return temp;
        }

        friend std::ostream& operator<<(std::ostream& out, const GimbalPose& gm)
        {
            out << "[pitch : " << gm.pitch << ", yaw : " << gm.yaw << "]";
            return out;
        }
    };
}

void monitorKeyboard(char*);

std::string _red(std::string content);
std::string _lightred(std::string content);
std::string _green(std::string content);
std::string _lightgreen(std::string content);
std::string _yellow(std::string content);
std::string _brown(std::string content);
std::string _blue(std::string content);
std::string _lightblue(std::string content);
std::string _purple(std::string content);
std::string _lightpurple(std::string content);
std::string _cyan(std::string content);
std::string _lightcyan(std::string content);
std::string _white(std::string content);
std::string _warning(std::string content);
std::string _underline(std::string content);
std::string _reverse(std::string content);

void _clear();

double R2D(double rad);
double D2R(double degree);
