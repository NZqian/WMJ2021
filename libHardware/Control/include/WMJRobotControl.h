#pragma once
#include "../../Transport/SerialPort/include/SerialPort.hpp"
#include "../../Transport/CanPort/include/CanPort.hpp"
#include <bits/stdint-uintn.h>
#include <queue>

#define Pi 3.141592653

namespace wmj
{
    class WMJRobotControl
    {
    public:
        bool m_enable_shoot;
        bool m_auto_exposure;
        bool m_use_default_exposure = true;

        bool m_use_number_detect = true;
        bool m_use_pose_predict = false;

        bool m_shoot_speed_initialized = false;
        ROBO_SHOOT m_shoot_level = wmj::ROBO_SHOOT::SPEED_IDLE;
        std::deque<float> m_prev_shoot_speed;
        float m_cur_shoot_speed = 0;
        float m_fifteen_shoot_speed;
        float m_eighteen_shoot_speed;
        float m_thirty_shoot_speed;

        WMJRobotControl() ;
        ~WMJRobotControl() {};

        //陀螺仪通信
        std::pair<int, cv::Point3d> getGyroAngle();
        std::pair<int, cv::Point3d> getGyroVelocity();
        std::pair<int, cv::Point3d> getGyroAcce();
        std::vector<std::pair<int, cv::Point3d>> GetGyroInfo();

        //底盘通讯
        void brake();
        void top(bool power = 0);
        void moveFront(float speed, bool power = 0);
        void moveBack(float speed, bool power = 0);
        void moveRight(float speed, bool power = 0);
        void moveLeft(float speed, bool power = 0);
        void chassisAngle(float angle);
        void navigate(float x_speed, float y_speed, float angular_speed, bool power);

        //云台通讯
        void SwitchBaseCoor(bool Mode);
        void SetGimbalAngle(float pitch_angle, float yaw_angle);
        void SetGimbalSpeed(float pitch_speed, float yaw_speed);
        void SetGimbal_YawSpeed_PitchAngle(float pitch_angle, float yaw_speed);
        GimbalPose GetGimbalAngle();
        GimbalPose GetGimbalSpeed();

        //射击通讯
        int GetBulletNumber();
        void openBox();
        void ShootNone();
        void ShootOnce();
        void ShootSome(int BulletNum);
        void KeepShoot();
        void StopShoot();

        //top aimer
        bool TopMode; //true if top key is pressed
        bool InTopMode; //true if first enter top mode
        cv::Point3d GyroBase;

        // 主控通讯
        bool sendInfo(uint8_t, uint8_t, bool, float);
        bool HighShootSpeed();
        KeyboardInfo GetKeyboardInfo();
        ROBO_STATE GetRobotStatus();
        ROBO_STATE LastState;

        // 裁判系统绘图
        void drawCircle(ROBO_LAYER layer, ROBO_COLOR color, cv::Point center_point, int radius);
        void drawLine(ROBO_LAYER layer, ROBO_COLOR color, cv::Point center_point, cv::Point end_point);
        void drawRect(ROBO_LAYER layer, ROBO_COLOR color, cv::Point center_point, cv::Point diag_point);
        void drawOval(ROBO_LAYER layer, ROBO_COLOR color, cv::Point center_point, cv::Point width_height);
        void clearAll();
        void clearLayer(ROBO_LAYER layer);

        // 裁判系统接收
        ROBO_GAME   getGameState();
        ROBO_ENERGY getEnergyStatus();
        float       getShootSpeedValue();
        bool        getVirtualShield();
        ROBO_GAIN   getRobotGain();
        ROBO_RFID   getRobotRFID();
        int         getLeftHP();
        int         getBaseHP();
        int         getOutpostHP();

        // 机器人通讯
        uint8_t recvFromRobot();
        void sendToRobot(ROBO_ID, uint8_t test);


    private:
        enum
        {
            ChassisCoor,
            GroundCoor
        };
        std::shared_ptr<wmj::Port> canBus;
        std::shared_ptr<wmj::Port> gyroCanBus;
        uint8_t _BaseCoor{ChassisCoor * 0x40};
        wmj::GimbalPose lastpose;

        bool m_box_open = false;
        bool m_keep_shoot = false;
        bool m_stop_shoot_compulsive = false;
        bool poseInit;
        int m_port_type;
        int circlecnt;

        std::vector<float> getGimbal();
        void setGimbal(uint8_t mode, float pitch_angle, float yaw_angle, float pitch_speed, float yaw_speed);
        void setChassis(uint8_t info1, uint8_t func, float x_speed, float y_speed, float x_distance, float y_distance, float target_angle);
        void setNavigationChassis(uint8_t info1, uint8_t func, float x_speed, float y_speed, float target_angle);
        void drawShape(ROBO_OPT opt, ROBO_SHAPE shape, ROBO_LAYER layer, ROBO_COLOR color, cv::Point start_point, cv::Point end_point, ROBO_DELETE del = DELETE_NONE);
    };
}
