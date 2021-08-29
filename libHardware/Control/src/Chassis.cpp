#include "../include/WMJRobotControl.h"

namespace wmj
{
    /*
     * @brief 操作底盘
     *
     * @param: uint8_t 电机力矩使能, uint8_t 坐标系, float x速度, float y速度, float x距离, float y距离, float 目标角度
     */
    void WMJRobotControl::setChassis(uint8_t info1, uint8_t func, float x_speed, float y_speed, float x_distance, float y_distance, float target_angular)
    {
        CHMPack control_data{};
        Buffer data_to_send{};

        // x_speed
        x_speed = std::fabs(x_speed) > 6 ? 6 * x_speed / std::fabs(x_speed) : x_speed;
        x_speed = x_speed / 6 * 128 + 128;

        // y_speed
        y_speed = std::fabs(y_speed) > 6 ? 6 * y_speed / std::fabs(y_speed) : y_speed;
        y_speed = y_speed / 6 * 128 + 128;

        // x_distance
        x_distance = std::fabs(x_distance) > 1 ? 1 * x_distance / std::fabs(x_distance) : x_distance;
        x_distance = x_distance / 1 * 128 + 128;

        // y_distance
        y_distance = std::fabs(y_distance) > 1 ? 1 * y_distance / std::fabs(y_distance) : y_distance;
        y_distance = y_distance / 1 * 128 + 128;

        // target_angular
        target_angular = R2D(target_angular);
        target_angular = target_angular > 360 ? 360 : target_angular < 0 ? 0 : target_angular;

        control_data.msg.cha_info1           = info1;
        control_data.msg.cha_info2           = func;
        control_data.msg.cha_x_speed         = (uint8_t)x_speed;
        control_data.msg.cha_y_speed         = (uint8_t)y_speed;
        control_data.msg.cha_x_distance      = (uint8_t)x_distance;
        control_data.msg.cha_y_distance      = (uint8_t)y_distance;
        control_data.msg.cha_target_angle    = (uint8_t)(target_angular / 2);
        control_data.msg.cha_placehoder1     = 0;

        data_to_send.push_back((uint8_t)Chassis);
        for(auto c : control_data.data)
            data_to_send.push_back(c);

        canBus->sendFrame(data_to_send);
    }

    /*
     * @brief 小陀螺
     *
     * @param: bool 缓冲能量使能
     */
    void WMJRobotControl::top(bool power)
    {
        uint8_t unleash_power = 0x00;
        if(power)
            unleash_power = CHA_POWER;

        this->setChassis(0x17, 0x00 | unleash_power | CHA_TOP, 0, 0, 0, 0, 0);
    }

    /*
     * @brief 刹车
     */
    void WMJRobotControl::brake()
    {
        this->setChassis(0x17, 0x00, 0, 0, 0, 0, 0);
    }

    /*
     * @brief 向前
     *
     * @param: float 速度, bool 缓冲能量使能
     */
    void WMJRobotControl::moveFront(float speed, bool power)
    {
        uint8_t unleash_power = 0x00;
        if(power)
            unleash_power = CHA_POWER;

        this->setChassis(0x17, 0x00 | unleash_power, 0, speed, 0, 0, 0);
    }

    /*
     * @brief 向后
     *
     * @param: float 速度, bool 缓冲能量使能
     */
    void WMJRobotControl::moveBack(float speed, bool power)
    {
        uint8_t unleash_power = 0x00;
        if(power)
            unleash_power = CHA_POWER;

        this->setChassis(0x17, 0x00 | unleash_power, 0, -speed, 0, 0, 0);
    }

    /*
     * @brief 向右
     *
     * @param: float 速度, bool 缓冲能量使能
     */
    void WMJRobotControl::moveRight(float speed, bool power)
    {
        uint8_t unleash_power = 0x00;
        if(power)
            unleash_power = CHA_POWER;

        this->setChassis(0x17, 0x00 | unleash_power, speed, 0, 0, 0, 0);
    }

    /*
     * @brief 向左
     *
     * @param: float 速度, bool 缓冲能量使能
     */
    void WMJRobotControl::moveLeft(float speed, bool power)
    {
        uint8_t unleash_power = 0x00;
        if(power)
            unleash_power = CHA_POWER;

        this->setChassis(0x17, 0x00 | unleash_power, -speed, 0, 0, 0, 0);
    }

    /*
     * @brief 设置目标角度
     *
     * @param: float 角度
     */
    void WMJRobotControl::chassisAngle(float angle)
    {
        this->setChassis(0x17, 0x00, 0, 0, 0, 0, angle);
    }

    /*
     * @brief 设置速度和角速度 导航测试用接口
     *
     * @param: uint8_t 电机力矩使能, uint8_t 坐标系, float x速度, float y速度, float 角速度
     */
    void WMJRobotControl::setNavigationChassis(uint8_t info1, uint8_t func, float x_speed, float y_speed, float angular_speed)
    {
        CHMPack control_data{};
        Buffer data_to_send{};

        // x_speed
        x_speed = x_speed / 5. * 32768.;

        // y_speed
        y_speed = y_speed / 5. * 32768.;

        // angular_speed
        angular_speed = R2D(angular_speed);
        angular_speed = angular_speed * 32768. / 180.;

        control_data.msg.cha_info1           = info1;
        control_data.msg.cha_info2           = func;
        control_data.msg.cha_x_speed         = (int8_t)((int16_t)x_speed);
        control_data.msg.cha_y_speed         = (int8_t)((int16_t)x_speed >> 8);
        control_data.msg.cha_x_distance      = (int8_t)((int16_t)y_speed);
        control_data.msg.cha_y_distance      = (int8_t)((int16_t)y_speed >> 8);
        control_data.msg.cha_target_angle    = (int8_t)((int16_t)angular_speed);
        control_data.msg.cha_placehoder1     = (int8_t)((int16_t)angular_speed >> 8);

        data_to_send.push_back((uint8_t)Chassis);
        for(auto c : control_data.data)
            data_to_send.push_back(c);
        canBus->sendFrame(data_to_send);
    }

    /*
     * @brief 设置速度和角速度 导航测试调用接口
     *
     * @param: float x速度, float y速度, float 角速度, bool 缓冲能量使能
     */
    void WMJRobotControl::navigate(float x_speed, float y_speed, float angular_speed, bool power)
    {
        uint8_t unleash_power = 0x00;
        if(power)
            unleash_power = CHA_POWER;
        this->setNavigationChassis(0x5f, 0x00 | CHA_AUTO | unleash_power, x_speed, y_speed, angular_speed);
    }
}
