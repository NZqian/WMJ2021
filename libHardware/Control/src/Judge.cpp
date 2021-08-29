#include "../include/WMJRobotControl.h"
#include <bits/stdint-intn.h>
#include <cmath>

namespace wmj
{
    ROBO_GAME WMJRobotControl::getGameState()
    {
        Buffer data_from_read{};
        return (ROBO_GAME)(data_from_read[0] & 0x07);
    }

    /**
     * @brief: 获取当前能量机关状态
     *
     * @retrun: wmj::ROBO_ENERGY ENERGY_IDLE 0x00不可激活, ENERGY_SMALL 0x01小能量, ENERGY_BIG 0x02大能量
     */
    ROBO_ENERGY WMJRobotControl::getEnergyStatus()
    {
        Buffer data_from_read = canBus->readFrame(Judge);
        return (ROBO_ENERGY)(data_from_read[0] & 0x03);
    }

    /**
     * @brief: 接收弹速回传计算平均值用以弹道补偿的回传
     *
     * @return: float 弹速
     */
    float WMJRobotControl::getShootSpeedValue()
    {
        Buffer data_from_read = canBus->readFrame(Judge);
        ROBO_SHOOT shoot_level = (ROBO_SHOOT)((data_from_read[0] & 0x0C) >> 2);
        float f_shoot_speed = 0;
        f_shoot_speed = ((int16_t)(((int16_t)data_from_read[1]) | ((uint16_t)data_from_read[2] << 8))) / 10.f;

        // 等级切换了 现在算出的弹速是上一等级的 弃用并清空队列 返回默认值
        // 为了读取实时弹速 Buffer会清空 设置判断条件SPEED_IDLE
        if(m_shoot_level != shoot_level && shoot_level != wmj::ROBO_SHOOT::SPEED_IDLE)
        {
            m_prev_shoot_speed.clear();
            m_shoot_level = shoot_level;
            m_shoot_speed_initialized = false;
            if(m_shoot_level == SPEED_HIGH)
            {
                return m_thirty_shoot_speed;
            }
            else if(m_shoot_level == SPEED_MID)
            {
                return m_eighteen_shoot_speed;
            }
            else
            {
                return m_fifteen_shoot_speed;
            }
        }

        // 回传不是0
        if(f_shoot_speed > 0)
        {
            m_shoot_speed_initialized = true;

            m_prev_shoot_speed.push_back(f_shoot_speed);
            if(m_prev_shoot_speed.size() == 11)
            {
                m_cur_shoot_speed = (10 * m_cur_shoot_speed - m_prev_shoot_speed.front() + m_prev_shoot_speed.back()) / 10.f;
                m_prev_shoot_speed.pop_front();
            }
            else
            {
                m_cur_shoot_speed = (m_cur_shoot_speed * (m_prev_shoot_speed.size() - 1) + f_shoot_speed) / m_prev_shoot_speed.size();
            }
            return m_cur_shoot_speed;
        }
        else // 等级切换后弹速回传仍是0
        {
            if(m_prev_shoot_speed.size() > 0)
                return m_cur_shoot_speed;
            if(m_shoot_level == SPEED_HIGH)
            {
                return m_thirty_shoot_speed;
            }
            else if(m_shoot_level == SPEED_MID)
            {
                return m_eighteen_shoot_speed;
            }
            else
            {
                return m_fifteen_shoot_speed;
            }
        }
    }

    bool WMJRobotControl::getVirtualShield()
    {
        Buffer data_from_read{};
        return (bool)(data_from_read[0] & 0x20);
    }

    ROBO_GAIN WMJRobotControl::getRobotGain()
    {
        Buffer data_from_read{};
        return (ROBO_GAIN)(data_from_read[0] & 0xc0);
    }

    ROBO_RFID WMJRobotControl::getRobotRFID()
    {
        Buffer data_from_read{};
        return (ROBO_RFID)(data_from_read[1] & 0x03);
    }

    int WMJRobotControl::getLeftHP()
    {
        Buffer data_from_read{};
        return (uint16_t)data_from_read[2] | ((uint16_t)(data_from_read[3] & 0x0f) << 8);
    }

    int WMJRobotControl::getBaseHP()
    {
        Buffer data_from_read{};
        return (uint16_t)(data_from_read[3] >> 4) | ((uint16_t)(data_from_read[4]) << 4);
    }

    int WMJRobotControl::getOutpostHP()
    {
        Buffer data_from_read{};
        return (uint8_t)data_from_read[5];
    }
}
