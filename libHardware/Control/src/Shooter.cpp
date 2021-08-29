#include "../include/WMJRobotControl.h"

namespace wmj
{
    /**
     *@brief: 是否拉高射速
     *
     *@return: bool 是否拉高
     */
    bool WMJRobotControl::HighShootSpeed()
    {
        Buffer data_from_read = canBus->readFrame((int)MainControl);
        if(data_from_read.size() > 0)
            if((uint8_t)data_from_read[7] & 0x8)
                return true;
        return false;
    }

    /**
     * @brief 获取连发剩余子弹数
     *
     * @return int
     */
    int WMJRobotControl::GetBulletNumber()
    {
        Buffer data_from_read = canBus->readFrame(Shooter) ;
        return (int)data_from_read[1] ;
    }

    /**
     *@brief: 连发子弹
     *
     *@param: int 发射数 float 子弹射速 float 拨弹轮转速
     */
    void WMJRobotControl::ShootSome(int BulletNum)
    {

        SCMPack control_data{};
        Buffer data_to_send{};
        if(m_box_open)
            control_data.msg.shot_mode       = 0x1F;
        else
            control_data.msg.shot_mode       = 0x0F;

        if((m_stop_shoot_compulsive && BulletNum == 0) || (m_keep_shoot && BulletNum == 0))
            return;

        m_stop_shoot_compulsive = false;
        m_keep_shoot =false;

        control_data.msg.shot_rub_speed_low    = (uint8_t)((uint16_t)4500);
        control_data.msg.shot_rub_speed_high   = (uint8_t)((uint16_t)4500 >> 8);
        control_data.msg.shot_num            = (uint8_t) BulletNum;
        control_data.msg.shot_boost_speed_low  = 0;
        control_data.msg.shot_boost_speed_high = 0;
        control_data.msg.placehoder1         = 0;
        control_data.msg.placehoder2         = 0;
        data_to_send.push_back((uint8_t)Shooter);

        for( auto c : control_data.data)
            data_to_send.push_back(c);

        canBus->sendFrame(data_to_send);
    }

    /**
     *@brief: 持续发射，给0才会停
     */
    void WMJRobotControl::KeepShoot()
    {
        SCMPack control_data{};
        Buffer data_to_send{};

        if(m_box_open)
            control_data.msg.shot_mode       = 0x1F;
        else
            control_data.msg.shot_mode       = 0x0F;

        if(!m_keep_shoot)
            m_keep_shoot = true;

        m_stop_shoot_compulsive = false;

        control_data.msg.shot_rub_speed_low    = (uint8_t)((uint16_t)4500);
        control_data.msg.shot_rub_speed_high   = (uint8_t)((uint16_t)4500 >> 8);
        control_data.msg.shot_num            = (uint8_t) 1;
        control_data.msg.shot_boost_speed_low  = (uint8_t)((uint16_t)2000);
        control_data.msg.shot_boost_speed_high = (uint8_t)((uint16_t)2000 >> 8);
        control_data.msg.placehoder1         = 0;
        control_data.msg.placehoder2         = 0;
        data_to_send.push_back((uint8_t)Shooter);

        for( auto c : control_data.data)
            data_to_send.push_back(c);

        canBus->sendFrame(data_to_send);
    }

    /**
     *@brief: 停止发射
     */
    void WMJRobotControl::StopShoot()
    {
        SCMPack control_data{};
        Buffer data_to_send{};

        if(!m_stop_shoot_compulsive)
            m_stop_shoot_compulsive = true;

        m_keep_shoot = false;

        if(m_box_open)
            control_data.msg.shot_mode       = 0x1F;
        else
            control_data.msg.shot_mode       = 0x0F;

        control_data.msg.shot_rub_speed_low    = (uint8_t)((uint16_t)0);
        control_data.msg.shot_rub_speed_high   = (uint8_t)((uint16_t)0);
        control_data.msg.shot_num            = (uint8_t) 0;
        control_data.msg.shot_boost_speed_low  = 0;
        control_data.msg.shot_boost_speed_high = 0;
        control_data.msg.placehoder1         = 0;
        control_data.msg.placehoder2         = 0;
        data_to_send.push_back((uint8_t)Shooter);

        for( auto c : control_data.data)
            data_to_send.push_back(c);

        canBus->sendFrame(data_to_send);
    }

    /**
     *@brief: 单发子弹
     *
     *@param: float 子弹射速 float 拨弹轮转速
     */
    void WMJRobotControl::ShootNone()
    {
        this->ShootSome(0);
    }

    /**
     *@brief: 单发子弹
     *
     *@param: float 子弹射速 float 拨弹轮转速
     */
    void WMJRobotControl::ShootOnce()
    {
        ShootSome(1);
    }

    /**
     *@brief: 开弹仓
     */
    void WMJRobotControl::openBox()
    {
        SCMPack control_data{};
        Buffer data_to_send{};

        if(m_box_open == false)
        {
            control_data.msg.shot_mode = 0x1F;
        }
        else
        {
            control_data.msg.shot_mode = 0x0F;
        }
        m_box_open = !m_box_open;
        data_to_send.push_back((uint8_t)Shooter);

        for( auto c : control_data.data)
            data_to_send.push_back(c);

        canBus->sendFrame(data_to_send);
    }
}
