#include "../include/WMJRobotControl.h"
#include <bits/stdint-uintn.h>

namespace wmj
{
    /**
     * @brief WMJRobotControl构造函数
     */
    WMJRobotControl::WMJRobotControl() : lastpose{0.f, 0.f}, poseInit{false}, circlecnt{0}
    {
        cv::FileStorage fs(CONTROL_CFG, cv::FileStorage::READ);
        fs["port_type"]             >> m_port_type;
        fs["fifteen_shoot_speed"]   >> m_fifteen_shoot_speed;
        fs["eighteen_shoot_speed"]  >> m_eighteen_shoot_speed;
        fs["thirty_shoot_speed"]    >> m_thirty_shoot_speed;
        fs.release();

        switch(m_port_type)
        {
            case 0:
                this->canBus = std::make_shared<wmj::SerialPort>();
                this->gyroCanBus = std::make_shared<wmj::SerialPort>();
                break;
            case 1:
                this->canBus = std::make_shared<wmj::CanPort>("can0") ;
                this->gyroCanBus = std::make_shared<wmj::CanPort>("can0");
                break;
        }
        this->LastState = STATE_IDLE;
        // this->InTopMode = false;
        // this->TopMode = false;
    }

    /**
     * @brief 切换基坐标系：Mode = false为底盘系，Mode = true为地面系
     *
     * @param Mode
     */
    void WMJRobotControl::SwitchBaseCoor(bool Mode)
    {
        _BaseCoor = (uint8_t)Mode * 0x40 ;
    }

    /*
     * @brief 向主控板发送相关信息
     *
     * @param armor
     * @param capture
     * @param distance
     * @return true
     * @return false
     */
    bool WMJRobotControl::sendInfo(uint8_t armor, uint8_t exposure_state, bool capture, float distance)
    {
        MCMPack control_data{} ;
        Buffer data_to_send{} ;

        //exposure_state 0x01, capture true正在自动曝光
        //exposure_state 0x02, capture true完成自动曝光
        switch(armor)
        {
            case 0x00:
                control_data.msg.mctl_base = 0x00 | 0x04 * capture;
                control_data.msg.mctl_if_end_exposure = exposure_state * capture;
                break;
            case 0x01:
                control_data.msg.mctl_base = 0x01 | 0x04 * capture;
                control_data.msg.mctl_if_end_exposure = exposure_state * capture;
                break;
            case 0x02:
                control_data.msg.mctl_base = 0x03 | 0x04 * capture;
                control_data.msg.mctl_if_end_exposure = exposure_state * capture;
                break;
            default:
                control_data.msg.mctl_base = 0x00 ;
                control_data.msg.mctl_if_end_exposure = exposure_state * capture;
        }
        control_data.msg.mctl_distance_low = (uint8_t)((uint16_t)distance) ;
        control_data.msg.mctl_distance_high = (uint8_t)((uint16_t)distance >> 8) ;
        control_data.msg.placehoder1 = 0 ;
        control_data.msg.placehoder2 = 0 ;
        control_data.msg.placehoder3 = 0 ;
        control_data.msg.placehoder4 = 0 ;

        data_to_send.push_back((uint8_t)MainControl);
        for( auto c : control_data.data )
            data_to_send.push_back(c);

        canBus->sendFrame(data_to_send) ;
        return true ;
    }

    //////////////////////// 按键通讯协议 ////////////////////////
    /**
     * data[1]  0   S1_low      数据1~3分别代表左拨杆高低中
     * data[1]  1   S1_high
     * data[1]  2   S2_low      数据1~3分别代表右拨杆高低中
     * data[1]  3   S2_high
     * data[1]  4   mouse_left  自动击发
     * data[1]  5   mouse_right 自瞄模式
     *
     * data[6]  0   W           前进
     * data[6]  1   S           后退
     * data[6]  2   A           左移
     * data[6]  3   D           右移
     * data[6]  4   shift       加速
     * data[6]  5   ctrl        减速
     * data[6]  6   Q
     * data[6]  7   E           右键+E反陀螺
     *
     * data[7]  0   R           关弹仓；Shift+R开弹仓
     * data[7]  1   F           打大符
     * data[7]  2   G           干扰对方打符
     * data[7]  3   Z           右键+Z自动曝光 Z切回默认曝光
     * data[7]  4   X           右键+X使用位姿预测，只按右键默认使用绝对坐标预测
     * data[7]  5   C
     * data[7]  6   V           固定底盘
     * data[7]  7   B           开启陀螺
     */
    ////////////////////////////////////////////////////////////

    /**
     * @brief 获取当前按键信息
     *
     * @return KeyboardInfo
     */
    KeyboardInfo WMJRobotControl::GetKeyboardInfo()
    {
        Buffer data_from_read = canBus->readFrame((int)MainControl) ;
        //bool aim_a, aim_r0, aim_r1, Rins, Rdes;
        wmj::KeyboardInfo Info;
        if(data_from_read.size() > 0)
        {
            // 自瞄模式
            // data1 0x05 左上右上
            // data1 0x09 左上右下
            // data1 0x20 右键
            // data1 0x30 左右键
            // 自动曝光 右键+Z
            // data7 0x08 Z
            // 位姿预测模式 右键+X
            // data7 0x10 X
            // 反陀螺模式 右键+E
            // data6 0x80 E
            // data7 0x02 F
            // 键鼠模式按右键或左右键同按
            if((uint8_t)data_from_read[1] == 0x25 || (uint8_t)data_from_read[1] == 0x29 || (uint8_t)data_from_read[1] == 0x35 || (uint8_t)data_from_read[1] == 0x39)
            {
                // 自动曝光
                // data7 0x08 Z
                // 其他键不能按下
                if(((uint8_t)data_from_read[7] & 0x08) && !((uint8_t)data_from_read[7] & 0xf7))
                {
                    Info.auto_exposure = true;
                }
                else
                {
                    Info.auto_exposure = false;
                }
                // 位姿预测模式
                // data7 0x10 X
                if((uint8_t)data_from_read[7] & 0x10)
                {
                    Info.use_pose_predict = true;
                    Info.auto_exposure = false;
                }
                else
                {
                    Info.use_pose_predict = false;
                }
                // 击发
                // data1 0x10 左键
                if((uint8_t)data_from_read[1] == 0x35 || (uint8_t)data_from_read[1] == 0x39)
                {
                    Info.enable_shoot = true;
                    Info.auto_exposure = false;
                }
                else
                {
                    Info.enable_shoot = false;
                }
                // 反陀螺
                // data6 0x80 右键+E
                // data7 0x02 F（不能按下）
                if(((uint8_t)data_from_read[6] & 0x80) && !((uint8_t)data_from_read[7] & 0x02))
                {
                    Info.aim_top = true;
                }
                else
                {
                    Info.aim_top = false;
                }
                Info.use_number_detect = true;
                Info.aim_armor = true;
            }
            else
            {
                Info.aim_armor = false;
                Info.enable_shoot = false;
                Info.use_pose_predict = false;
                Info.auto_exposure = false;
                Info.use_number_detect = true;
            }

            // 小符模式
            // data7 0x02 F
            // data6 0x80 E（不能按下）
            // data1 0x20 右键（不能按下）
            if(((uint8_t)data_from_read[7] & 0x02) && !((uint8_t)data_from_read[6] & 0x80) && !((uint8_t)data_from_read[1] & 0x20))
            {
                Info.aim_rune_line = true;
            }
            else
            {
                Info.aim_rune_line = false;
            }

            // 大符模式
            // data7 0x04 G
            // data6 0x80 E（不能按下）
            // data1 0x20 右键（不能按下）
            if(((uint8_t)data_from_read[7] & 0x04) && !((uint8_t)data_from_read[6] & 0x80) && !((uint8_t)data_from_read[1] & 0x20))
            {
                Info.aim_rune_sine = true;
            }
            else
            {
                Info.aim_rune_sine = false;
            }

            // 切回默认曝光值 右键不能按下
            // data7 0x08 Z
            if(!((uint8_t)data_from_read[1] & 0x20) && ((uint8_t)data_from_read[7] & 0x08))
            {
                Info.use_default_exposure = true;
            }
            else
            {
                Info.use_default_exposure = false;
            }
        }
        //std::cout << "[info]" << ((uint8_t)data_from_read[1] & 0x10) << " "
        //                      << ((uint8_t)data_from_read[1] & 0x20) << " "
        //                      << ((uint8_t)data_from_read[7] & 0x02) << " "
        //                      << ((uint8_t)data_from_read[1]) << " "
        //                      << ((uint8_t)data_from_read[7]) << std::endl
        //          << "[info]aim_armor:\t" << Info.aim_armor << std::endl
        //          << "[info]aim_top:\t" << Info.aim_top << std::endl
        //          << "[info]aim_rune:\t" << Info.aim_rune << std::endl
        //          << "[info]aim_dark\t" << Info.aim_dark << std::endl
        //          << "[info]auto_expo:\t" << Info.auto_exposure << std::endl
        //          << "[info]default_expo:\t" << Info.use_default_exposure << std::endl
        //          << "[info]num_detect:\t" << Info.use_number_detect << std::endl
        //          << "[info]pose_predict:\t" << Info.use_pose_predict << std::endl
        //          << "[info]enable_shoot:\t" << Info.enable_shoot << std::endl;
        return Info;
    }

    /**
     *@brief: 获取发送的命令
     *
     *@return: ROBOSTATE 发送的命令
     */
    ROBO_STATE WMJRobotControl::GetRobotStatus()
    {
        KeyboardInfo robotStatus;
        robotStatus = this->GetKeyboardInfo();
        ROBO_STATE curState{STATE_IDLE};
        if(robotStatus.aim_armor)
        {
            if(robotStatus.auto_exposure) // 自动曝光
            {
                this->m_auto_exposure = true;
                std::cout << "ARMOR AUTOEXPOSURE\n";
            }
            else
            {
                this->m_auto_exposure = false;
            }

            if(robotStatus.use_pose_predict) // 使用位姿预测
            {
                this->m_use_pose_predict = true;
                std::cout << "ARMOR USE POSE PREDICT\n";
            }
            else // 不使用位姿预测
            {
                this->m_use_pose_predict = false;
                std::cout << "ARMOR USE POINT PREDICT\n";
            }

            if(robotStatus.enable_shoot) // 击发 不能自动曝光
            {
                this->m_enable_shoot = true;
                this->m_auto_exposure = false;
                std::cout << "ARMOR SHOOTING\n";
            }
            else
            {
                this->m_enable_shoot = false;
            }

            if(robotStatus.aim_top)
            {
                curState = STATE_TOP;
                TopMode = true;
            }
            else
            {
                TopMode = false;
                curState = STATE_ARMOR;
            }

        }

        if(robotStatus.use_default_exposure)
        {
            this->m_use_default_exposure = true;
        }
        else
        {
            this->m_use_default_exposure = false;
        }

        if(robotStatus.aim_rune_line)
        {
            curState = STATE_RUNE;
        }

        if(robotStatus.aim_rune_sine)
        {
            curState = STATE_RUNE_SINE;
        }

        LastState = curState;
        std::cout << "[ROBO_STATE]\t" << curState << std::endl;
        return curState;
    }
}
