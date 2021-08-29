#include "../include/WMJRobotControl.h"

namespace wmj
{
    /**
     *@brief: 操作云台
     *
     *@param: uint8_t 0位选项, float 俯仰角, float 偏航角, float 俯仰角速度, float 偏航角速度
     */
    void WMJRobotControl::setGimbal(uint8_t mode, float pitch_angle, float yaw_angle, float pitch_speed, float yaw_speed)
    {
        GCMPack control_data{};
        Buffer data_to_send{};

        // angle
        pitch_angle = pitch_angle < 0 ? pitch_angle + 2 * PI : pitch_angle;
        yaw_angle   = yaw_angle   < 0 ? yaw_angle   + 2 * PI : yaw_angle;

        pitch_angle = pitch_angle > 2 * PI ? pitch_angle -= 2 * PI : pitch_angle;
        yaw_angle   = yaw_angle   > 2 * PI ? yaw_angle   -= 2 * PI : yaw_angle;

        // speed
        pitch_speed = std::fabs(pitch_speed) > PI ? (pitch_speed > 0 ? PI : - PI) : pitch_speed;
        yaw_speed   = std::fabs(yaw_speed)   > PI ? (yaw_speed   > 0 ? PI : - PI) : yaw_speed;

        int16_t s_pitch = (2047 * pitch_speed) / (2 * PI);
        int16_t s_yaw   = (2047 * yaw_speed)   / (2 * PI);

        s_pitch = s_pitch > 2047 ? 2047 : (s_pitch < -2048 ? -2048 : s_pitch);
        s_yaw   = s_yaw   > 2047 ? 2047 : (s_yaw   < -2048 ? -2048 : s_yaw);

        control_data.msg.gm_mode             = mode | _BaseCoor;
        control_data.msg.gm_yaw_velocity     = (uint8_t)s_yaw;
        control_data.msg.gm_both_velocity    = (uint8_t)((((uint16_t)s_yaw >> 8) & 0x0f) | (uint8_t)(s_pitch << 4));
        control_data.msg.gm_pitch_velocity   = (uint8_t)((uint16_t)s_pitch >> 4);
        control_data.msg.gm_pitch_angle_low  = (uint8_t)((uint16_t)(pitch_angle / PI * 180 / 360 * 65535));
        control_data.msg.gm_pitch_angle_high = (uint8_t)((uint16_t)(pitch_angle / PI * 180 / 360 * 65535) >> 8);
        control_data.msg.gm_yaw_angle_low    = (uint8_t)((uint16_t)(yaw_angle   / PI * 180 / 360 * 65535));
        control_data.msg.gm_yaw_angle_high   = (uint8_t)((uint16_t)(yaw_angle   / PI * 180 / 360 * 65535) >> 8);

        data_to_send.push_back((uint8_t)Gimbal);
        for(auto c : control_data.data)
            data_to_send.push_back(c);

        canBus->sendFrame(data_to_send);
    }

    /**
     *@brief: 获取云台姿态数据
     *
     *@return: std::vector<float> 俯仰角-偏航角-俯仰角速度-偏航角速度
     */
    std::vector<float> WMJRobotControl::getGimbal()
    {
        uint16_t pitch_angle{}, yaw_angle{}, pitch_speed{}, yaw_speed{};
        float f_pitch_angle = 0;
        float f_yaw_angle = 0;
        float f_pitch_speed = 0;
        float f_yaw_speed = 0;
        Buffer data_from_read;
        if(_BaseCoor == ChassisCoor)
        {
            data_from_read = canBus->readFrame(Gimbal);
       
            yaw_speed   = (int16_t)((int16_t)data_from_read[1] | ((int16_t)data_from_read[2]  & 0x0f) << 8);
           

            pitch_speed = (int16_t)(((int16_t)data_from_read[2] >> 4) | ((int16_t)data_from_read[3] << 4));
            yaw_angle   = (int16_t)((int16_t)(data_from_read[4]) | ((int16_t)(data_from_read[5]) << 8));
            pitch_angle = (int16_t)((int16_t)(data_from_read[6]) | ((int16_t)(data_from_read[7]) << 8));
            f_pitch_speed = float(pitch_speed / 2048.f * 2 * PI);
            f_yaw_speed   = float(yaw_speed   / 2048.f * 2 * PI);
            f_pitch_speed = f_pitch_speed > 2.f * PI ? (f_pitch_speed - 4.f * PI) : f_pitch_speed;
            f_yaw_speed   = f_yaw_speed   > 2.f * PI ? (f_yaw_speed   - 4.f * PI) : f_yaw_speed;

            f_pitch_angle = (float)pitch_angle / 65535.f * 2 * PI;
            f_yaw_angle   = (float)yaw_angle   / 65535.f * 2 * PI;
            f_pitch_angle = f_pitch_angle > PI ? (f_pitch_angle - 2.f * PI ) : f_pitch_angle;
            f_yaw_angle   = f_yaw_angle   > PI ? ( f_yaw_angle  - 2.f * PI ) : f_yaw_angle;

            float data[4] = {f_pitch_angle, f_yaw_angle, f_pitch_speed, f_yaw_speed};
            return std::vector<float>(data, data + 4);
        }
        else // if(_BaseCoor == GroundCoor)
        {
            uint16_t timestamp{}, roll_angle{};
            data_from_read = canBus->readFrame(Gyro);
            timestamp   = (uint16_t)data_from_read[0] | (uint16_t)data_from_read[1] << 8;
            yaw_angle   = (uint16_t)data_from_read[2] | (uint16_t)data_from_read[3] << 8;
            pitch_angle = (uint16_t)data_from_read[4] | (uint16_t)data_from_read[5] << 8;
            roll_angle  = (uint16_t)data_from_read[6] | (uint16_t)data_from_read[7] << 8;

            float fpitch = (float)pitch_angle / 65535.f * 2 * PI;
            float fyaw   = (float)yaw_angle   / 65535.f * 2 * PI - 2 * PI;
            float froll  = (float)roll_angle  / 65535.f * 2 * PI;

            wmj::GimbalPose cur_pose{fpitch, fyaw, froll};
            cur_pose.timestamp = timestamp;
            if(!poseInit)
            {
                lastpose = cur_pose;
                poseInit = true;
            }
            if(std::fabs(lastpose.yaw - cur_pose.yaw) > PI)
            {
                if(cur_pose.yaw - lastpose.yaw > 0)
                    circlecnt++;
                else
                    circlecnt--;
            }
            lastpose = cur_pose;
            cur_pose.yaw   = cur_pose.yaw - circlecnt * 2.f * PI;
            cur_pose.pitch = cur_pose.pitch > PI ? cur_pose.pitch - 2 * PI : cur_pose.pitch;

            float data[4] = {cur_pose.pitch, cur_pose.yaw, 0, 0};
            return std::vector<float>(data, data + 4);
        }
    }

    /**
     * @brief 使云台以最高速度转到目标角度
     *
     * @param float 俯仰角度 float 偏航角度
     */
    void WMJRobotControl::SetGimbalAngle(float pitch_angle, float yaw_angle)
    {
        setGimbal(0x33, pitch_angle, yaw_angle, 0, 0);
    }

    /**
     * @brief 设定云台的角速度
     *
     * @param float 俯仰速度 float 偏航速度
     */
    void WMJRobotControl::SetGimbalSpeed(float pitch_speed, float yaw_speed)
    {
        setGimbal(0x0f, 0, 0, pitch_speed, yaw_speed);
    }

    /**
     * @brief 设定云台的偏航角速度,俯仰角度
     *
     * @param float 俯仰角度 float 偏航速度
     */
    void WMJRobotControl::SetGimbal_YawSpeed_PitchAngle(float pitch_angle, float yaw_speed)
    {
        setGimbal(0x27, pitch_angle, 0, 0, yaw_speed);
    }

    /**
     * @brief 读取云台回传的角度数据
     *
     * @return GimbalPose
     */
    GimbalPose WMJRobotControl::GetGimbalAngle()
    {
        std::vector<float> gimbal_data = getGimbal();
        return GimbalPose{gimbal_data[0], gimbal_data[1]};
    }

    /**
     * @brief 获取云台当前角速度值
     *
     * @return
     */
    GimbalPose WMJRobotControl::GetGimbalSpeed()
    {
        std::vector<float> gimbal_data = getGimbal();
        return GimbalPose{gimbal_data[2], gimbal_data[3]};
    }
}
