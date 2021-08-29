#include "WMJRobotControl.h"

namespace wmj
{
    std::pair<int, cv::Point3d> WMJRobotControl::getGyroAngle()
    {
        Buffer data_from_read;
        GyroAnglePack gyro_angle_data{};
        data_from_read = gyroCanBus->readFrame(GyroAngle);
        
        std::copy(data_from_read.begin(), data_from_read.end(), gyro_angle_data.data);
        
        int     timestamp    = (int16_t)((int16_t)data_from_read[0] | (int16_t)data_from_read[1] << 8);
        double  yaw_angle    = (int16_t)((int16_t)data_from_read[2] | (int16_t)data_from_read[3] << 8);
        double  pitch_angle  = (int16_t)((int16_t)data_from_read[4] | (int16_t)data_from_read[5] << 8);
        double  roll_angle   = (int16_t)((int16_t)data_from_read[6] | (int16_t)data_from_read[7] << 8);


        yaw_angle   = (double)yaw_angle   / 65535. * 180.;
        pitch_angle = (double)pitch_angle / 65535. * 180.;
        roll_angle  = (double)roll_angle  / 65535. * 180.;
        cv::Point3d cur_gyro_angle{yaw_angle, pitch_angle, roll_angle};
        if(InTopMode){
            GyroBase = cur_gyro_angle;
        }
        if(TopMode){
            cur_gyro_angle -= GyroBase;
        }

        return std::make_pair(timestamp, cur_gyro_angle);
    }

    std::pair<int, cv::Point3d> WMJRobotControl::getGyroVelocity()
    {
        Buffer data_from_read;
        GyroAngularVelocityPack gyro_velocity_data{};
        data_from_read = gyroCanBus->readFrame(GyroAngularVelocity);
        std::copy(data_from_read.begin(), data_from_read.end(), gyro_velocity_data.data);

        int    timestamp      = (int16_t)((int16_t)data_from_read[0] | (int16_t)data_from_read[1] << 8);
        double yaw_velocity   = (int16_t)((int16_t)data_from_read[2] | (int16_t)data_from_read[3] << 8);
        double pitch_velocity = (int16_t)((int16_t)data_from_read[4] | (int16_t)data_from_read[5] << 8);
        double roll_velocity  = (int16_t)((int16_t)data_from_read[6] | (int16_t)data_from_read[7] << 8);

        yaw_velocity   = (double)yaw_velocity  / 65535. * 180.;
        pitch_velocity = (double)pitch_velocity/ 65535. * 180.;
        roll_velocity  = (double)roll_velocity / 65535. * 180.;

        return std::make_pair(timestamp, cv::Point3d{yaw_velocity, pitch_velocity, roll_velocity});
    }

    std::pair<int, cv::Point3d> WMJRobotControl::getGyroAcce()
    {
        Buffer data_from_read;
        GyroAccePack gyro_acce_data{};
        data_from_read = gyroCanBus->readFrame(GyroAcce);
        std::copy(data_from_read.begin(), data_from_read.end(), gyro_acce_data.data);
        int    timestamp = (int16_t)((int16_t)data_from_read[0] | (int16_t)data_from_read[1] << 8);
        double x_acce    = (int16_t)((int16_t)data_from_read[2] | (int16_t)data_from_read[3] << 8);
        double y_acce    = (int16_t)((int16_t)data_from_read[4] | (int16_t)data_from_read[5] << 8);
        double z_acce    = (int16_t)((int16_t)data_from_read[6] | (int16_t)data_from_read[7] << 8);
        
        x_acce = (double)x_acce / 32768. * 4.;
        y_acce = (double)y_acce / 32768. * 4.;
        z_acce = (double)z_acce / 32768. * 4.;

        return std::make_pair(timestamp, cv::Point3d{x_acce * 9.7944, y_acce * 9.7944, z_acce * 9.7944});
    }

    std::vector<std::pair<int, cv::Point3d>> WMJRobotControl::GetGyroInfo()
    {
        return std::vector<std::pair<int, cv::Point3d>>{this->getGyroAngle(), this->getGyroVelocity(), this->getGyroAcce()};
    }
}
