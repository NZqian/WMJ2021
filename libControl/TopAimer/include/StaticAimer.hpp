#pragma once

#include "Aimer.hpp"

namespace wmj
{
    class StaticAimer : public Aimer
    {
    public:
        StaticAimer();
        ~StaticAimer() {};

    private:
        void setParam();
        void buildModel(Armor&);
        GimbalPose getTargetPose(Armor&);

        // std::pair<double, cv::Point3f> m_pos_with_time[2];
        Armor m_abs_armor;
        Armor m_armor_target[2];
        Armor m_armor_cur_target;           // 最近装甲板

        GimbalPose m_pose_target[2];
        GimbalPose m_pose_cur_target;       // 最近装甲板所对应位姿

        int m_armor_no = 0;
        
        double m_horizontal_diff_max;
        double m_horizontal_diff_min;
        double m_min_dist;
        double m_time_diff;                 // 旋转90°用时
        double m_angle_predict;             // 偏离正对位置的角度，角度制
    };
} // namespace wmj
