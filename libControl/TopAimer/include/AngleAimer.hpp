#pragma once

#include "../../Predict/include/MotionPredict.hpp"
#include "Aimer.hpp"
#include <fcntl.h>
#include <limits>

namespace wmj
{
    class AngleAimer : public Aimer
    {
    public:
        AngleAimer();
        AngleAimer(const std::string &);
        ~AngleAimer(){};

        void buildModel(Armor &);
        Armor getTargetArmor(Armor &);
        GimbalPose getTargetPose(Armor &);

    private:
        void setParam();
        void initData();

        Armor getAbsArmor(Armor &);
        void getPalstance();
        void getHeight();
        void getCircle();

        // 状态量
        // bool m_height_diff;                 // 是否有高低装甲板
        // bool m_radius_diff;                 // 是否有长短半径
        bool m_use_fluent_angle;            // 是否使用平滑角度变化
        bool m_palstance_stable;            // 角速度波动
        bool m_center_stable;               // 旋转中心移动
        int m_rotate_direction;             // 旋转方向，+1 为正方向，-1 为负方向，0 为静止

        // 周期记录
        int m_cycle_cnt;
        int m_cur_pair_num;                 // 当前组编号
        int m_target_pair_num;              // 目标组编号

        // 测量量
        Armor m_abs_last_armor;             // 上一个被识别到的装甲板
        double m_armor_height[2];           // 高低装甲板记录
        double m_radiuses[2];               // 长短半径记录
        cv::Point2d m_approx_center;

        // 玄学参数
        double m_time_off;                  // 预测击打时间偏移量
        double m_abs_angle_off;             // 目标角度绝对偏移，弧度制
        double m_angle_shoot_off;           // 击发窗口相对偏移，弧度制，正方向为当前旋转方向

        // 逻辑参数
        double m_time_switch;               // 甩头所需时间
        double m_angle_diff_shift;          // 装甲板切换阈值，弧度制
        double m_angle_diff_static;         // 装甲板静止阈值，弧度制
        double m_angle_predict_max;         // 最大可预测装甲板角度，弧度制
        double m_angle_follow_shoot_max;    // 跟随模式最大击发窗口角度，弧度制
        double m_angle_focus_shoot_max;     // 专注模式最大击发窗口角度，弧度制
        double m_palstance_min;             // 陀螺最低角速度，弧度制
        double m_palstance_diff_ratio_max;  // 角速度波动幅度阈值
        double m_center_diff_ratio_max;     // 中心漂移和距离比例阈值
        double m_infantry_radius_min;       // 步兵最小半径
        double m_infantry_radius_max;       // 步兵最大半径
        double m_hero_radius_min;           // 英雄最小半径
        double m_hero_radius_max;           // 英雄最大半径

        // 滤波器
        std::shared_ptr<KalmanFilter>   m_abs_angle_filter;
        std::shared_ptr<KalmanFilter>   m_palstance_filter;
        std::shared_ptr<KalmanFilter>   m_height_filter[2];
        std::shared_ptr<KalmanFilter>   m_radius_filter[2];
        std::shared_ptr<MotionPredict>  m_center_predicter;
    };
} // namespace wmj
