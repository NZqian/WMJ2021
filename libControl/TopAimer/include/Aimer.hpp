#pragma once
#include <vector>
#include <memory>
#include <cmath>
#include <utility>
#include <opencv2/opencv.hpp>

#include "../../../libBase/include/common.h"
#include "../../../libVision/Armor/include/ArmorDetector.hpp"
#include "../../../libControl/Pose/include/AngleSolver.hpp"

namespace wmj
{
    class Ransac
    {
    public:
        Ransac();
        ~Ransac() {};

        std::pair<double, cv::Point2d> fitCircle(std::vector<Armor> &);

    private:
        void setParam();
        void initData();
        // std::pair<double, cv::Point2d> getCircle(Armor &, Armor &);
        std::pair<double, cv::Point2d> getCircle(Armor &, Armor &, Armor &);
        float verifyCircle(double, cv::Point2d, std::vector<Armor> &);

        bool m_debug;

        // 迭代参数
        int m_iteration_cnt;                                // 当前迭代次数
        int m_cur_iteration;                                // 当前需迭代次数
        int m_iteration_max;                                // 最大迭代次数
        double m_model_confidence;                          // 置信度

        // 模型参数
        cv::RNG m_rng;                                      // 随机数发生器
        int m_point_need;                                   // 取一个圆所需目标点数，2 或 3
        int m_vec_size;                                     // 目标点数量
        double m_radius_min;
        double m_radius_max;
        double m_inlier_dist_ratio;                         // 内点与圆的距离和半径的比值
        double m_inlier_dist_min;                           // 内点距离阈值下限
        double m_inlier_dist_max;                           // 内殿距离阈值上限
        
        // 最佳结果记录
        float       m_bset_inlier_proportion;               // 最佳内点占比
        double      m_best_radius;                          // 最佳拟合半径
        cv::Point2d m_best_center;                          // 最佳拟合圆心
    };

    class Aimer
    {
    public:
        Aimer();
        ~Aimer() {};

        Armor getFinalArmor(GimbalPose &, Armor &);
        Armor getFinalArmor(GimbalPose &, Armor &, float);
        GimbalPose getFinalPose(GimbalPose &, Armor &);
        GimbalPose getFinalPose(GimbalPose &, Armor &, float);

        enum TrackMode
        {
            FOLLOW = 0,
            FOCUS  = 1
        };
        void setTrackMode(TrackMode);
        void setBulletSpeed(float);
        bool shootable();

        bool m_shoot_flag;

    private:
        void setParam();

    protected:
        virtual void buildModel(Armor &) = 0;
        virtual Armor getTargetArmor(Armor &) = 0;
        virtual GimbalPose getTargetPose(Armor &) = 0;

        // DEBUG
        bool m_debug;
        bool m_write_file;
        int m_file;

        cv::Point3f m_center;
        GimbalPose m_cur_pose;

        // 控制参数
        bool m_model_ready;
        int m_lost_cnt;
        int m_min_vec_size;                                 // 最小建模尺寸
        double m_time_delay;                                // 控制延迟
        TrackMode m_track_mode;                             // 跟随模式

        // 测量量
        float m_bullet_speed;
        double m_palstance;                                 // 角速度，弧度制
        double m_radius;                                    // 装甲板旋转半径

        // 记录量
        std::vector<wmj::Armor>         m_armor_seq;
        std::vector<wmj::Armor>         m_abs_armor_seq;
        std::vector<wmj::GimbalPose>    m_pose_seq;         // armor角度为相对值，须对应保留GimbalPose

        std::shared_ptr<wmj::AngleSolver>   m_angle_solver;
        std::shared_ptr<wmj::Ransac>        m_ransac_solver;

    public:
        double getDistance(cv::Point3f p)
        {
            return sqrt(p.x * p.x + p.y * p.y);
        }
        double getDistance(cv::Point2d p)
        {
            return sqrt(p.x * p.x + p.y * p.y);
        }

    };
} //namespace wmj
