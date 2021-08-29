#pragma once
#include "../../../libBase/include/common.h"
#include <opencv2/core/types.hpp>

namespace wmj
{
    /**
     * @brief 三维装甲绝对坐标预测
     */
    class MotionPredict
    {
    public:
        MotionPredict();
        ~MotionPredict() {}
        void setParam();
        cv::Point3f predict(cv::Point3f);
        void setShootSpeed(int); // 设置射速等级 根据射速调整预测量

    private:
        bool        m_initialized{false};
        int         m_debug;
        int         m_measure_num = 3;
        int         m_state_num   = 9;
        int         m_lost_count;
        int         m_init_count_threshold;
        float       m_control_freq;
        float       m_predict_coe;
        float       m_cur_max_shoot_speed;
        float       m_cur_armor_dis;
        float       m_last_armor_dis;
        float       m_target_change_threshold;
        cv::Mat     m_measurement;
        cv::Point3f m_last_position;
        std::shared_ptr<cv::KalmanFilter> m_KF;

        void initFilter(cv::Point3f);
    };


    /**
     * @brief 二维云台位姿滤波器
     */
    class PosePredict
    {
    public:
        PosePredict();
        ~PosePredict() {}

        void setParam();
        cv::Point3f predict(cv::Point3f);
        wmj::GimbalPose predict(wmj::GimbalPose);

    private:
        int m_debug;
        int m_freq;

        float m_dt;
        float m_Q_x;
        float m_Q_y;
        float m_Q_z;
        float m_R_x;
        float m_R_y;
        float m_R_z;
        float m_gain_x;
        float m_gain_y;
        float m_gain_z;

        bool m_init = false;
        int m_init_count_max;
        cv::Mat m_x_plus_his;

        cv::Mat m_trans;         // F
        cv::Mat m_observe;       // H
        cv::Mat m_predict_noise; // Q
        cv::Mat m_measure_noise; // R
        cv::Mat m_p_plus;        // P

        std::deque<cv::Point3f> m_point_deque;
        std::deque<wmj::GimbalPose> m_pose_deque;
    };

    /**
     * @brief 陀螺自瞄专用一维滤波器
     */
    class KalmanFilter
    {
    public:
        KalmanFilter(double, double, double);
        ~KalmanFilter() {}
        void setParam();
        double predict(double);

    private:
        bool    m_initialized{false};
        int     m_debug;
        int     m_measure_num = 1;
        int     m_state_num   = 2;
        int     m_lost_count;
        int     m_init_count_threshold;
        float   m_control_freq;
        float   m_predict_coe;
        // double  m_process_noise;
        // double  m_measure_noise;
        double  m_last_val;
        double  m_val_diff;
        cv::Mat m_measurement;
        std::shared_ptr<cv::KalmanFilter> m_KF;

        void initFilter(double);
    };
}
