#include "../include/MotionPredict.hpp"
#include <opencv2/core/types.hpp>

namespace wmj
{
    /////////////// MotionPredict ///////////////

    /*
     * @breif 绝对坐标预测器构造函数
     */
    MotionPredict::MotionPredict()
    {
        setParam();
        m_KF = std::make_shared<cv::KalmanFilter>(m_state_num, m_measure_num, 0);
        m_measurement = cv::Mat::zeros(m_measure_num, 1, CV_32F);
        cv::setIdentity(m_KF->measurementMatrix);
        cv::setIdentity(m_KF->processNoiseCov, cv::Scalar::all(1e-5));
        cv::setIdentity(m_KF->measurementNoiseCov, cv::Scalar::all(1e-3));
        cv::setIdentity(m_KF->errorCovPost, cv::Scalar::all(1));
        float dt = 1.f / m_control_freq;
        m_KF->transitionMatrix = (cv::Mat_<float>(m_state_num, m_state_num) <<
                                  1, 0, 0, dt, 0, 0, 0.5 * dt * dt, 0, 0,
                                  0, 1, 0, 0, dt, 0, 0, 0.5 * dt * dt, 0,
                                  0, 0, 1, 0, 0, dt, 0, 0, 0.5 * dt * dt,
                                  0, 0, 0, 1, 0, 0, dt, 0, 0,
                                  0, 0, 0, 0, 1, 0, 0, dt, 0,
                                  0, 0, 0, 0, 0, 1, 0, 0, dt,
                                  0, 0, 0, 0, 0, 0, 1, 0, 0,
                                  0, 0, 0, 0, 0, 0, 0, 1, 0,
                                  0, 0, 0, 0, 0, 0, 0, 0, 1);
    }

    /*
     * @breif 预测器参数设置
     */
    void MotionPredict::setParam()
    {
        cv::FileStorage fs(PREDICT_CFG, cv::FileStorage::READ);
        fs["motion"]["debug"]                   >> m_debug;
        fs["motion"]["init_count_threshold"]    >> m_init_count_threshold;
        fs["motion"]["predict_coe"]             >> m_predict_coe;
        fs["motion"]["control_freq"]            >> m_control_freq;
        fs["motion"]["target_change_threshold"] >> m_target_change_threshold;
        fs.release();
    }

    /*
     * @breif 滤波器初始化
     *
     * @param a_position 第一帧的装甲板绝对坐标
     */
    void MotionPredict::initFilter(cv::Point3f a_position)
    {
        m_KF->statePost = (cv::Mat_<float>(m_state_num, 1) <<
                           a_position.x, a_position.y, a_position.z, 0, 0, 0, 0, 0, 0);
        m_KF->predict();
        m_measurement.at<float>(0) = a_position.x;
        m_measurement.at<float>(1) = a_position.y;
        m_measurement.at<float>(2) = a_position.z;

        m_cur_armor_dis = std::sqrt(std::pow(a_position.x, 2) + std::pow(a_position.y, 2) + std::pow(a_position.z, 2));
        m_last_armor_dis = std::sqrt(std::pow(a_position.x, 2) + std::pow(a_position.y, 2) + std::pow(a_position.z, 2));

        for(int i = 0; i < m_init_count_threshold; i++)
        {
            m_KF->correct(m_measurement);
            m_KF->predict();
        }
    }

    /*
     * @breif 根据射速等级设置弹速 调整预测量大小
     *
     * @param a_shoot_level 当前射速等级
     */
    void MotionPredict::setShootSpeed(int a_shoot_level)
    {
        switch(a_shoot_level)
        {
            case 0:
                m_cur_max_shoot_speed = 15.f;
                break;
            case 1:
                m_cur_max_shoot_speed = 15.f;
                break;
            case 2:
                m_cur_max_shoot_speed = 18.f;
                break;
            case 3:
                m_cur_max_shoot_speed = 30.f;
                break;
            default:
                m_cur_max_shoot_speed = 15.f;
                break;
        }
    }

    /*
     * @breif 根据新的绝对坐标测量量预测
     *
     * @param a_position 当前装甲绝对坐标
     *
     * @return 返回预测后的绝对坐标
     */
    cv::Point3f MotionPredict::predict(cv::Point3f a_position)
    {
        cv::Mat prediction;
        m_cur_armor_dis = std::sqrt(std::pow(a_position.x, 2) + std::pow(a_position.y, 2) + std::pow(a_position.z, 2));

        if(!m_initialized)
        {
            std::cout << "Init" << std::endl;
            initFilter(a_position);
            m_last_position = a_position;
            m_initialized = true;
        }

        m_measurement.at<float>(0) = a_position.x;
        m_measurement.at<float>(1) = a_position.y;
        m_measurement.at<float>(2) = a_position.z;
        m_KF->correct(m_measurement);
        prediction = m_KF->predict();
        if(std::sqrt(std::pow(std::fabs(a_position.x - m_last_position.x), 2) + std::pow(std::fabs(a_position.y - m_last_position.y), 2) + std::pow(std::fabs(a_position.z - m_last_position.z), 2)) > m_target_change_threshold )
        {
            initFilter(a_position);
            if(m_debug)
                std::cout << "Target has changed" << std::endl;

        }
        m_last_position = a_position;
        m_last_armor_dis = m_cur_armor_dis;

        return cv::Point3f(prediction.at<float>(0) + m_predict_coe * m_cur_armor_dis / m_cur_max_shoot_speed * prediction.at<float>(3),
                           prediction.at<float>(1) + m_predict_coe * m_cur_armor_dis / m_cur_max_shoot_speed * prediction.at<float>(4),
                           prediction.at<float>(2) + m_predict_coe * m_cur_armor_dis / m_cur_max_shoot_speed * prediction.at<float>(5));
    }


    /////////////// PosePredict///////////////

    /*
     * @breif 位姿预测器构造函数
     */
    PosePredict::PosePredict()
    {
        setParam();
        m_dt = 1.0 / m_freq;
        m_trans =  (cv::Mat_<float>(9, 9) <<
                    1, m_dt, 0.5 * m_dt * m_dt, 0, 0, 0, 0, 0, 0,
                    0, 1, m_dt, 0, 0, 0, 0, 0, 0,
                    0, 0, 1, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 1, m_dt, 0.5 * m_dt * m_dt, 0, 0, 0,
                    0, 0, 0, 0, 1, m_dt, 0, 0, 0,
                    0, 0, 0, 0, 0, 1, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 1, m_dt, 0.5 * m_dt * m_dt,
                    0, 0, 0, 0, 0, 0, 0, 1, m_dt,
                    0, 0, 0, 0, 0, 0, 0, 0, 1);

        m_observe = (cv::Mat_<float>(3, 9) <<
                     1, 0, 0, 0, 0, 0, 0, 0, 0,
                     0, 0, 0, 1, 0, 0, 0, 0, 0,
                     0, 0, 0, 0, 0, 0, 1, 0, 0);

        m_predict_noise = (cv::Mat_<float>(9, 9) <<
                           m_Q_x, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0.01, 0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0.001, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, m_Q_y, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0.01, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0.001, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, m_Q_z, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0.01, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0.001);

        m_measure_noise = (cv::Mat_<float>(3, 3) <<
                           m_R_x, 0, 0,
                           0, m_R_y, 0,
                           0, 0, m_R_z);

        m_p_plus = (cv::Mat_<float>(9, 9) <<
                    1, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0.01, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0.001, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 1, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0.01, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0.001, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 1, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0.01, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0.001);
    }

    /*
     * @breif 预测器参数设置
     */
    void PosePredict::setParam()
    {
        cv::FileStorage fs(PREDICT_CFG, cv::FileStorage::READ);
        fs["pose"]["debug"]          >> m_debug;
        fs["pose"]["freq"]           >> m_freq;
        fs["pose"]["Q_x"]            >> m_Q_x;
        fs["pose"]["Q_y"]            >> m_Q_y;
        fs["pose"]["Q_z"]            >> m_Q_z;
        fs["pose"]["R_x"]            >> m_R_x;
        fs["pose"]["R_y"]            >> m_R_y;
        fs["pose"]["R_z"]            >> m_R_z;
        fs["pose"]["gain_x"]         >> m_gain_x;
        fs["pose"]["gain_y"]         >> m_gain_y;
        fs["pose"]["gain_z"]         >> m_gain_z;
        fs["pose"]["init_count_max"] >> m_init_count_max;

        fs.release();
    }

    /*
     * @breif 根据新的绝对坐标测量量预测
     *
     * @param a_position 当前装甲绝对坐标
     *
     * @return 返回预测后的绝对坐标
     */
    cv::Point3f PosePredict::predict(cv::Point3f a_point)
    {
        if(m_init == false)
        {
            m_x_plus_his = (cv::Mat_<float>(9, 1) << a_point.x, 0, 0, a_point.y, 0, 0, a_point.z, 0, 0);
            m_init = !m_init;
            for(int i = 0; i <= m_init_count_max; i++)
                predict(a_point);
        }

        // 更新步
        cv::Mat x_minus = m_trans * m_x_plus_his;
        cv::Mat p_minus = m_trans * m_p_plus * m_trans.t() + m_predict_noise;

        // 预测步
        cv::Mat K = (p_minus * m_observe.t()) * (m_observe * p_minus * m_observe.t() + m_measure_noise);
        cv::Mat y = (cv::Mat_<float>(3, 1) << a_point.x, a_point.y, a_point.z);
        cv::Mat x_plus = x_minus + K * (y - m_observe * x_minus);
        m_p_plus = (cv::Mat::eye(9, 9, CV_32F) - K * m_observe) * p_minus;

        m_x_plus_his = x_plus;

        cv::Point3f ret_point = cv::Point3f(x_plus.at<float>(0, 0) * m_gain_x + x_plus.at<float>(1, 0) * m_gain_x / 2,
                                            x_plus.at<float>(3, 0) * m_gain_y + x_plus.at<float>(4, 0) * m_gain_y / 2,
                                            x_plus.at<float>(6, 0) * m_gain_z + x_plus.at<float>(7, 0) * m_gain_z / 2);

        return ret_point;
    }

    /*
     * @breif 根据新的云台位姿测量量预测
     *
     * @param a_pose 当前云台位姿
     *
     * @return 返回预测后的云台位姿
     */
    wmj::GimbalPose PosePredict::predict(wmj::GimbalPose a_pose)
    {
        if(m_init == false)
        {
            m_x_plus_his = (cv::Mat_<float>(9, 1) << a_pose.pitch, 0, 0, a_pose.yaw, 0, 0, a_pose.roll, 0, 0);
            m_init = !m_init;
            for(int i = 0; i <= m_init_count_max; i++)
                predict(a_pose);
        }

        // 更新步
        cv::Mat x_minus = m_trans * m_x_plus_his;
        cv::Mat p_minus = m_trans * m_p_plus * m_trans.t() + m_predict_noise;

        // 预测步
        cv::Mat K = (p_minus * m_observe.t()) * (m_observe * p_minus * m_observe.t() + m_measure_noise).inv();
        cv::Mat y = (cv::Mat_<float>(3, 1) << a_pose.pitch, a_pose.yaw, a_pose.roll);
        cv::Mat x_plus = x_minus + K * (y - m_observe * x_minus);
        m_p_plus = (cv::Mat::eye(9, 9, CV_32F) - K * m_observe) * p_minus;

        m_x_plus_his = x_plus;

        wmj::GimbalPose ret_pose;
        ret_pose = wmj::GimbalPose(x_plus.at<float>(0, 0) + x_plus.at<float>(1, 0) * m_gain_x + x_plus.at<float>(2, 0) * 1,
                                   x_plus.at<float>(3, 0) + x_plus.at<float>(4, 0) * m_gain_y + x_plus.at<float>(5, 0) * 1,
                                   x_plus.at<float>(6, 0) + x_plus.at<float>(7, 0) * m_gain_z + x_plus.at<float>(8, 0) * 1);

        if(m_debug)
        {
            std::cout << "sub " << ret_pose.yaw - a_pose.yaw << "\n";
        }

        // 防止超调
        if(std::fabs(a_pose.yaw - ret_pose.yaw) > 0.12)
        {
            ret_pose.yaw = a_pose.yaw + 0.7 * (ret_pose.yaw - a_pose.yaw);
        }

        // pitch发位置 不给预测
        ret_pose.pitch = a_pose.pitch;

        return ret_pose;
    }
    /////////////// AngleFilter ///////////////

    /*
     * @breif 单量滤波器器构造函数
     */
    KalmanFilter::KalmanFilter(double a_process_noise, double a_measure_noise, double a_val_diff)
    {
        setParam();
        m_KF = std::make_shared<cv::KalmanFilter>(m_state_num, m_measure_num, 0);
        m_measurement = cv::Mat::zeros(m_measure_num, 1, CV_32F);
        cv::setIdentity(m_KF->measurementMatrix);
        cv::setIdentity(m_KF->processNoiseCov, cv::Scalar::all(a_process_noise));
        cv::setIdentity(m_KF->measurementNoiseCov, cv::Scalar::all(a_measure_noise));
        cv::setIdentity(m_KF->errorCovPost, cv::Scalar::all(1));
        float dt = 1.f / m_control_freq;
        m_KF->transitionMatrix = (cv::Mat_<float>(m_state_num, m_state_num) <<
                                  1, dt,
                                  0, 1);
        m_val_diff = a_val_diff;
    }

    /*
     * @breif 预测器参数设置
     */
    void KalmanFilter::setParam()
    {
        cv::FileStorage fs(PREDICT_CFG, cv::FileStorage::READ);
        fs["KF"]["debug"]                >> m_debug;
        fs["KF"]["init_count_threshold"] >> m_init_count_threshold;
        fs["KF"]["predict_coe"]          >> m_predict_coe;
        fs["KF"]["control_freq"]         >> m_control_freq;
        fs.release();
    }

    /*
     * @breif 滤波器初始化
     *
     * @param a_val 第一帧的值
     */
    void KalmanFilter::initFilter(double a_val)
    {
        m_KF->statePost = (cv::Mat_<float>(m_state_num, 1) << a_val, 0);
        m_KF->predict();
        m_measurement.at<float>(0) = a_val;

        for(int i = 0; i < m_init_count_threshold; i++)
        {
            m_KF->correct(m_measurement);
            m_KF->predict();
        }
        std::cout << "Init" << std::endl;
    }

    /*
     * @breif 根据新的测量量滤波
     *
     * @param a_val 当前测量量
     *
     * @return 返回滤波后的量
     */
    double KalmanFilter::predict(double a_val)
    {
        cv::Mat result;

        if(!m_initialized)
        {
            initFilter(a_val);
            m_last_val = a_val;
            m_initialized = true;
        }

        m_measurement.at<float>(0) = a_val;
        m_KF->correct(m_measurement);
        result = m_KF->predict();
        if(std::fabs(a_val - m_last_val) > m_val_diff)
        {
            initFilter(a_val);
            if(m_debug)
                std::cout << "Target has changed" << std::endl;
        }
        m_last_val = a_val;

        return 1.0 * result.at<float>(0) + 0.0 * result.at<float>(1);
    }
}
