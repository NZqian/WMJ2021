#pragma once
#include "../../../libBase/include/common.h"

namespace wmj
{
    class Binocular
    {
    private:
        double m_f;                        // 焦距 单位:mm
        double m_camera_dist;              // 相机内距离
        cv::Mat m_camera_matrix_left;      // 左侧内参矩阵
        cv::Mat m_new_camera_matrix_left;  // 矫正后左侧内参矩阵
        cv::Mat m_dist_coeffs_left;        // 左侧畸变矩阵
        cv::Mat m_camera_matrix_right;     // 右侧内参矩阵
        cv::Mat m_new_camera_matrix_right; // 矫正后右侧内参矩阵
        cv::Mat m_dist_coeffs_right;       // 右侧畸变矩阵
        cv::Mat m_rotation;                // 两个相机之间旋转矩阵
        cv::Mat m_translation;             // 两个相机之间平移矩阵
        cv::Mat m_R1, m_R2;                // 左右相机旋转矩阵, P1, P2;
        cv::Mat m_P1, m_P2;                // 左右相机平移矩阵
        cv::Mat m_Q;                       // 重投影矩阵

        void setParam();
        // 将像素坐标转化为图像坐标     flag=0为左目，1为右目
        cv::Point2f pixelToImagePlane(cv::Point2f& input_point, cv::Mat); 
        // 用相机内参将像素点坐标进行矫正并转化为以毫米为单位的图像坐标
        cv::Point2f distortionCorrection(cv::Point2f& input_point, bool flag);
    public:
        Binocular() {}
        ~Binocular() {}
        // 带参构造
        Binocular(double f);
        // 测距
        cv::Point3f getPosition(cv::Point2f point_avg_left, cv::Point2f point_avg_right, bool direction);
        // 测距并求yaw
        std::pair<cv::Point3f, double> getPositionAngle(std::pair<cv::Point2f, cv::Point2f> point_avg_left, std::pair<cv::Point2f, cv::Point2f> point_avg_right);
        // 测距并求三维角
        std::pair<cv::Point3f, cv::Point3d> getPositionAngle(std::vector<cv::Point2f> points_left,std::vector<cv::Point2f> points_right);
    };
}
