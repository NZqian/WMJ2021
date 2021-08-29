/**
 * @file    ArmorTrigger.hpp
 * @author  liangyufeng@mail.nwpu.edu.cn
 * @version V1.0.0
 * @date    2-August-2021
 * @brief   This file contains all the functions for the energy detection library.
 */
#pragma once

#include <opencv2/opencv.hpp>
#include <iostream>
#include "../../../libBase/include/common.h"
#include "../../Bino/include/Binocular.h"

using namespace std;
using namespace cv;

namespace wmj
{
    //扇叶类型
    typedef enum
    {
        UNKOWN,   //未知类型
        INACTION, //未激活
        ACTION    //已激活
    } Type;

    /** 
     * @brief 扇叶类
     */
    class Vane
    {
    public:
        Vane() {}
        cv::RotatedRect m_small_rect; //装甲板矩形
        cv::RotatedRect m_big_rect;   //扇叶矩形
        cv::Mat m_armor_img;          //装甲板图像
        int m_type = UNKOWN;          //扇叶类型
        float m_angle;                //扇叶角度
        double m_time;                //时间戳(s)
        double m_rate;                //扇叶亮度
    };

    /** 
     * @brief 检测目标装甲板的单目类
     */
    class RuneDetector
    {
    public:
        RuneDetector() {}

        /**
         * @brief 图像处理
         * @param src_img 原始输入图像
         */
        void imgProcess(cv::Mat &src_img);

        /**
         * @brief 寻找图像中的一系列坐标参数
         * @param frame 输入待识别图像向量
         * @param state 能量机关状态 (0:静止 1:匀速 2:正弦)
         * @param center_point 圆心R的二维坐标
         * @param armor_point 待击打装甲板的二维坐标
         * @param predict_angle 计算得到的预测角度
         */
        void findTargetPoint(wmj::MatWithTime &frame, int state, cv::Point2f &center_point, cv::Point2f &armor_point, float &predict_angle);

        /**
         * @brief 读取能量机关参数配置文件
         * @param file_path 配置文件路径
         */
        void setRuneParam(const string &file_path);

        /**
         * @brief 读取相机参数配置文件
         * @param file_path 配置文件路径
         * @param location 相机位置，0:左目，1:右目
         */
        void setCameraParam(const string &file_path, bool location);

        /**
         * @brief 计算能量机关旋转方向
         * @param angle 输入扇叶角度
         */
        void getRotatedDirection(float angle);

        /**
         * @brief 计算能量机关的预测角度
         */
        void predict();

        /**
         * @brief 截取ROI区域
         * @param vane 输入扇叶类
         */
        void getImageROI(Vane &vane);

        /**
         * @brief 计算旋转速度与函数拟合
         * @param vane 输入扇叶类
         */
        void getRotatedSpeed(Vane &vane);

        /**
         * @brief 寻找能量机关中心坐标
         */
        void findCenter();

        /**
         * @brief 切割能量机关装甲板图像
         * @param rect 待切割的矩形
         * @return 装甲板图像
         */
        cv::Mat armorCut(RotatedRect &rect);

        /**
         * @brief 计算所输入图像的亮度比例
         * @param src 输入图像
         * @return 图像亮度比例
         */
        double judge(cv::Mat &src);

        /**
         * @brief 计算装甲板中心的世界坐标位置(单目测距,仅作参考)
         * @return 装甲板中心的三维坐标
         */
        cv::Point3f getAbsolutePosition();

        /**
         * @brief 计算所输入两点的距离
         * @param point_a a点的坐标
         * @param point_b b点的坐标
         * @return 两点间的距离
         */
        double getDistance(cv::Point2f &point_a, cv::Point2f &point_b);

        /**
         * @brief 判断输入的矩形区域是否超出边界，是否满足裁剪要求
         * @param rect 待判断的矩形
         * @return 1:符合要求 0:不符合要求
         */
        bool isResectable(RotatedRect &rect);

        /**
         * @brief 调节输入角度使其范围在[0,360)
         * @param angle 待规范的角度
         * @return 规范后的角度
         */
        float makeAngleRegular(float angle);

        /**
         * @brief 计算扇叶的角度
         * @param vane 扇叶类
         */
        void calcAngle(Vane &vane);

        // 图像处理过程
        cv::Mat m_src_img, m_result_img;
        cv::Mat m_binary_img, m_binary_color_img;

        bool m_debug;          //debug标志位
        bool m_rune_color;     //颜色
        float m_predict_angle; //预测角度
        double m_predict_time; //预测时间
        string m_index;        //显示左或右目图像

        int m_rot_direction = 0; //旋转方向
        int m_rotation_state;    //旋转状态(0:静止 1:匀速 2:正弦)

        //roi矩形
        cv::Rect m_img_rect;
        cv::Rect m_roi_rect, m_roi_rect_last;

        Vane m_final_target;  //最终得到的未激活扇叶
        cv::Point2f m_center; //圆心的二维坐标

        cv::Point2f m_armor_point;  //装甲板中心坐标
        float m_armor_width = 0.24; //装甲板物理宽度

        bool m_isDetected = false;        //是否识别到目标装甲板
        bool m_center_isDetected = false; //是否识别到圆心
        bool m_sin_flag = false;          //正弦拟合是否成功

    private:
        //相机参数
        cv::Mat m_camera_matrix, m_distortion_coefficients;

        //各种阈值
        int m_threshold_color, m_threshold_gray;
        float m_small_contour_area_min, m_small_contour_area_max;
        float m_big_contour_area_min, m_big_contour_area_max;
        float m_center_area_min, m_center_area_max;

        //存放轮廓信息
        vector<vector<Point>> m_contours;
        vector<Vec4i> m_hierarchy;

        double m_time;  //图像的时间戳(s)
        string m_lable; //图像的标签 "left" or "right"

        //正弦拟合采集的数据容器与相关参数
        vector<Vane> m_vanes;
        vector<Vane> m_corrects;
        vector<float> m_angles;
        int m_sample_size = 40;
        int m_count = 0;
        bool m_sin_false = false;
        float m_angle_fit;                                    //根据函数计算得到的扇叶角度
        double m_time_param, m_function_param, m_angle_param; //正弦转动函数参数
    };

    /**
     * @brief 识别目标装甲板的类
     */
    class ArmorTrigger
    {
    public:
        /**
         * @brief 构造函数,读取配置文件内容
         */
        ArmorTrigger();

        /**
         * @brief 寻找待击打装甲板入口函数
         * @param frame 图像向量
         * @param state 能量机关状态(0:静止 1:匀速 2:正弦)
         * @return 装甲板的三维坐标(未预测),用于判断是否识别到目标
         */
        cv::Point3f findTarget(std::vector<MatWithTime> &frame, int state);

        /**
         * @brief 左目识别线程函数
         */
        void threadLeft();

        /**
         * @brief 右目识别线程函数
         */
        void threadRight();

        /**
         * @brief 设置预测提前量
         * @param predict_time 预测量
         */
        void setPredictTime(double predict_time);

        /**
         * @brief 设置预测提前量并重新计算预测角度
         * @param predict_time 预测量
         */
        void recomputePredictAngle(double predict_time);

        /**
         * @brief 选择显示左右目图像
         * @param index "left" or "right"
         */
        void setImshowIndex(string index);

        /**
         * @brief 设置要识别的颜色
         * @param color 0:对方颜色 1:己方颜色
         */
        void setRuneColor(bool color);

    public:
        bool m_auto_shoot = false; //自动击发标志位
        double m_pitch_off;        //弹道补偿

        float m_left_predict_angle, m_right_predict_angle; //预测角度
        cv::Point3f m_armor_position, m_center_position;   //装甲板和圆心的三维坐标

    private:
        wmj::RuneDetector m_rune_detector_left, m_rune_detector_right; //两个单目识别类
        cv::Point2f m_left_armor_point, m_right_armor_point;           //装甲板坐标
        cv::Point2f m_left_center_point, m_right_center_point;         //圆心坐标

        std::vector<MatWithTime> m_frame; //图像
        wmj::Binocular m_binocular;       //双目测距

        int m_state;       //能量机关状态(0:静止 1:匀速 2:正弦)
        bool m_rune_color; //颜色
    };
} // namespace wmj