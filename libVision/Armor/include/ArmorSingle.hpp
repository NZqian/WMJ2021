#pragma once
#include "../../../libBase/include/common.h"
#include "../../Bino/include/Binocular.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <vector>

namespace wmj
{

    /* 颜色定义 */
    enum _COLOR
    {
        _RED = 0,
        _BLUE = 1,
        _WHITE = 2
    };
    enum ARMORTYPE
    {
        ARMOR_NONE = 0,
        ARMOR_SMALL = 1,
        ARMOR_LARGE = 2
    };
    enum DETECTEDTYPE
    {
        DETECT_LEFT = 0,
        DETECT_RIGHT = 1,
        DETECT_DOUBLE = 2
    };
    enum FINDTYPE
    {
        NOTFIND = 0,
        FIRST = 1,
        SHOOT = 2
    };
    enum STATE
    {
        SEARCHING = 0,
        TRACKING = 2,
        BUFFER = 1
    };
    /*装甲配置文件导入*/

    class ArmorParam
    {
    public:
        // 图像预处理参数
        double m_roi_value; // roi的放大系数
        int m_min_thresh_red;
        int m_min_thresh_blue;
        int m_color_thresh;

        // 灯条几何筛选参数
        float m_light_max_area;
        float m_light_min_area;
        float m_light_min_ratio;
        float m_light_max_ratio;
        float m_light_min_angle;
        float m_light_max_angle;
        float m_light_area_ratio;
        float m_min_RBdiff;
        float m_min_BRdiff;

        // 装甲板几何筛选参数
        float m_armor_tiltangle;
        float m_armor_malposition;
        float m_armor_angle_diff;
        float m_armor_ratio_max;
        float m_armor_ratio_min;
        float m_lights_diff;
        float m_armor_min_area;
        float m_angle_diff;

        //敌方颜色
        int m_enemy_color;

        // 无数字识别 灯条几何参数
        float m_dimlight_max_area;
        float m_dimlight_min_area;
        float m_dimlight_min_ratio;
        float m_dimlight_max_ratio;
        float m_dimlight_min_angle;
        float m_dimlight_max_angle;
        float m_dimlight_area_ratio;
        float m_dim_min_RBdiff;
        float m_dim_min_BRdiff;

        // 无数字识别 装甲板几何参数
        float m_dim_armor_tiltangle;
        float m_dim_armor_malposition;
        float m_dim_armor_angle_diff;
        float m_dim_armor_ratio_max;
        float m_dim_armor_ratio_min;
        float m_dim_lights_diff;
        float m_dim_angle_diff;
        float m_dim_armor_min_area;

    public:
        ArmorParam(){};
        void setParam(const cv::FileStorage &fs);
    };

    /* 灯条定义 */
    class Light
    {
    public:
        cv::RotatedRect m_rect;           //灯条外接矩形
        cv::Point2f m_center;             //灯条中心
        cv::Rect2d m_rectR;               // 灯条正接矩形
        int m_color;                      // 灯条颜色
        double m_ratio;                   //长宽比
        double m_length;                  //灯条长度
        double m_width;                   //灯条宽度
        double m_area;                    //灯条面积
        double m_area_ratio;              //轮廓面积和最小外接矩形面积之比
        double m_angle;                   //灯条角度
        std::vector<cv::Point> m_contour; //灯条轮廓点集
    public:
        Light(){};
        Light(const std::vector<cv::Point> contour, const cv::Point2d &base); //带参构造函数
        bool isLight(ArmorParam _param, bool dimOrlight);                     //灯条几何条件筛选
        void regularRect(cv::RotatedRect &rect);                              //外接矩形矫正
    };
    typedef std::vector<Light> Lights;

    /* 装甲板定义 */
    class Armor
    {
    public:
        cv::Rect2d m_rect;                    //装甲板外接矩形
        cv::RotatedRect m_lightRect;          //灯条构成的矩形
        _COLOR m_color;                       //装甲板颜色
        Lights m_pairs;                       //灯条对
        std::vector<cv::Point2f> m_vertices;  //单目测距用角点
        int m_id;                             //装甲id
        cv::Point3f m_position;               //三维坐标信息
        cv::Point3f m_angle;                  //三维角度坐标，等待运算
        cv::Point2f m_center;                 //中心位置，为了双目识别的
        double m_time_seq;                    //时间戳
        ARMORTYPE m_armor_type = ARMOR_NONE;        //大小装甲
        DETECTEDTYPE m_detectedtype;          //判断是双目识别还是单目识别
        FINDTYPE m_bestArmorStatus = NOTFIND; //判断识别结果
        float m_ratio;                        //装甲长宽比
        double m_yaw_angle;                    //按yaw轴旋转的角度
        float m_area;                         //装甲面积
        float m_width;                        //装甲横向长度
        float m_height;                       //装甲纵向长度
        float m_tiltAngle;                    //装甲roll旋转角度
        float m_lighsRatio;                   //装甲板两个灯条的长度比
        float m_angle_diff;                   //装甲板两个灯条的角度差
        double m_socre;                       //装甲板优先度

    public:
        Armor()
        {
            m_pairs.resize(2);
        };
        Armor(const Light &left, const Light &right, cv::Point2f targetPoint); //带参构造
        bool IsArmor(ArmorParam _param, bool dimOrLight);                      //装甲板几何参数
        bool operator>(const Armor &armor) const;                              //装甲板排序
    };
    typedef std::vector<Armor> Armors;

    /* hog-svm定义 */
    class HOG_SVM
    {
    private:
        cv::Ptr<cv::ml::SVM> m_svm;
        std::map<int, int> m_label2id;
        cv::HOGDescriptor m_hog;

    public:
        HOG_SVM();
        int test(const cv::Mat &src);
    };

    /* 识别类定义 */
    class ArmorSingle
    {
    public:
        ArmorParam m_param;             //参数类
        cv::Mat m_src;                  //图像
        cv::Mat m_binary;               //颜色二值图
        cv::Mat m_binary_gray;          //灰度二值图
        std::vector<cv::Mat> m_numbers; //存储用于数字识别的图片
        cv::Mat m_gray;                 //灰度图
        HOG_SVM m_svm;                  //数字识别类
        Armor m_last_armor;             //上一次识别的装甲
        cv::Rect2d m_roi;               // ROI

        STATE m_state; //识别模式

        //相机内参导入
        cv::Mat m_CameraMat;
        cv::Mat m_DistMat;

        Armors m_armors;   //识别到的装甲板
        Lights m_lights;   //识别到的灯条
        Armor m_bestArmor; //找到的最优装甲板

        int m_detectnum = -1;  //用于锁定装甲板ID
        int m_roi_value = 5;   //ROI计算比例
        int m_endebug = 0;     //开启debug模式的标志位
        bool m_complete;       //主函数完成的标志位
        int m_decide;          //左右相机选择的标志位
        bool useNumber = true; //是否使用数字识别

    public:
        /* 识别主程序 */
        Armors ArmorSingleDetector();
        /* 获取yaml文件配置*/
        void readSetting(cv::FileStorage &fs);
        //开启双目debug的函数
        void openDoubledebug(int m_choose);

        // roi封装
        void roiChoose();

        ArmorSingle();

        //这里仅作数字的筛选
        int SetArmorId(Armor &armor);
        //初始化
        void init();
        //debug
        void Debug();

    private:
        /* 图像处理 */
        void preProcess();

        /* 查找灯条 */
        bool findLights();

        /* 查找装甲 */
        bool findArmors();
        //找到一个最好的装甲板
        bool findTargetArmor();
        //删除重复装甲板
        bool needToDelate(Armor &armor, Armor &b_armor);

        /* 判断是否为假装甲 */
        bool IsFakeArmor(int i, int j);

        /* 获取灯条颜色 */
        bool judgeColor(Light &light, std::vector<cv::Point> Contours);

    public:
    };

}
