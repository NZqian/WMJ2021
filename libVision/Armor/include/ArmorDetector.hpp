#pragma once
#include "ArmorSingle.hpp"
#include "../include/ArmorDebug.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>

namespace wmj
{
    class Monocular
    {
    public:
        double m_small_width;
        double m_small_height;
        double m_large_width;
        double m_large_height;
    };
    enum State
    {
        Debug = 0,
        Single = 1,
        Double = 2,
        DoubleDebug = 3,
        SentryAttack = 4,
        SentryUse = 5
    }; //各种状态
    class ArmorDetectorDouble
    {
    public:
        State m_state = Double;                  //模式选择，这里默认使用double,需要外部设置
        int i_state = 2;                         //int形模式
        std::vector<ArmorSingle> m_ArmorSingles; // 单目识别类
        wmj::Binocular m_binocular;              // 测距类
        cv::Point3f m_repairPoint;               // 修正坐标
        double m_time_left;
        double m_time_right;
        std::vector<double> m_times;
        std::vector<Armor> m_armors;
        Monocular m_monocular;
        int m_thread_number; // 选择的进程数，注意到这里和读进来的相机数目不一定一样
        int m_singleChoose = 0;

        int m_max_single_distance;    //单目识别的极限距离
        int m_lost_num[2] = {10, 10}; //左右目BUFFER大小
    public:
        ArmorDetectorDouble();

        void init();
        /* 识别主程序,无论单目镜双目都在这里 */
        Armor DetectArmorDouble(std::vector<wmj::MatWithTime> a_frames, std::vector<Armor> &a_armors);
        /*#############################*/

        //选择最终的结果
        void chooseTargetArmor(std::vector<Armor> &a_armors);

        void chooseArmorDouble(std::vector<Armor> &a_armors);
        //单目匹配结果
        void chooseTargetArmorSingle(std::vector<Armor> &a_armors, int a_choose);
        //双目匹配结果
        void chooseTargetArmorDouble(std::vector<Armor> &a_armors);

        //输出组后目标装甲，包括坐标距离
        Armor measureArmorDistance(std::vector<Armor> &a_armors);
        //单双线程的开启和关闭
        void detectArmor(std::vector<wmj::MatWithTime> a_frames);
        //双目匹配函数
        bool doubleCompare(Armor a, Armor b);
        /*单目识别*/
        void SingleDetect(int a_choose, cv::Mat &src);

        //比率比对，比较两个值是不是符合一个比率 范围
        double compareRate(double a, double b);

        //选择处理的数量
        void chooseThreadNumber();
        /* 获取装甲板position位置 */
        cv::Point3f calcArmorDeepth(const Armor &armor);

        /* 获取具体位置 */
        void GetObjPoints(float _width, float _height,
                          std::vector<cv::Point3f> &op_v);
        //获取单目参数
        void readCameraSetting(const cv::FileStorage &fs);

        // Double 判定的参数倒入
        void setParamDouble(const cv::FileStorage &fs);
    };

} // namespace wmj
