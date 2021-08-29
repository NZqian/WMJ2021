#include "../include/ArmorDetector.hpp"
#include <thread>
using namespace std;
using namespace cv;
using namespace cv::ml;
#define LOSE_NUM 2
namespace wmj
{

    /* ##############################################

                    初始化全部

    ############################################## */
    ArmorDetectorDouble::ArmorDetectorDouble()
    {
        m_binocular = wmj::Binocular(6);
        m_ArmorSingles.resize(2); //初始化设定数量
        FileStorage fs("../libVision/Armor/ArmorDetector.yaml", cv::FileStorage::READ);
        FileStorage cfg("../libVision/Bino/Binocular.yaml", cv::FileStorage::READ);
        m_repairPoint.x = 0;
        m_repairPoint.y = 222.6;
        m_repairPoint.z = 0;
        setParamDouble(fs);
        readCameraSetting(cfg);
        i_state = 2;
    }
    void ArmorDetectorDouble::setParamDouble(const cv::FileStorage &fs)
    {
        fs["ArmorDetector"]["singleChoose"] >> m_singleChoose;
        fs["ArmorDetector"]["armor_small_width"] >> m_monocular.m_small_width;
        fs["ArmorDetector"]["armor_small_height"] >> m_monocular.m_small_height;
        fs["ArmorDetector"]["armor_large_width"] >> m_monocular.m_large_width;
        fs["ArmorDetector"]["armor_large_height"] >> m_monocular.m_large_height;
        fs["ArmorDetector"]["max_single_distance"] >> m_max_single_distance;
    }
    void ArmorDetectorDouble::readCameraSetting(const cv::FileStorage &fs)
    {
        fs["M1"] >> m_ArmorSingles[0].m_CameraMat;
        fs["D1"] >> m_ArmorSingles[0].m_DistMat;
        fs["M2"] >> m_ArmorSingles[1].m_CameraMat;
        fs["D2"] >> m_ArmorSingles[1].m_DistMat;
    }
    /*######################################################

                多线程运行主程序
                找到所有装甲板

    ######################################################*/
    void ArmorDetectorDouble::detectArmor(std::vector<wmj::MatWithTime> frames)
    {
        chooseThreadNumber();
        if (m_thread_number == 1)
        {
            if (frames.size() == 1)
            {
                SingleDetect(m_singleChoose, frames[0].m_img);
            }
            else
            {
                SingleDetect(m_singleChoose, frames[m_singleChoose].m_img);
            }
        }
        else
        {
            if (frames.size() == 2)
            {

                if (m_state == State::SentryAttack)
                {
                    m_ArmorSingles[0].useNumber = false;
                    m_ArmorSingles[1].useNumber = false;
                }
                std::thread threadRight(bind(&ArmorDetectorDouble::SingleDetect, this, 1, frames[1].m_img));
                //加入子线程，并行计算加快速度
                SingleDetect(0, frames[0].m_img);
                if (m_ArmorSingles[0].m_complete == true)
                {
                    threadRight.join();
                }
            }
            else
            {
                cout << "双目模式只有一组图像" << endl;
                exit(0);
            }
            if (m_state == DoubleDebug)
            {
                cout << _blue("leftSize:\t") << m_ArmorSingles[0].m_armors.size() << endl;
                cout << _blue("rightSize:\t") << m_ArmorSingles[1].m_armors.size() << endl;
                m_ArmorSingles[0].openDoubledebug(0);
                m_ArmorSingles[1].openDoubledebug(1);
            }
        }
    }

    void ArmorDetectorDouble::SingleDetect(int choose, Mat &src)
    {
        if (m_state == State::Debug)
        {
            m_ArmorSingles[choose].m_endebug = choose + 1;
        }
        else
        {
            m_ArmorSingles[choose].m_endebug = 0;
        }

        m_ArmorSingles[choose].m_src = src;
        m_ArmorSingles[choose].m_decide = choose;
        m_ArmorSingles[choose].ArmorSingleDetector();

        m_ArmorSingles[choose].m_decide = choose;
    }
    void ArmorDetectorDouble::chooseThreadNumber()
    {
        switch (m_state)
        {
        case Debug:

        case Single:
            m_thread_number = 1;
            break;
        case Double:

        case SentryAttack:

        case DoubleDebug:
            m_thread_number = 2;
            break;
        default:
            m_thread_number = 1;
            break;
        }
    }

    /*###################################################

            选择目标装甲板，并设置上一帧装甲板

    ##################################################*/
    // 根据识别模式进行选择
    void ArmorDetectorDouble::chooseTargetArmor(std::vector<Armor> &armors)
    {
        switch (m_state)
        {
        case Single:
            //直接调用但目程序，给armors赋值
            chooseTargetArmorSingle(armors, m_singleChoose);
            break;
        case Debug:
            //直接调用但目程序，给armors赋值
            chooseTargetArmorSingle(armors, m_singleChoose);
            break;
        case Double:
            //这时候就得用双目程序了
            chooseArmorDouble(armors);
            break;
        case DoubleDebug:
            //这时候就得用双目程序了
            chooseArmorDouble(armors);
            break;
        case SentryAttack:
            //这时候就得用双目程序了
            chooseArmorDouble(armors);
            break;
        default:
            break;
        }
    }
    void ArmorDetectorDouble::chooseArmorDouble(std::vector<Armor> &armors)
    {

        int leftFlag = 0;
        int rightFlag = 0;
        bool flag1 = 0, flag2 = 0;
        armors.resize(2);
        if (m_ArmorSingles[0].m_bestArmor.m_bestArmorStatus != NOTFIND)
        {
            leftFlag = 1;
        }
        if (m_ArmorSingles[1].m_bestArmor.m_bestArmorStatus != NOTFIND)
        {
            rightFlag = 1;
        }
        if (leftFlag && rightFlag)
        {
            chooseTargetArmorDouble(armors);
        }
        else
        {
            if (leftFlag)
            {
                chooseTargetArmorSingle(armors, 0);
            }
            else if (rightFlag)
            {
                chooseTargetArmorSingle(armors, 1);
            }
            else
            {
                armors[0].m_armor_type = ARMOR_NONE;
                armors[1].m_armor_type = ARMOR_NONE;
            }
            m_ArmorSingles[0].m_state = wmj::SEARCHING;
            m_ArmorSingles[1].m_state = wmj::SEARCHING;
        }
    }
    //单目、debug模式
    void ArmorDetectorDouble::chooseTargetArmorSingle(std::vector<Armor> &armors, int choose)
    {
        //先进行一次排序
        if (m_ArmorSingles[choose].m_bestArmor.m_bestArmorStatus != NOTFIND) // 有结果
        {
            m_ArmorSingles[choose].m_state = wmj::TRACKING;
            armors[0] = m_ArmorSingles[choose].m_bestArmor;
            m_ArmorSingles[choose].m_last_armor = armors[0];
            m_lost_num[choose] = 0;
            if (choose == 0)
            {
                armors[0].m_detectedtype = wmj::DETECT_LEFT;
            }
            else
            {
                armors[0].m_detectedtype = wmj::DETECT_RIGHT;
            }
        }
        else if (++m_lost_num[choose] < LOSE_NUM)
        {

            armors[0] = m_ArmorSingles[choose].m_last_armor;
            m_ArmorSingles[choose].m_state = wmj::BUFFER;
        }
        else
        {
            armors[0].m_armor_type = ARMOR_NONE;
            m_ArmorSingles[choose].m_state = wmj::SEARCHING;
        }
    }
    //双目程序，将两个相机识别到的装甲板进行匹配
    void ArmorDetectorDouble::chooseTargetArmorDouble(std::vector<Armor> &armors)
    {
        Armor armors_left_last;
        Armor armors_right_last;
        int _result = 0; //做一个临时判断
        //筛选合适的选项，得到好几个等条
        if (m_ArmorSingles[0].m_bestArmor.m_bestArmorStatus != NOTFIND && m_ArmorSingles[1].m_bestArmor.m_bestArmorStatus != NOTFIND)
        {
            if (doubleCompare(m_ArmorSingles[0].m_bestArmor, m_ArmorSingles[1].m_bestArmor) &&
                m_ArmorSingles[0].m_bestArmor.m_id == m_ArmorSingles[1].m_bestArmor.m_id)
            {
                _result = 1;

                armors_left_last = m_ArmorSingles[0].m_bestArmor;
                armors_right_last = m_ArmorSingles[1].m_bestArmor;
            }
            else
            {
                for (int i = 0; i < m_ArmorSingles[0].m_armors.size(); i++)
                {
                    for (int j = 0; j < m_ArmorSingles[1].m_armors.size(); j++)
                    {
                        if (doubleCompare(m_ArmorSingles[0].m_armors[i], m_ArmorSingles[1].m_armors[j]) && !_result)
                        {

                            int idLeft = m_ArmorSingles[0].SetArmorId(m_ArmorSingles[0].m_armors[i]);
                            int idRight = m_ArmorSingles[1].SetArmorId(m_ArmorSingles[1].m_armors[j]);
                            if (idLeft == 0 || idRight == 0 || idLeft == 2 || idRight == 2 || idLeft == 11 || idRight == 11)
                            {
                                continue;
                            }
                            if (idLeft != idRight)
                            {
                                continue;
                            }
                            armors_left_last = m_ArmorSingles[0].m_armors[i];
                            armors_right_last = m_ArmorSingles[1].m_armors[j];
                            _result = 1; //有一次为真即可，后续补充多组合优先度判定
                        }
                    }
                }
            }
            if (_result == 1)
            {
                armors[0] = armors_left_last;
                armors[1] = armors_right_last;
                m_lost_num[0] = 0;
                m_lost_num[1] = 0;
                armors[0].m_detectedtype = wmj::DETECT_DOUBLE;

                armors[1].m_detectedtype = wmj::DETECT_DOUBLE;
                m_ArmorSingles[0].m_last_armor = armors[0];
                m_ArmorSingles[1].m_last_armor = armors[1];
                m_ArmorSingles[0].m_state = wmj::TRACKING;
                m_ArmorSingles[1].m_state = wmj::TRACKING;
            }
            else
            {
                armors[0].m_armor_type = ARMOR_NONE;
                armors[1].m_armor_type = ARMOR_NONE;
                m_ArmorSingles[0].m_state = wmj::SEARCHING;
                m_ArmorSingles[1].m_state = wmj::SEARCHING;
            }
        }
        else
        {
            armors[0].m_armor_type = ARMOR_NONE;
            m_ArmorSingles[0].m_state = wmj::SEARCHING;
            armors[1].m_armor_type = ARMOR_NONE;
            m_ArmorSingles[1].m_state = wmj::SEARCHING;
        }
    }
    //判断两个装甲板是否匹配
    bool ArmorDetectorDouble::doubleCompare(Armor a, Armor b)
    {

        double center_diff = (a.m_center.x - b.m_center.x) / ((a.m_width + b.m_width) / 2);
        //视差占装甲板的比例

        double ratio_diff = a.m_ratio > b.m_ratio ? a.m_ratio / b.m_ratio : b.m_ratio / a.m_ratio;

        double width_diff = a.m_width > b.m_width ? a.m_width / b.m_width : b.m_width / a.m_width;

        double height_diff = a.m_height > b.m_height ? a.m_height / b.m_height : b.m_height / a.m_height;

        if (ratio_diff > 1.8 || width_diff > 1.8 || height_diff > 1.8)
        {
            return false;
        }
        if (center_diff < 0.1 || center_diff > 3.4)
        {
            return false;
        }
        return true;
    }
    double ArmorDetectorDouble::compareRate(double a, double b)
    {
        double result_rate;
        if (a < b)
            result_rate = b / a;
        else
            result_rate = a / b;
        return result_rate;
    }

    /*###################################################

            对目标装甲板测距，并做最后筛选
    ###################################################*/
    Armor ArmorDetectorDouble::measureArmorDistance(std::vector<Armor> &armors)
    {
        Armor armor_final; //定义最后传出armor
        if (armors[0].m_armor_type == ARMOR_NONE)
        {

            armor_final.m_armor_type = ARMOR_NONE;
        }
        else
        {
            armor_final = armors[0];
            armor_final.m_armor_type = armors[0].m_armor_type;

            if (armors[0].m_detectedtype != wmj::DETECT_DOUBLE)
            {
                armor_final.m_position = calcArmorDeepth(armors[0]);
                if (armor_final.m_detectedtype == wmj::DETECT_RIGHT)
                {

                    armor_final.m_position.y -= m_repairPoint.y;
                }
                if (armor_final.m_position.x > m_max_single_distance)
                {

                    armors[0].m_armor_type = ARMOR_NONE;
                    armor_final.m_armor_type = ARMOR_NONE;
                }
            }
            else
            {
                //双目测距
                pair<Point2f, Point2f> leftLightsCenter = make_pair(armors[0].m_pairs[0].m_center, armors[0].m_pairs[1].m_center);
                pair<Point2f, Point2f> rightLightsCenter = make_pair(armors[1].m_pairs[0].m_center, armors[1].m_pairs[1].m_center);
                //获取中点坐标
                tie(armor_final.m_position, armor_final.m_yaw_angle) = m_binocular.getPositionAngle(leftLightsCenter, rightLightsCenter);
                if (armor_final.m_position.x > 10000)
                {
                    armors[0].m_armor_type = ARMOR_NONE;
                    armor_final.m_armor_type = ARMOR_NONE;
                }
            }
            if (armor_final.m_position.x > 5000)
            {
                m_ArmorSingles[0].m_roi_value = 10;
                m_ArmorSingles[1].m_roi_value = 10;
            }
            else if (armor_final.m_position.x > 4000)
            {
                m_ArmorSingles[0].m_roi_value = 7;
                m_ArmorSingles[1].m_roi_value = 7;
            }
            else
            {
                m_ArmorSingles[0].m_roi_value = 4;
                m_ArmorSingles[1].m_roi_value = 4;
            }
        }
        return armor_final;
    }
    /* ##########################################
                单目测距函数
    #######################################*/
    cv::Point3f ArmorDetectorDouble::calcArmorDeepth(const Armor &armor)
    {
        std::vector<cv::Point3f> op_v;
        if (armor.m_armor_type == wmj::ARMOR_SMALL)
            GetObjPoints(m_monocular.m_small_width, m_monocular.m_small_height, op_v);
        else if (armor.m_armor_type == wmj::ARMOR_LARGE)
            GetObjPoints(m_monocular.m_large_width, m_monocular.m_large_height, op_v);
        else
            return cv::Point3f(500.f, 500.f, 500.f);

        Mat opoints(op_v, CV_32FC1);
        std::vector<cv::Point2f> ipoints_v;
        ipoints_v.push_back(cv::Point2f(armor.m_vertices[1].x, armor.m_vertices[1].y));
        ipoints_v.push_back(cv::Point2f(armor.m_vertices[0].x, armor.m_vertices[0].y));
        ipoints_v.push_back(cv::Point2f(armor.m_vertices[3].x, armor.m_vertices[3].y));
        ipoints_v.push_back(cv::Point2f(armor.m_vertices[2].x, armor.m_vertices[2].y));
        Mat rvec, tvec;
        bool ret;
        if (armor.m_detectedtype == wmj::DETECT_LEFT)
        {
            ret = solvePnP(opoints, Mat(ipoints_v), m_ArmorSingles[0].m_CameraMat, m_ArmorSingles[0].m_DistMat, rvec, tvec);
        }
        else
        {
            ret = solvePnP(opoints, Mat(ipoints_v), m_ArmorSingles[1].m_CameraMat, m_ArmorSingles[1].m_DistMat, rvec, tvec);
        }

        if (ret)
        {
            return cv::Point3f(tvec.ptr<double>(0)[2] * 1000, tvec.ptr<double>(0)[0] * -1000, tvec.ptr<double>(0)[1] * (-1) * 1000);
        }
        else
        {
            return cv::Point3f(500.f, 500.f, 500.f);
        }
    }

    void ArmorDetectorDouble::GetObjPoints(float _width, float _height,
                                           std::vector<cv::Point3f> &op_v)
    {
        op_v.clear();
        float width = _width * 1., height = _height * 1.;
        std::vector<cv::Point3f> opoints_v(4);
        op_v.push_back(cv::Point3f(-width / 2.f, -height / 2.f, 0.0));
        op_v.push_back(cv::Point3f(-width / 2.f, height / 2.f, 0.0));
        op_v.push_back(cv::Point3f(width / 2.f, height / 2.f, 0.0));
        op_v.push_back(cv::Point3f(width / 2.f, -height / 2.f, 0.0));
    }

    void ArmorDetectorDouble::init()
    {
        m_ArmorSingles[0].m_detectnum = -1;
        m_ArmorSingles[1].m_detectnum = -1;
    }

    /*  #############################################
             识别主程序：
                     1. 先双线程运行
                     2. 选择目标装甲板
                     3. 筛选后输出结果
     ################################################
     */

    Armor ArmorDetectorDouble::DetectArmorDouble(
        std::vector<wmj::MatWithTime> frames, std::vector<Armor> &armors)
    {
        m_state = (enum State)i_state;
        cout << "已选择模式: " << m_state << endl;

        detectArmor(frames);

        chooseTargetArmor(armors);

        Armor finalArmor = measureArmorDistance(armors);

        return finalArmor;
    }
} // namespace wmj
