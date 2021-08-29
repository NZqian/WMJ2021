#include "../include/ArmorSingle.hpp"
#include "../include/ArmorDebug.hpp"
#include <algorithm>
#include <thread>
#define MIN_DIS 10

using namespace std;
using namespace cv;
using namespace cv::ml;

std::shared_ptr<wmj::ArmorDebug> debug = std::make_shared<wmj::ArmorDebug>();

namespace wmj
{

    template <typename _Tp>
    //计算两点间的距离
    double getDistance(const Point_<_Tp> &p1, const Point_<_Tp> &p2)
    {
        const Point_<_Tp> tmp = p1 - p2;
        return sqrt(tmp.ddot(tmp));
    }

    /* 旋转矩形规范化 使得width为长边，角度(-180,0]*/
    void regularLight(Light &light)
    {
        if (light.m_rect.size.width < light.m_rect.size.height)
        {
            swap(light.m_rect.size.width, light.m_rect.size.height);
            light.m_rect.angle -= 90.0f;
        }
    }

    /*##############################################
            ArmorParam类函数
    ##############################################*/

    void ArmorParam::setParam(const cv::FileStorage &fs)
    {
        //预处理参数
        fs["ArmorDetector"]["roi_value"] >> m_roi_value;
        fs["ArmorDetector"]["min_thresh_red"] >> m_min_thresh_red;
        fs["ArmorDetector"]["min_thresh_blue"] >> m_min_thresh_blue;
        fs["ArmorDetector"]["color_thresh"] >> m_color_thresh;
        //灯条参数
        fs["ArmorDetector"]["light_max_area"] >> m_light_max_area;
        fs["ArmorDetector"]["light_min_area"] >> m_light_min_area;
        fs["ArmorDetector"]["light_min_ratio"] >> m_light_min_ratio;
        fs["ArmorDetector"]["light_max_ratio"] >> m_light_max_ratio;
        fs["ArmorDetector"]["light_min_angle"] >> m_light_min_angle;
        fs["ArmorDetector"]["light_max_angle"] >> m_light_max_angle;
        fs["ArmorDetector"]["light_area_ratio"] >> m_light_area_ratio;
        fs["ArmorDetector"]["RBdiff"] >> m_min_RBdiff;
        fs["ArmorDetector"]["BRdiff"] >> m_min_BRdiff;
        //装甲板参数
        fs["ArmorDetector"]["armor_tiltangle"] >> m_armor_tiltangle;
        fs["ArmorDetector"]["armor_malposition"] >> m_armor_malposition;
        fs["ArmorDetector"]["armor_ratio_max"] >> m_armor_ratio_max;
        fs["ArmorDetector"]["armor_ratio_min"] >> m_armor_ratio_min;
        fs["ArmorDetector"]["lights_diff_max"] >> m_lights_diff;
        fs["ArmorDetector"]["armor_min_area"] >> m_armor_min_area;
        fs["ArmorDetector"]["angle_diff"] >> m_angle_diff;
        //敌方颜色
        fs["ArmorDetector"]["enemy_color"] >> m_enemy_color;

        fs["dimDetect"]["light_max_area"] >> m_dimlight_max_area;
        fs["dimDetect"]["light_min_area"] >> m_dimlight_min_area;
        fs["dimDetect"]["light_min_ratio"] >> m_dimlight_min_ratio;
        fs["dimDetect"]["light_max_ratio"] >> m_dimlight_max_ratio;
        fs["dimDetect"]["light_min_angle"] >> m_dimlight_min_angle;
        fs["dimDetect"]["light_max_angle"] >> m_dimlight_max_angle;
        fs["dimDetect"]["light_area_ratio"] >> m_dimlight_area_ratio;
        fs["dimDetect"]["RBdiff"] >> m_dim_min_RBdiff;
        fs["dimDetect"]["BRdiff"] >> m_dim_min_BRdiff;

        //装甲板参数
        fs["dimDetect"]["armor_tiltangle"] >> m_dim_armor_tiltangle;
        fs["dimDetect"]["angle_diff"] >> m_dim_angle_diff;
        fs["dimDetect"]["armor_malposition"] >> m_dim_armor_malposition;
        fs["dimDetect"]["armor_ratio_max"] >> m_dim_armor_ratio_max;
        fs["dimDetect"]["armor_ratio_min"] >> m_dim_armor_ratio_min;
        fs["dimDetect"]["lights_diff_max"] >> m_dim_lights_diff;
    }

    /*#########################################
                Light类函数
    ########################################*/
    Light::Light(const vector<Point> contour, const Point2d &base)
    {
        m_rect = fitEllipse(contour);
        regularRect(m_rect);
        ;
        m_rectR = boundingRect(contour);
        m_center = m_rect.center;
        m_length = m_rect.size.width;
        m_width = m_rect.size.height;
        m_area = m_rectR.area();
        m_ratio = m_length / m_width;
        m_contour = contour;
        m_area_ratio = contourArea(contour) / (m_width * m_length);
    }

    bool Light::isLight(ArmorParam _param, bool dimOrlight)
    {
        //  if(!(m_width > 2 && m_length > 4.5))
        //  {
        //      cout<<"width:" << m_width << endl;
        //      cout<<"length:" << m_length << endl;
        //  }
        //  else if(!(m_area < _param.m_light_max_area && m_area > _param.m_light_min_area  ))
        // {
        //     cout<<"area:"<<m_area<<endl;
        // }
        //  else if(!(m_ratio < _param.m_light_max_ratio && m_ratio > _param.m_light_min_ratio))
        //  {
        //      cout<<"ratio:"<<m_ratio<<endl;
        //  }
        //  else if(!(m_area_ratio > _param.m_light_area_ratio))
        //  {
        //       cout<<"arearatio:"<<m_area_ratio<<endl;
        //  }
        //  else
        //  {
        //      cout <<"IsLight"<<endl;
        //  }
        if (dimOrlight)
        {
            return m_width > 2 && m_length > 4.5 &&
                   m_ratio < _param.m_light_max_ratio &&
                   m_area < _param.m_light_max_area &&
                   m_area_ratio > _param.m_light_area_ratio &&
                   m_area > _param.m_light_min_area &&
                   m_ratio > _param.m_light_min_ratio;
        }
        else
        {
            return m_ratio > _param.m_dimlight_min_ratio &&
                   m_ratio < _param.m_dimlight_max_ratio &&
                   m_area > _param.m_dimlight_min_area &&
                   m_area < _param.m_dimlight_max_area &&
                   m_angle > _param.m_dimlight_min_angle &&
                   m_angle < _param.m_dimlight_max_angle &&
                   m_area_ratio > _param.m_dimlight_area_ratio;
        }
    }
    void Light::regularRect(cv::RotatedRect &rect)
    {
        if (rect.size.height > rect.size.width)
        {
            std::swap<float>(rect.size.height, rect.size.width);
            rect.angle =
                rect.angle >= 0.0f ? rect.angle - 90.0f : rect.angle + 90.0f;
        }
        if (rect.angle < 0)
        {
            rect.angle += 180.0f;
        }
    }

    /*#################################
                Armor类函数
    #################################*/

    Armor::Armor(const Light &left, const Light &right, Point2f targetPoint)
    {
        m_vertices.resize(4);
        //装甲板的两根灯条
        m_pairs.push_back(left);
        m_pairs.push_back(right);
        //求装甲板ROI，用于下一帧ROI识别
        Point2d leftCenter = left.m_center;
        Point2d rightCenter = right.m_center;
        double Armor_lightRatio = 2.0; //假定的灯条和装甲板长度的比值
        Point2d LTpoint, RDpoint;
        LTpoint.x = leftCenter.x - left.m_width / 2;
        LTpoint.y = leftCenter.y - left.m_length * Armor_lightRatio / 2;
        RDpoint.x = rightCenter.x + right.m_width / 2;
        RDpoint.y = rightCenter.y + right.m_length * Armor_lightRatio / 2;
        //得到装甲板框选区域
        m_rect = Rect(LTpoint, RDpoint);
        m_rect &= Rect2d(Point(0, 0), Point(1280, 1024));
        //计算装甲板中心
        m_center = Point2f((leftCenter.x + rightCenter.x) / 2, (leftCenter.y + rightCenter.y) / 2);
        m_width = getDistance(left.m_center, right.m_center);                                   //横向长度
        m_height = (left.m_length + right.m_length) / 2;                                        //纵向长度
        m_ratio = m_width / m_height;                                                           //长宽比
        m_armor_type = m_ratio > 3 ? wmj::ARMOR_LARGE : wmj::ARMOR_SMALL;                                   //装甲板分类
        m_tiltAngle = asin(abs(left.m_center.y - right.m_center.y) / m_width) * 180 / PI;       //倾斜角
        m_angle_diff = abs(m_pairs[0].m_angle - m_pairs[1].m_angle);                            //装甲板的灯条角度差
        m_lighsRatio = max(left.m_length, right.m_length) / min(left.m_length, right.m_length); //两个灯条长度的比值
        //框出灯条画出的矩形
        Point2f left_points[4], right_points[4];
        m_pairs[0].m_rect.points(left_points);
        m_pairs[1].m_rect.points(right_points);
        m_vertices[0] = (left_points[0] + left_points[1]) / 2;
        m_vertices[1] = (left_points[3] + left_points[2]) / 2;
        m_vertices[2] = (right_points[2] + right_points[3]) / 2;
        m_vertices[3] = (right_points[1] + right_points[0]) / 2;
        m_lightRect = minAreaRect(m_vertices);
        /*###############################
        根据装甲板到屏幕中心的距离和几何形状规范程度计算装甲板得分
        ################################*/
        m_socre = exp(-getDistance(m_center, targetPoint) / 100) + m_tiltAngle / 5;
    }

    /* 装甲不超出范围且旋转不能超过一定角度 */
    bool Armor::IsArmor(ArmorParam _param, bool dimOrLight)
    {

        /*########################################
        用于装甲板几何筛选debug，如果灯条能框到，但被筛掉，则可以解注释这里查看
        #########################################*/

        //   if(!(m_tiltAngle < _param.m_armor_tiltangle))
        //   {
        //       cout<<"m_tilAngle:"<<m_tiltAngle<<endl;
        //   }
        //   else if(!(m_ratio < _param.m_armor_ratio_max&&_param.m_armor_ratio_min < m_ratio))
        //   {
        //       cout<<"ratio"<<m_ratio<<endl;
        //   }
        //   else if(!(m_lighsRatio < _param.m_lights_diff))
        //   {
        //       cout<<"light_diff"<<m_lighsRatio<<endl;
        //   }
        //   else if(!(m_width > 7))
        //   {
        //       cout<<"width:"<<m_width<<endl;
        //   }
        //   else if(!(m_height > 5))
        //   {
        //       cout<<"height:"<<m_width<<endl;
        //   }
        //   else if( !(m_angle_diff < _param.m_angle_diff))
        //   {
        //       cout<<"anglediff:"<<m_angle_diff<<endl;
        //   }
        //   else
        //   {
        //       cout<<"IsArmor"<<endl;
        //   }
        if (dimOrLight)
        {
            if (!(m_tiltAngle < _param.m_armor_tiltangle &&
                  m_ratio < _param.m_armor_ratio_max &&
                  m_ratio > _param.m_armor_ratio_min &&
                  m_lighsRatio < _param.m_lights_diff &&
                  m_width > 7 && m_height > 5 &&
                  m_angle_diff < _param.m_angle_diff))
            {
                return 0;
            }

            return 1;
        }
        else
        {

            if (!(m_tiltAngle < _param.m_dim_armor_tiltangle &&
                  m_ratio < _param.m_dim_armor_ratio_max &&
                  m_ratio > _param.m_dim_armor_ratio_min &&
                  m_lighsRatio < _param.m_dim_lights_diff &&
                  m_width > _param.m_dim_armor_malposition &&
                  m_angle_diff < _param.m_dim_angle_diff))
            {
                return 0;
            }

            return 1;
        }
    }

    //装甲优先度,找到距离镜头中心最近的装甲板
    bool Armor::operator>(const Armor &armor) const
    {
        if (m_socre > armor.m_socre)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }

    //svm类函数
    HOG_SVM::HOG_SVM()
    {
        m_label2id = {{0, 0}, {1, 1}, {2, 2}, {3, 3}, {4, 4}, {5, 5}, {6, 11}, {7, 7}, {8, 8}};
        m_hog.winSize = Size(48, 32);
        m_hog.blockSize = Size(16, 16);
        m_hog.blockStride = Size(8, 8);
        m_hog.cellSize = Size(8, 8);
        m_hog.nbins = 9;
        m_hog.derivAperture = 1;
        m_hog.winSigma = -1;
        m_hog.histogramNormType = HOGDescriptor::L2Hys;
        m_hog.L2HysThreshold = 0.2;
        m_hog.gammaCorrection = false;
        m_hog.free_coef = -1.f;
        m_hog.nlevels = HOGDescriptor::DEFAULT_NLEVELS;
        m_hog.signedGradient = false;
        if (m_svm)
        {
            m_svm->clear();
        }
        m_svm = SVM::load(SVM_XML);
    }
    int HOG_SVM::test(const Mat &src)
    {
        if (m_svm)
        {
            vector<float> descriptors;
            m_hog.compute(src, descriptors, Size(8, 8));
            int label = m_svm->predict(descriptors);
            return m_label2id[label];
        }
        else
        {
            return 0;
        }
    }

    //ArmorSingle类函数

    //计算轮廓上红蓝通道差值来判断颜色
    bool ArmorSingle::judgeColor(Light &light, vector<Point> Contours)
    {

        Vec3b pixel;
        int red = 0, blue = 0;
        for (int j = 0; j < Contours.size(); j++)
        {
            pixel = m_src.at<Vec3b>(Point(Contours[j]));
            red += pixel[0];
            blue += pixel[2];
        }
        int RB_diff = (red - blue) / int(Contours.size());
        if (useNumber)
        {
            if (RB_diff > m_param.m_min_RBdiff)
            {
                light.m_color = 0;
            }
            else if (RB_diff < (-1 * m_param.m_min_BRdiff))
            {
                light.m_color = 1;
            }
            else
            {
                light.m_color = 2;
            }
        }
        else
        {
            if (RB_diff > m_param.m_dim_min_RBdiff)
            {
                light.m_color = 0;
            }
            else if (RB_diff < (-1 * m_param.m_dim_min_BRdiff))
            {
                light.m_color = 1;
            }
            else
            {
                light.m_color = 2;
            }
        }

        if (light.m_color == m_param.m_enemy_color)
        {

            return true;
        }
        else
        {
            return false;
        }
    }
    //选择ROI
    void ArmorSingle::roiChoose()
    {

        if (m_state == wmj::SEARCHING)
        {
            m_roi = Rect2d(Point(0, 0), Point(m_src.cols, m_src.rows));
            return;
        }
        else if (m_state == wmj::BUFFER)
        {
            m_roi.x = m_roi.x > m_src.cols / 10 ? m_roi.x - m_src.cols / 10 : 0;
            m_roi.y = m_roi.y > m_src.rows / 10 ? m_roi.y - m_src.rows / 10 : 0;
            m_roi.width = (m_roi.x + m_roi.width + m_src.cols / 5) < m_src.cols ? (m_roi.width + m_src.cols / 5) : m_src.cols - 1 - m_roi.x;
            m_roi.height = (m_roi.y + m_roi.height + m_src.rows / 5) < m_src.rows ? (m_roi.height + m_src.rows / 5) : m_src.rows - 1 - m_roi.y;
            m_roi &= Rect2d(Point(0, 0), Point(m_src.cols, m_src.rows));
            return;
        }
        else if (m_state == wmj::TRACKING)
        {
            m_roi = m_last_armor.m_rect;

            m_roi.x -= (m_roi_value - 1) * m_roi.width / 2.0;
            m_roi.y -= (m_roi_value - 1) * m_roi.height / 2.0;
            m_roi.width = m_roi.width * m_roi_value;
            m_roi.height = m_roi.height * m_roi_value;
            m_roi.x = m_roi.x > 0 ? m_roi.x : 0;
            m_roi.y = m_roi.y > 0 ? m_roi.y : 0;
            m_roi.width = (m_roi.x + m_roi.width) < m_src.cols ? m_roi.width : m_src.cols - 1 - m_roi.x;
            m_roi.height = (m_roi.y + m_roi.height) < m_src.rows ? m_roi.height : m_src.rows - 1 - m_roi.y;
            m_roi &= Rect2d(Point(0, 0), Point(m_src.cols, m_src.rows));

            return;
        }
        else
        {
            m_roi = Rect2d(Point(0, 0), Point(m_src.cols, m_src.rows));
            return;
        }
    }
    //获得二值化阈值
    int getThresh(ArmorParam param, bool Islight)
    {
        if (param.m_enemy_color == wmj::_RED) // 0是红色
        {
            if (Islight)
            {
                return param.m_min_thresh_red;
            }
            else
            {
                return param.m_min_thresh_red;
            }
        }
        else
        {
            if (Islight)
            {
                return param.m_min_thresh_blue;
            }
            else
            {
                return param.m_min_thresh_blue;
            }
        }
    }
    //图像预处理
    void ArmorSingle::preProcess()
    {
        /* 识别区域选择 */
        roiChoose();
        Mat src_roi = m_src(m_roi);
        // 分通道
        vector<Mat> rgb;
        split(src_roi, rgb);
        int thresh_light = getThresh(m_param, useNumber);
        // 根据颜色选通道
        if (m_param.m_enemy_color == wmj::_RED)
        {
            m_gray = rgb[0];
        }
        else
        {
            m_gray = rgb[2];
        }
        // 二值化
        threshold(m_gray, m_binary_gray, thresh_light, 255, cv::THRESH_BINARY);
        // 形态学开操作
        cv::Mat element = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 5));
        morphologyEx(m_binary_gray, m_binary_gray, cv::MORPH_OPEN, element);
    }
    //识别灯条
    bool ArmorSingle::findLights()
    {
        vector<vector<Point>> contours_gray;
        vector<Vec4i> lines;
        // 找轮廓
        findContours(m_binary_gray, contours_gray, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, m_roi.tl());
        for (size_t i = 0; i < contours_gray.size(); i++)
        {
            RotatedRect rect = minAreaRect(contours_gray[i]);
            if (contours_gray[i].size() > 10 && rect.boundingRect().area() > 10) //粗筛选
            {
                Light light(contours_gray[i], m_roi.tl());
                if (light.isLight(m_param, useNumber))
                {
                    // 几何条件筛选
                    if (judgeColor(light, contours_gray[i]) && light.m_rect.angle > 20 && light.m_rect.angle < 160)
                    {
                        // 颜色判断
                        cv::Vec4f line;
                        vector<Point2f> tmp;
                        Mat(contours_gray[i]).convertTo(tmp, CV_32FC1);
                        resize(tmp, tmp, Size(20, 1));
                        // 线性拟合
                        fitLine(tmp, line, CV_DIST_HUBER, 0, 0.01, 0.01);
                        light.m_angle = atan2(line[1], line[0]) * 180 / PI;
                        light.m_angle = light.m_angle < 0 ? light.m_angle + 180 : light.m_angle;
                        if (light.m_angle > 60 && light.m_angle < 120)
                        {
                            m_lights.push_back(light);
                        }
                    }
                }
            }
        }

        return m_lights.size() >= 2;
    }

    bool ArmorSingle::findArmors()
    {
        // 灯条从左到右排序
        if (m_lights.size() != 0)
        {
            sort(m_lights.begin(), m_lights.end(), [](const Light &a, const Light &b)
                 { return a.m_center.x < b.m_center.x; });
        }
        else
        {
            m_bestArmor.m_bestArmorStatus = wmj::NOTFIND;
            return !m_armors.empty();
        }

        //找相邻的两个灯条两两匹配
        for (size_t i = 0; i < m_lights.size() - 1; i++)
        {
            for (size_t j = i + 1; j < m_lights.size(); j++)
            {
                Light left = m_lights[i];
                Light right = m_lights[j];
                Point2f targetPoint;
                // 测算图传中心对应的左右目中心
                if (m_decide)
                {
                    targetPoint = Point2f(620, 552);
                }
                else
                {
                    targetPoint = Point2f(660, 552);
                }
                // 如果灯条最低点高于灯条最高点，则必定不匹配
                if (left.m_rect.boundingRect2f().br().y < right.m_rect.boundingRect2f().tl().y || right.m_rect.boundingRect2f().br().y < left.m_rect.boundingRect2f().tl().y)
                {
                    continue;
                }

                if (m_endebug)
                {
                    debug->m_debugLishtsRoi.push_back(m_lights[i]);
                    debug->m_debugLishtsRoi.push_back(m_lights[j]);
                }

                Armor armor(left, right, targetPoint);
                if (armor.IsArmor(m_param, useNumber))
                {
                    // 装甲板几何筛选
                    if (!IsFakeArmor(i, j))
                    {
                        // 判断是否是虚假装甲板
                        m_armors.emplace_back(armor);
                    }
                }
            }
        }
        // 判断装甲板是否重叠
        if (m_armors.size() > 1)
        {
            for (int i = 0; i < m_armors.size() - 1; i++)
            {
                for (int j = i + 1; j < m_armors.size(); j++)
                {
                    if (needToDelate(m_armors[i], m_armors[j]))
                    {
                        m_armors[i].m_armor_type = ARMOR_NONE;
                    }
                }
            }
        }
        // 删除不符合的装甲板
        m_armors.erase(std::remove_if(m_armors.begin(), m_armors.end(), [](Armor a)
                                      { return a.m_armor_type == ARMOR_NONE; }),
                       m_armors.end());
        // 找目标装甲板
        if (m_armors.size() != 0)
        {
            if (!findTargetArmor() && useNumber)
            {
                m_bestArmor.m_bestArmorStatus = NOTFIND;
            }
            else
            {
                m_bestArmor.m_bestArmorStatus = FIRST;
            }
        }
        else
        {
            m_bestArmor.m_bestArmorStatus = wmj::NOTFIND;
        }
        return !m_armors.empty();
    }
    // 如果装甲板中间有灯条证明是虚假装甲板
    bool ArmorSingle::IsFakeArmor(int i, int j)
    {
        Rect2d rect_left = m_lights[i].m_rectR;
        Rect2d rect_right = m_lights[j].m_rectR;
        double min_y, max_y;
        min_y = fmin(rect_left.y, rect_right.y) + 2;
        max_y = fmax(rect_left.y + rect_left.height, rect_right.y + rect_right.height) - 2;

        for (int s = i + 1; s < j; s++)
        {
            if ((m_lights[s].m_center.y < max_y && m_lights[s].m_center.y > min_y) || (m_lights[s].m_rectR.y < max_y && m_lights[s].m_rectR.y > min_y) || ((m_lights[s].m_rectR.y + m_lights[s].m_rectR.height) < max_y && (m_lights[s].m_rectR.y + m_lights[s].m_rectR.height) > min_y))
            {
                return true;
            }
        }
        return false;
    }
    // 判断装甲板是否相似
    bool compare(Armor a, Armor b)
    {
        double ratio_diff = a.m_ratio > b.m_ratio ? a.m_ratio / b.m_ratio : b.m_ratio / a.m_ratio;

        double width_diff = a.m_width > b.m_width ? a.m_width / b.m_width : b.m_width / a.m_width;

        double height_diff = a.m_height > b.m_height ? a.m_height / b.m_height : b.m_height / a.m_height;

        double center_diff = (a.m_center.x - b.m_center.x) / ((a.m_width + b.m_width) / 2);

        double tiltangle_diff = abs(a.m_tiltAngle - b.m_tiltAngle);

        if (center_diff > 1.4)
        {
            return false;
        }
        if (ratio_diff > 1.1 || width_diff > 1.1 || height_diff > 1.1)
        {
            return false;
        }
        if (tiltangle_diff > 5)
        {
            return false;
        }
        return true;
    }
    // 找目标装甲板
    bool ArmorSingle::findTargetArmor()
    {
        //如果只识别到了一个装甲板
        if (m_armors.size() == 1)
        {
            m_bestArmor = m_armors[0];
            //如果之前没有找到装甲板
            if (m_state == wmj::SEARCHING)
            {
                // 数字识别
                int armorID = SetArmorId(m_armors[0]);
                if (armorID == 0)
                {
                    return false;
                }
                if (armorID == 2)
                {
                    cout << "识别到工程" << endl;
                    return false;
                }
                if (armorID == 11)
                {
                    cout << "识别到前哨站" << endl;
                    return false;
                }
                if (armorID == m_detectnum)
                {
                    m_bestArmor.m_bestArmorStatus = SHOOT;
                    m_bestArmor.m_id = armorID;
                    m_armors[0].m_id = armorID;
                    return true;
                }
                else
                {
                    if (m_detectnum == -1)
                    {
                        m_detectnum = armorID;
                        m_bestArmor.m_bestArmorStatus = FIRST;
                        m_bestArmor.m_id = armorID;
                        m_armors[0].m_id = armorID;
                        return true;
                    }
                    else
                    {
                        return false;
                    }
                }
            }
            //如果之前找到装甲板，则给装甲板更换状态
            else if (compare(m_last_armor, m_armors[0]))
            {

                m_bestArmor.m_bestArmorStatus = SHOOT;
                m_bestArmor.m_id = m_last_armor.m_id;
                m_armors[0].m_id = m_last_armor.m_id;
                return true;
            }
            else //切换装甲板
            {

                int armorID = SetArmorId(m_armors[0]);
                if (armorID == 0)
                {
                    return false;
                }
                if (armorID == 2)
                {
                    cout << "识别到工程" << endl;
                    return false;
                }
                if (armorID == 11)
                {
                    cout << "识别到前哨站" << endl;
                    return false;
                }
                if (armorID == m_detectnum)
                {
                    m_bestArmor.m_bestArmorStatus = SHOOT;
                    m_bestArmor.m_id = armorID;
                    m_armors[0].m_id = armorID;
                    return true;
                }
                else
                {
                    return false;
                }
            }
        }
        //如果在找装甲板
        if (m_state == wmj::SEARCHING)
        {

            sort(m_armors.begin(), m_armors.end(), [&](const Armor &a, const Armor &b)
                 { return a > b; });
            //排序后找到最好的装甲板
            for (int i = 0; i < m_armors.size(); i++)
            {
                int armorID = SetArmorId(m_armors[i]);

                if (armorID == 0)
                {
                    continue;
                }
                if (armorID == 2)
                {
                    cout << "识别到工程" << endl;
                    continue;
                }
                if (armorID == 11)
                {
                    cout << "识别到前哨站" << endl;
                    continue;
                }
                if (armorID == m_detectnum)
                {
                    m_bestArmor = m_armors[i];
                    m_bestArmor.m_bestArmorStatus = SHOOT;
                    m_bestArmor.m_id = armorID;
                    m_armors[i].m_id = armorID;
                    return true;
                }
                else
                {
                    if (m_detectnum == -1)
                    {
                        m_bestArmor = m_armors[i];
                        m_detectnum = armorID;
                        m_bestArmor.m_bestArmorStatus = FIRST;
                        m_bestArmor.m_id = armorID;
                        m_armors[i].m_id = armorID;
                        return true;
                    }
                    else
                    {
                        continue;
                    }
                }
            }
            return false;
        }
        float ScoreMIN = 4.3;
        int index = -1;
        for (size_t i = 0; i < m_armors.size(); i++)
        {
            // 排序找到匹配度最高的装甲板
            double ratio_diff = m_last_armor.m_ratio > m_armors[i].m_ratio ? m_last_armor.m_ratio / m_armors[i].m_ratio : m_armors[i].m_ratio / m_last_armor.m_ratio;
            double width_diff = m_last_armor.m_width > m_armors[i].m_width ? m_last_armor.m_width / m_armors[i].m_width : m_armors[i].m_width / m_last_armor.m_width;
            double height_diff = m_last_armor.m_height > m_armors[i].m_height ? m_last_armor.m_height / m_armors[i].m_height : m_armors[i].m_height / m_last_armor.m_height;
            double center_diff = getDistance(m_last_armor.m_center, m_armors[i].m_center) / ((m_last_armor.m_width + m_armors[i].m_width) / 2);
            double tiltangle_diff = m_armors[i].m_tiltAngle > m_last_armor.m_tiltAngle ? m_armors[i].m_tiltAngle / m_last_armor.m_tiltAngle : m_last_armor.m_tiltAngle / m_armors[i].m_tiltAngle;
            double score = ratio_diff + width_diff + height_diff + tiltangle_diff;
            if (score <= ScoreMIN)
            {
                ScoreMIN = score;
                if (center_diff < 1.2)
                {
                    index = i;
                }
            }
        }

        if (index != -1)
        {
            m_bestArmor = m_armors[index];
            m_bestArmor.m_bestArmorStatus = SHOOT;
            m_bestArmor.m_id = m_last_armor.m_id;
            return true;
        }
        else
        {
            // 数字识别
            sort(m_armors.begin(), m_armors.end(), [&](const Armor &a, const Armor &b)
                 { return a > b; });
            for (int i = 0; i < m_armors.size(); i++)
            {
                int armorID = SetArmorId(m_armors[i]);
                if (armorID == 0)
                {
                    continue;
                }
                if (armorID == 2)
                {
                    cout << "识别到工程" << endl;
                    continue;
                }
                if (armorID == 11)
                {
                    cout << "识别到前哨站" << endl;
                    continue;
                }
                if (armorID == m_detectnum)
                {
                    m_bestArmor = m_armors[i];
                    m_bestArmor.m_bestArmorStatus = SHOOT;
                    m_bestArmor.m_id = armorID;
                    return true;
                }
                else
                {
                    continue;
                }
            }
            return false;
        }
    }
    // 删除重叠装甲板
    bool ArmorSingle::needToDelate(Armor &armor, Armor &b_armor)
    {
        if (armor.m_rect.x <= b_armor.m_rect.x && armor.m_rect.y <= b_armor.m_rect.y &&
            armor.m_rect.x + armor.m_rect.width >= b_armor.m_rect.x + b_armor.m_rect.width &&
            armor.m_rect.y + armor.m_rect.height >= b_armor.m_rect.y + b_armor.m_rect.height)
        {
            return true;
        }
        return false;
    }
    // 数字识别
    int ArmorSingle::SetArmorId(Armor &armor)
    {
        // 选择数字识别的区域
        Rect2d rect_left = armor.m_pairs[0].m_rectR;
        Rect2d rect_right = armor.m_pairs[1].m_rectR;
        double min_x, min_y, max_x, max_y;
        min_x = fmin(rect_left.x + rect_left.width, rect_right.x + rect_right.width) + 4;
        max_x = fmax(rect_left.x, rect_right.x) - 4;
        min_y = fmin(rect_left.y, rect_right.y) - 0.5 * (rect_left.height + rect_right.height) / 2.0;
        max_y = fmax(rect_left.y + rect_left.height, rect_right.y + rect_right.height) + 0.5 * (rect_left.height + rect_right.height) / 2.0;
        // 冗余操作保证不会内存越界
        min_x = min_x <= 0 ? 0 : min_x;
        min_y = min_y <= 0 ? 0 : min_y;
        max_x = max_x >= 1279 ? 1279 : max_x;
        max_y = max_y >= 1023 ? 1023 : max_y;
        if (min_x > max_x)
        {
            swap(min_x, max_x);
        }
        if (min_y > max_y)
        {
            swap(min_y, max_y);
        }
        Rect2d rect = Rect(Point(min_x, min_y), Point(max_x, max_y));
        Rect2d _bounding = Rect(Point(0, 0), Point(m_src.cols, m_src.rows));
        rect &= _bounding;

        //截取数字识别部分图片
        Mat front;
        if (useNumber)
        {
            if (rect.area() > m_param.m_armor_min_area)
            {
                Mat tmp = m_src(rect);
                resize(tmp, front, Size(48, 32));
                //转为灰度图
                cvtColor(front, front, COLOR_RGB2GRAY);
                //转换尺寸
                Rect front_roi(Point(20, 0), Size(10, 32));
                Mat front_roi_img = front(front_roi);
                //加强亮度，便于识别
                double min = 0, max = 0;
                minMaxLoc(front_roi_img, &min, &max);
                front = front * (255.0 / max);
                //识别数字
                armor.m_id = m_svm.test(front);
                if (m_endebug)
                {
                    m_numbers.push_back(front.clone());
                }
            }
            else
            {
                armor.m_id = 0;
            }
        }
        else
        {
            armor.m_id = 10;
        }

        return armor.m_id;
    }

    ArmorSingle::ArmorSingle()
    {
        FileStorage fs(ARMOR_CFG, cv::FileStorage::READ);
        // 读取参数文件
        m_param.setParam(fs);

        m_detectnum = -1;
        m_roi = cv::Rect(cv::Point(0, 0), cv::Point(m_src.cols, m_src.rows));
        m_state = wmj::SEARCHING;
        m_complete = false;
    }

    void ArmorSingle::init()
    {
        // 清空数组
        m_armors.clear();
        m_lights.clear();
        m_numbers.clear();
        m_complete = 0;
    }

    void ArmorSingle::Debug()
    {
        if (m_endebug)
        {
            debug->m_src = m_src.clone();
            debug->m_debugArmorsBefore = m_armors;
            debug->m_debugArmors.push_back(m_bestArmor);
            //debug->m_binary = m_binary.clone();
            debug->m_debugLights = m_lights;
            debug->m_binary_gray = m_binary_gray.clone();
            debug->m_numbers = m_numbers;

            debug->deBug(m_endebug - 1);
        }
    }
    Armors ArmorSingle::ArmorSingleDetector()
    {
        /*#########################
        识别主程序
        ##########################*/
        // 初始化
        init();
        // 预处理
        preProcess();
        // 识别灯条 
        findLights();
        // 识别装甲
        findArmors();
        
        Debug();
        m_complete = 1;

        return m_armors; //这里其实可以没有返回值了，有待修改
    }

    //双目debug模式调用的
    void ArmorSingle::openDoubledebug(int m_choose)
    {

        debug->m_debugType = m_choose;
        debug->m_src = m_src.clone();
        debug->m_binary_gray = m_binary_gray.clone();
        debug->m_debugLights = m_lights;
        debug->m_debugArmorsBefore = m_armors;
        debug->m_debugArmors.push_back(m_bestArmor);
        debug->showArmors();
        debug->showLights();
        debug->showBinary();
        debug->showArmorsBefore();
        debug->m_debugArmors.clear();
    }
}
