#include "../include/ArmorDebug.hpp"

using namespace std;
using namespace cv;
using namespace cv::ml;
namespace wmj
{
    void ArmorDebug::initSet()
    {
        m_debugArmors.clear();
        m_debugArmorsBefore.clear();
        m_debugLishtsRoi.clear();
        m_debugLights.clear();

        if (!m_src.empty())
        {
            cv::cvtColor(m_src, m_src, cv::COLOR_RGB2BGR);
        }
    }

    void ArmorDebug::showLights()
    {
        Mat _showImage;
        _showImage = m_src.clone();
        for (auto &_light : m_debugLights)
        {
            Point2f _points[4];
            _light.m_rect.points(_points);
            for (int i = 0; i < 4; i++)
            {
                line(_showImage, _points[i], _points[(i + 1) % 4], Scalar(0, 255, 0),
                     2);
            }
        }
        string _showName;
        _showName = "lights";
        if (m_debugType == 0)
            _showName.append("_left");
        else
            _showName.append("_right");
        namedWindow(_showName, CV_WINDOW_FREERATIO);
        imshow(_showName, _showImage);
    }

    void ArmorDebug::showVerticesDistance()
    {
        Mat _showImage;
        _showImage = m_src.clone();
        for (auto &_armor : m_debugArmors)
        {
            for (int i = 0; i < 4; i++)
            {

                circle(_showImage, Point(_armor.m_vertices[i]), 1, Scalar(0, 0, 255), -1);
            }
        }
        string _showName;
        _showName = "vertices_distance";
        if (m_debugType == 0)
            _showName.append("_left");
        else
            _showName.append("_right");
        namedWindow(_showName, CV_WINDOW_FREERATIO);
        imshow(_showName, _showImage);
    }

    void ArmorDebug::showLightRoi()
    {
        Point2f _left_points[4], _right_points[4];
        Mat _showImage;
        _showImage = m_src.clone();
        for (int i = 0; i < m_debugLishtsRoi.size(); i += 2)
        {
            m_debugLishtsRoi[i].m_rect.points(_left_points);
            m_debugLishtsRoi[i + 1].m_rect.points(_right_points);
            Point2f m_vertices[4];
            m_vertices[0] = (_left_points[0] + _left_points[1]) / 2;
            m_vertices[1] = (_left_points[2] + _left_points[3]) / 2;
            m_vertices[2] = (_right_points[2] + _right_points[3]) / 2;
            m_vertices[3] = (_right_points[1] + _right_points[0]) / 2;

            for (int i = 0; i < 4; i++)
            {
                line(_showImage, m_vertices[i], m_vertices[(i + 1) % 4], Scalar(0, 255, 255), 2);
            }
        }
        string show_name;
        show_name = "LightRoi";
        if (m_debugType == 0)
            show_name.append("_left");
        else
            show_name.append("_right");
        namedWindow(show_name, CV_WINDOW_FREERATIO);
        imshow(show_name, _showImage);
    }
    void ArmorDebug::showNumberRoi()
    {
        /*
        Lights _pairs;
        _pairs.push_back(lights_left);
        _pairs.push_back(lights_right);
        */
        Mat _showImage;
        _showImage = m_src.clone();

        for (int i = 0; i < m_debugArmorsBefore.size(); i++)
        {
            Rect2d rect_left = m_debugArmorsBefore[i].m_pairs[0].m_rectR;

            Rect2d rect_right = m_debugArmorsBefore[i].m_pairs[1].m_rectR;
            double min_x, min_y, max_x, max_y;
            min_x = fmin(rect_left.x + rect_left.width, rect_right.x + rect_right.width);
            max_x = fmax(rect_left.x, rect_right.x);
            min_y = fmin(rect_left.y, rect_right.y) - 0.5 * (rect_left.height + rect_right.height) / 2.0;
            max_y = fmax(rect_left.y + rect_left.height, rect_right.y + rect_right.height) + 0.5 * (rect_left.height + rect_right.height) / 2.0;
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
            Rect2d _bounding = Rect(Point(0, 0), Point(1280, 1024));
            rect &= _bounding;
            rect &= Rect2d(Point(0, 0), Point(1280, 1024));
            if (rect.area() > 40)
            {
                rectangle(_showImage, rect, Scalar(0, 255, 0), 2);
            }
        }

        string show_name;
        show_name = "NumberRoi";
        if (m_debugType == 0)
            show_name.append("_left");
        else
            show_name.append("_right");
        namedWindow(show_name, CV_WINDOW_FREERATIO);
        imshow(show_name, _showImage);
    }

    void ArmorDebug::showArmorsBefore()
    {
        Mat _showImage;
        _showImage = m_src.clone();
        for (auto &_armor : m_debugArmorsBefore)
        {
            rectangle(_showImage, _armor.m_rect, Scalar(0, 255, 0), 2);
            if (_armor.m_id)
                putText(_showImage, "m_id:" + to_string(_armor.m_id),
                        _armor.m_rect.tl() - Point2d(0, 10), CV_FONT_HERSHEY_SIMPLEX, 1,
                        cv::Scalar(0, 0, 255), 2);
            for (int i = 0; i < 4; i++)
            {
                line(_showImage, Point(_armor.m_vertices[i]), Point(_armor.m_vertices[(i + 1) % 4]),
                     Scalar(0, 255, 255), 2);
            }
        }
        string show_name;
        show_name = "before";
        if (m_debugType == 0)
            show_name.append("_left");
        else
            show_name.append("_right");
        namedWindow(show_name, CV_WINDOW_FREERATIO);
        imshow(show_name, _showImage);
    }

    void ArmorDebug::showArmors()
    {
        Mat _showImage;
        _showImage = m_src.clone();
        for (int i = 0; i < m_debugArmors.size(); i++)
        {
            if (m_debugArmors[i].m_bestArmorStatus != wmj::NOTFIND)
            {
                rectangle(_showImage, m_debugArmors[i].m_rect, Scalar(0, 255, 0), 2);
                putText(_showImage, "m_id:" + to_string(m_debugArmors[i].m_id),
                        m_debugArmors[i].m_rect.tl() - Point2d(0, 10), CV_FONT_HERSHEY_SIMPLEX, 1,
                        cv::Scalar(0, 0, 255), 2);

                for (int j = 0; j < 4; j++)
                {
                    line(_showImage, Point(m_debugArmors[i].m_vertices[j]), Point(m_debugArmors[i].m_vertices[(j + 1) % 4]),
                         Scalar(0, 255, 255), 2);
                }
            }
        }
        string _showName;
        _showName = "armors";
        if (m_debugType == 0)
            _showName.append("_left");
        else
            _showName.append("_right");
        namedWindow(_showName, CV_WINDOW_FREERATIO);
        imshow(_showName, _showImage);
    }

    void ArmorDebug::showBinary()
    {
        Mat _showImage;
        string _show_name;
        _showImage = m_binary_gray.clone();
        _show_name = "binary_gray";
        if (m_debugType == 0)
            _show_name.append("_left");
        else
            _show_name.append("_right");
        namedWindow(_show_name, CV_WINDOW_FREERATIO);
        imshow(_show_name, _showImage);
    }

    void ArmorDebug::showNumber()
    {
        Mat _showImage;
        for (int i = 0; i < m_numbers.size(); i++)
        {
            _showImage = m_numbers[i].clone();
            string _show_name;
            _show_name = "Number";
            if (m_debugType == 0)
                _show_name.append("_left");
            else
                _show_name.append("_right");
            _show_name.append(to_string(i));

            resize(_showImage, _showImage, Size(480, 320));
            namedWindow(_show_name, CV_WINDOW_FREERATIO);
            imshow(_show_name, _showImage);
        }
    }

    void ArmorDebug::deBug(int debugType)
    {
        m_debugType = debugType;

        showBinary();
        //二值图

        showLights();
        //灯条

        showLightRoi();
        //灯条ROI，不做筛选，有几个框选几个

        showNumberRoi();
        //数字识别区域

        showArmorsBefore();
        //展示所有符合几何特征的装甲板

        showArmors();
        //符合条件的装甲板

        showNumber();
        //数字识别的图片

        //showVerticesDistance();
        //标出角点，判断测距

        initSet();
        //初始化
    }

}
