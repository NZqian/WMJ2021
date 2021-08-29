/**
 * @file    ArmorTrigger.cpp
 * @author  liangyufeng@mail.nwpu.edu.cn
 * @version V1.0.0
 * @date    2-August-2021
 * @brief   This file implements all the functions in the energy detection library.
 */
#include "../include/ArmorTrigger.hpp"

namespace wmj
{
    ArmorTrigger::ArmorTrigger()
    {
        m_binocular = wmj::Binocular(6.0);
        m_rune_detector_left.setRuneParam(RUNE_CFG);
        m_rune_detector_left.setCameraParam(BINO_CFG, 0);
        m_rune_detector_right.setRuneParam(RUNE_CFG);
        m_rune_detector_right.setCameraParam(BINO_CFG, 1);
        cv::FileStorage config(RUNE_CFG, cv::FileStorage::READ);
        config["Rune"]["color"] >> this->m_rune_color;
        config["Rune"]["auto_shoot"] >> this->m_auto_shoot;
        config["Rune"]["pitch_off"] >> this->m_pitch_off;
        config.release();
    }

    void RuneDetector::setCameraParam(const string &file_path, bool location)
    {
        cv::FileStorage camera_config(file_path, cv::FileStorage::READ);
        if (location == 0)
        {
            camera_config["M1"] >> this->m_camera_matrix;
            camera_config["D1"] >> this->m_distortion_coefficients;
        }
        else if (location == 1)
        {
            camera_config["M2"] >> this->m_camera_matrix;
            camera_config["D2"] >> this->m_distortion_coefficients;
        }
        camera_config.release();
    }

    void RuneDetector::setRuneParam(const string &file_path)
    {
        cv::FileStorage rune_config(file_path, cv::FileStorage::READ);
        rune_config["Rune"]["color"] >> this->m_rune_color;
        rune_config["Rune"]["debug"] >> this->m_debug;
        rune_config["Rune"]["index"] >> this->m_index;

        if (m_rune_color == 0)
        {
            rune_config["Rune"]["threshold_color_blue"] >> this->m_threshold_color;
            rune_config["Rune"]["threshold_gray_blue"] >> this->m_threshold_gray;
        }
        else
        {
            rune_config["Rune"]["threshold_color_red"] >> this->m_threshold_color;
            rune_config["Rune"]["threshold_gray_red"] >> this->m_threshold_gray;
        }

        rune_config["Rune"]["small_contour_area_max"] >> this->m_small_contour_area_max;
        rune_config["Rune"]["small_contour_area_min"] >> this->m_small_contour_area_min;
        rune_config["Rune"]["big_contour_area_max"] >> this->m_big_contour_area_max;
        rune_config["Rune"]["big_contour_area_min"] >> this->m_big_contour_area_min;
        rune_config["Rune"]["center_area_max"] >> this->m_center_area_max;
        rune_config["Rune"]["center_area_min"] >> this->m_center_area_min;
        rune_config.release();
    }

    void ArmorTrigger::threadLeft()
    {
        m_rune_detector_left.findTargetPoint(m_frame[0], m_state, m_left_center_point, m_left_armor_point, m_left_predict_angle);
    }

    void ArmorTrigger::threadRight()
    {
        m_rune_detector_right.findTargetPoint(m_frame[1], m_state, m_right_center_point, m_right_armor_point, m_right_predict_angle);
    }

    cv::Point3f ArmorTrigger::findTarget(std::vector<MatWithTime> &frame, int state)
    {
        m_state = state;
        m_frame = frame;
        if (m_frame.size() == 2)
        {
            if (m_frame[0].m_orientation == "right" && m_frame[1].m_orientation == "left")
            {
                swap(m_frame[0], m_frame[1]);
            }
            //双目线程
            std::thread thread_left(&ArmorTrigger::threadLeft, this);
            std::thread thread_right(&ArmorTrigger::threadRight, this);
            thread_left.join();
            thread_right.join();

            if (m_left_armor_point.x != -1 && m_right_armor_point.x != -1 && m_left_center_point.x != -1 && m_right_center_point.x != -1)
            {
                if (state == 1)
                {
                    if (m_rune_detector_left.m_rot_direction != m_rune_detector_right.m_rot_direction)
                        return cv::Point3f(-1, -1, -1);
                }
                else if (state == 2)
                {
                    if (!m_rune_detector_left.m_sin_flag || !m_rune_detector_right.m_sin_flag)
                        return cv::Point3f(-1, -1, -1);
                }
                //解算装甲板中心和圆心的三维坐标
                m_armor_position = m_binocular.getPosition(m_left_armor_point, m_right_armor_point, 0) / 1000;
                m_center_position = m_binocular.getPosition(m_left_center_point, m_right_center_point, 0) / 1000;

                return m_armor_position;
            }
            else
            {
                return cv::Point3f(-1, -1, -1);
            }
        }
        else
        {
            std::cout << _warning("图像数量不为二") << std::endl;
            return cv::Point3f(-1, -1, -1);
        }
    }

    void ArmorTrigger::setPredictTime(double predict_time)
    {
        m_rune_detector_left.m_predict_time = predict_time;
        m_rune_detector_right.m_predict_time = predict_time;
    }

    void ArmorTrigger::recomputePredictAngle(double predict_time)
    {
        m_rune_detector_left.m_predict_time = predict_time;
        m_rune_detector_right.m_predict_time = predict_time;
        m_rune_detector_left.predict();
        m_rune_detector_right.predict();
        m_left_predict_angle = m_rune_detector_left.m_predict_angle;
        m_right_predict_angle = m_rune_detector_right.m_predict_angle;
    }

    void ArmorTrigger::setImshowIndex(string index)
    {
        m_rune_detector_left.m_index = index;
        m_rune_detector_right.m_index = index;
    }

    void ArmorTrigger::setRuneColor(bool color)
    {
        m_rune_detector_left.m_rune_color = color ? m_rune_color : !m_rune_color;
        m_rune_detector_right.m_rune_color = color ? m_rune_color : !m_rune_color;
    }

    void RuneDetector::imgProcess(cv::Mat &src_img)
    {
        cv::Mat img;
        m_img_rect = cv::Rect(cv::Point(0, 0), src_img.size());

        if (m_isDetected == true)
        {
            getImageROI(m_final_target);
            m_roi_rect_last = m_roi_rect + m_roi_rect_last.tl();
            m_roi_rect_last = m_roi_rect_last & m_img_rect;
            img = src_img(m_roi_rect_last);
        }
        else
        {
            m_roi_rect_last = m_img_rect;
            img = src_img;
        }
        //获取亮度二值图
        cv::cvtColor(img, m_binary_img, COLOR_BGR2GRAY);
        cv::GaussianBlur(m_binary_img, m_binary_img, cv::Size(3, 3), 0);
        cv::threshold(m_binary_img, m_binary_img, m_threshold_gray, 255, CV_THRESH_BINARY);
        if (m_debug && m_lable == m_index)
            cv::imshow("binary" + m_lable, m_binary_img);
        //获取颜色二值图
        std::vector<cv::Mat> channels;
        cv::split(img, channels);
        if (m_rune_color)
        {
            m_binary_color_img = channels.at(0) - channels.at(2);
        }
        else
        {
            m_binary_color_img = channels.at(2) - channels.at(0);
        }
        cv::threshold(m_binary_color_img, m_binary_color_img, m_threshold_color, 255, CV_THRESH_BINARY);

        cv::Mat element2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
        cv::Mat element3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::Mat element4 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(4, 4));
        cv::Mat element5 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::Mat element6 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(6, 6));

        dilate(m_binary_color_img, m_binary_color_img, element2);
        if (m_debug && m_lable == m_index)
            cv::imshow("color" + m_lable, m_binary_color_img);
        //得到最终处理后的图像
        cv::bitwise_and(m_binary_color_img, m_binary_img, m_result_img);
        cv::morphologyEx(m_result_img, m_result_img, MORPH_CLOSE, element5);
        if (m_debug && m_lable == m_index)
            cv::imshow("result" + m_lable, m_result_img);
    }

    void RuneDetector::findTargetPoint(wmj::MatWithTime &frame, int state, cv::Point2f &center_point, cv::Point2f &armor_point, float &predict_angle)
    {
        m_lable = frame.m_orientation;
        m_time = frame.m_time_stamp;
        m_src_img = frame.m_img;
        m_rotation_state = state;
        vector<Vane> targets;

        imgProcess(m_src_img);

        m_contours.clear();
        m_hierarchy.clear();
        findContours(m_result_img, m_contours, m_hierarchy, CV_RETR_CCOMP, CHAIN_APPROX_NONE);
        //对轮廓进行筛选
        for (size_t i = 0; i < m_contours.size(); i++)
        {
            if (m_debug)
                cout << "---------------开始---------------" << endl;
            if (m_hierarchy[i][3] < 0 || m_contours[i].size() < 6 || m_contours[static_cast<uint>(m_hierarchy[i][3])].size() < 6)
                continue;

            double small_contour_area = contourArea(m_contours[i]);
            if (m_debug)
                cout << "small_contour_area:" << small_contour_area << endl;

            if (small_contour_area < m_small_contour_area_min || small_contour_area > m_small_contour_area_max)
                continue;

            RotatedRect small_rect = minAreaRect(m_contours[i]);

            float small_rect_size_ratio;
            if (small_rect.size.width > small_rect.size.height)
            {
                small_rect_size_ratio = small_rect.size.width / small_rect.size.height;
            }
            else
            {
                small_rect_size_ratio = small_rect.size.height / small_rect.size.width;
            }

            if (m_debug)
                cout << "small_rect_size_ratio:" << small_rect_size_ratio << endl;

            if (small_rect_size_ratio > 2.5f)
                continue;

            float small_contour_rect_ratio = small_contour_area / small_rect.size.area();

            if (m_debug)
                cout << "small_contour_rect_ratio:" << small_contour_rect_ratio << endl;

            if (small_contour_rect_ratio < 0.6f)
                continue;

            if (isResectable(small_rect))
            {
                cv::Mat armor_img = armorCut(small_rect);
                double armor_rate = judge(armor_img);

                if (m_debug)
                    cout << "armor_rate:" << armor_rate << endl;

                if (armor_rate > 0.35)
                    continue;
            }

            double big_contour_area = contourArea(m_contours[static_cast<uint>(m_hierarchy[i][3])]);
            if (m_debug)
                cout << "big_contour_area:" << big_contour_area << endl;
            if (big_contour_area < m_big_contour_area_min || big_contour_area > m_big_contour_area_max)
                continue;

            RotatedRect big_rect = minAreaRect(m_contours[static_cast<uint>(m_hierarchy[i][3])]);

            float big_rect_size_ratio;
            if (big_rect.size.width > big_rect.size.height)
            {
                big_rect_size_ratio = big_rect.size.width / big_rect.size.height;
            }
            else
            {
                big_rect_size_ratio = big_rect.size.height / big_rect.size.width;
            }

            if (m_debug)
                cout << "big_rect_size_ratio:" << big_rect_size_ratio << endl;

            if (big_rect_size_ratio > 3.0f || big_rect_size_ratio < 1.5f)
                continue;

            if (isResectable(big_rect))
            {
                cv::Mat fan_img = armorCut(big_rect);
                double fan_rate = judge(fan_img);

                if (m_debug)
                    cout << "fan_rate:" << fan_rate << endl;

                if (fan_rate < 0.1 || fan_rate > 0.7)
                    continue;
            }

            if (m_debug)
                cout << "big/small(rect_area):" << big_rect.size.area() / small_rect.size.area() << endl;
            if ((big_rect.size.area() / small_rect.size.area()) > 10 || (big_rect.size.area() / small_rect.size.area()) < 4)
                continue;

            if (m_debug)
                cout << "distance:" << getDistance(big_rect.center, small_rect.center) << ">" << max(big_rect.size.width, big_rect.size.height) / 4 << endl;

            if (getDistance(big_rect.center, small_rect.center) < (max(big_rect.size.width, big_rect.size.height) / 4))
                continue;

            if (m_debug)
            {
                cout << "---------------结束---------------" << endl;
                cout << _red("找到一个扇叶") << endl;
            }

            Vane vane;
            vane.m_small_rect = small_rect;
            vane.m_big_rect = big_rect;

            if (m_debug)
                cout << "big/samll(contour_area):" << big_contour_area / small_contour_area << endl;

            //判断是否激活
            if (small_contour_area * 12 > big_contour_area && small_contour_area * 5 <= big_contour_area)
            {
                vane.m_type = ACTION;
            }
            else if (small_contour_area * 5 > big_contour_area && small_contour_area * 2 < big_contour_area)
            {
                vane.m_type = INACTION;
                targets.push_back(vane);
            }
            else
            {
                vane.m_type = UNKOWN;
            }

            if (m_debug)
            {
                Point2f small_point_tmp[4];
                vane.m_small_rect.points(small_point_tmp);
                Point2f big_point_tmp[4];
                vane.m_big_rect.points(big_point_tmp);
                for (int k = 0; k < 4; k++)
                {
                    line(m_src_img, small_point_tmp[k] + cv::Point2f(m_roi_rect_last.tl()), small_point_tmp[(k + 1) % 4] + cv::Point2f(m_roi_rect_last.tl()), Scalar(255, 0, 0), 2);
                    line(m_src_img, big_point_tmp[k] + cv::Point2f(m_roi_rect_last.tl()), big_point_tmp[(k + 1) % 4] + cv::Point2f(m_roi_rect_last.tl()), Scalar(0, 0, 255), 2);
                }
                putText(m_src_img, to_string(vane.m_type), small_rect.center + cv::Point2f(m_roi_rect_last.tl()), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255));
            }
        }
        if (targets.size() != 1)
        {
            std::cout << _warning("扇叶不唯一") << std::endl;
            m_isDetected = false;
            armor_point = cv::Point2f(-1, -1);
            return;
        }
        m_isDetected = true;
        m_final_target = targets.at(0);
        m_final_target.m_time = m_time;
        m_armor_point = m_final_target.m_small_rect.center + cv::Point2f(m_roi_rect_last.tl());
        armor_point = m_armor_point;
        getAbsolutePosition();
        findCenter();
        if (m_center_isDetected)
        {
            center_point = m_center + cv::Point2f(m_roi_rect_last.tl());
            if (m_debug)
                circle(m_src_img, center_point, 5, cv::Scalar(255, 0, 0), -1);
        }
        else
        {
            std::cout << _warning("未识别到圆心") << std::endl;
            center_point = cv::Point2f(-1, -1);
            return;
        }

        calcAngle(m_final_target);
        getRotatedDirection(m_final_target.m_angle);
        getRotatedSpeed(m_final_target);
        predict();
        predict_angle = m_predict_angle;

        if (m_debug)
        {
            std::cout << _lightgreen(m_lable + "_roi_flag :") << m_isDetected << std::endl;
            cv::putText(m_src_img, "direction:" + to_string(m_rot_direction), Point2f(0, 50), FONT_HERSHEY_SIMPLEX, 2, Scalar(255, 0, 255));
            cv::putText(m_src_img, "state:" + to_string(m_rotation_state), Point2f(450, 50), FONT_HERSHEY_SIMPLEX, 2, Scalar(255, 0, 255));
            cv::putText(m_src_img, "sin:" + to_string(m_sin_flag), Point2f(800, 50), FONT_HERSHEY_SIMPLEX, 2, Scalar(255, 0, 255));
            cv::putText(m_src_img, "final_target", Point2f(10, -50) + m_final_target.m_small_rect.center + cv::Point2f(m_roi_rect_last.tl()), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255));
        }
    }

    void RuneDetector::calcAngle(Vane &vane)
    {
        if (m_center_isDetected)
        {
            double angle;
            angle = atan2(m_center.y - vane.m_small_rect.center.y, vane.m_small_rect.center.x - m_center.x) / CV_PI * 180;
            vane.m_angle = makeAngleRegular(angle);
        }
        else
        {
            if (vane.m_small_rect.size.width > vane.m_small_rect.size.height)
            {
                if ((vane.m_small_rect.center.y + vane.m_small_rect.center.x) < (vane.m_big_rect.center.y + vane.m_big_rect.center.x))
                    vane.m_angle = 90 - vane.m_small_rect.angle;
                if ((vane.m_small_rect.center.y + vane.m_small_rect.center.x) > (vane.m_big_rect.center.y + vane.m_big_rect.center.x))
                    vane.m_angle = 270 - vane.m_small_rect.angle;
            }
            if (vane.m_small_rect.size.width < vane.m_small_rect.size.height)
            {
                if ((vane.m_small_rect.center.y - vane.m_small_rect.center.x) < (vane.m_big_rect.center.y - vane.m_big_rect.center.x))
                    vane.m_angle = -vane.m_small_rect.angle;
                if ((vane.m_small_rect.center.y - vane.m_small_rect.center.x) > (vane.m_big_rect.center.y - vane.m_big_rect.center.x))
                    vane.m_angle = 180 - vane.m_small_rect.angle;
            }
        }
        if (m_debug)
            std::cout << _lightcyan(m_lable + "_vane_angle :") << vane.m_angle << std::endl;
    }

    void RuneDetector::getRotatedDirection(float angle)
    {
        if (m_rotation_state == 0)
        {
            m_rot_direction = 0;
            return;
        }
        //采集一定数量的角度数据进行判断
        if (m_angles.empty())
        {
            m_angles.push_back(angle);
        }
        else if (m_angles.size() < 30)
        {
            if (fabs(angle - m_angles[m_angles.size() - 1]) < 20)
            {
                m_angles.push_back(angle);
            }
            else
            {
                m_angles.clear();
            }
        }
        if (m_angles.size() == 30)
        {
            int stop = 0, clockwise = 0, counter_clockwise = 0;
            for (size_t i = 0; i < 15; i++)
            {
                if (fabs(m_angles[i + 15] - m_angles[i]) < 3.5)
                    stop++;
                else if ((m_angles[i + 15] - m_angles[i]) > 0)
                    counter_clockwise++;
                else
                    clockwise++;
            }
            m_angles.clear();
            if (stop > counter_clockwise && stop > clockwise)
            {
                m_rot_direction = 0;
            }
            else if (clockwise > counter_clockwise)
            {
                m_rot_direction = -1;
            }
            else
            {
                m_rot_direction = 1;
            }
        }
    }

    void RuneDetector::predict()
    {
        if (m_rotation_state == 0)
        {
            m_predict_angle = 0;
        }
        else if (m_rotation_state == 1)
        {
            if (m_rot_direction == 1)
                m_predict_angle = 60 * m_predict_time;
            else if (m_rot_direction == -1)
                m_predict_angle = -60 * m_predict_time;
            else if (m_rot_direction == 0)
                m_predict_angle = 0;
        }
        else if (m_rotation_state == 2)
        {
            if (m_debug)
                std::cout << m_lable << "_sin_flag :" << m_sin_flag << std::endl;
            if (m_sin_flag)
            {
                if (m_rot_direction == 1)
                    m_predict_angle = (-0.785 / 1.884 * cos(1.884 * (m_time - m_time_param + m_predict_time + m_function_param)) + 1.305 * m_predict_time + 0.785 / 1.884 * cos(1.884 * (m_time - m_time_param + m_function_param))) / CV_PI * 180;
                else if (m_rot_direction == -1)
                    m_predict_angle = (0.785 / 1.884 * cos(1.884 * (m_time - m_time_param + m_predict_time + m_function_param)) - 1.305 * m_predict_time - 0.785 / 1.884 * cos(1.884 * (m_time - m_time_param + m_function_param))) / CV_PI * 180;
                else if (m_rot_direction == 0)
                    m_predict_angle = 0;
            }
            else
                m_predict_angle = 0;
        }

        if (m_debug)
            cout << _cyan(m_lable + "_predict_angle :") << m_predict_angle << endl;
    }

    void RuneDetector::getImageROI(Vane &vane)
    {
        RotatedRect roi_rect_tmp = vane.m_big_rect;
        float k = 2.2;
        Point2f center = (k + 1) * vane.m_big_rect.center - k * vane.m_small_rect.center;
        roi_rect_tmp.center = center;
        roi_rect_tmp.size.width = 3 * max(vane.m_big_rect.size.width, vane.m_big_rect.size.height);
        roi_rect_tmp.size.height = 3 * max(vane.m_big_rect.size.width, vane.m_big_rect.size.height);

        Point2f points_tmp[4];
        roi_rect_tmp.points(points_tmp);
        vector<Point2f> points;
        for (int i = 0; i < 4; i++)
        {
            points.push_back(points_tmp[i]);
        }
        m_roi_rect = boundingRect(points);
    }

    void RuneDetector::getRotatedSpeed(Vane &vane)
    {
        if (m_rotation_state == 0 || m_rotation_state == 1)
            return;
        //验证拟合函数误差是否在允许范围内
        if (m_sin_flag)
        {
            if (m_corrects.size() == 0)
            {
                m_corrects.push_back(vane);
            }
            else if (m_corrects.size() < 10)
            {
                if (fabs(vane.m_angle - m_corrects[m_corrects.size() - 1].m_angle) < 20 || fabs(vane.m_angle - m_corrects[m_corrects.size() - 1].m_angle) > 340)
                {
                    m_corrects.push_back(vane);
                }
                else
                {
                    m_corrects.clear();
                }
            }
            if (m_corrects.size() == 10)
            {
                double angle_diff_fit;
                if (m_rot_direction == 1)
                {
                    angle_diff_fit = (-0.785 / 1.884 * cos(1.884 * (m_corrects[9].m_time - m_time_param + m_function_param)) + 0.785 / 1.884 * cos(1.884 * (m_corrects[0].m_time - m_time_param + m_function_param)) + 1.305 * (m_corrects[9].m_time - m_corrects[0].m_time)) * 180 / CV_PI;
                }
                else if (m_rot_direction == -1)
                {
                    angle_diff_fit = (0.785 / 1.884 * cos(1.884 * (m_corrects[9].m_time - m_time_param + m_function_param)) - 0.785 / 1.884 * cos(1.884 * (m_corrects[0].m_time - m_time_param + m_function_param)) - 1.305 * (m_corrects[9].m_time - m_corrects[0].m_time)) * 180 / CV_PI;
                }

                double angle_diff_real = m_corrects[9].m_angle - m_corrects[0].m_angle;
                if (angle_diff_real < -180)
                    angle_diff_real += 360;
                else if (angle_diff_real > 180)
                    angle_diff_real -= 360;

                double angle_diff_diff = fabs(angle_diff_fit - angle_diff_real);

                if (m_debug)
                    cout << "angle_diff_diff:" << angle_diff_diff << endl;

                m_corrects.clear();
                //连续三帧不满足要求即重新拟合
                if (m_sin_false == false && angle_diff_diff > 3)
                {
                    m_sin_false = true;
                }
                else if (m_sin_false == true && angle_diff_diff > 3)
                {
                    m_count++;
                }
                else
                {
                    m_count = 0;
                    m_sin_false = false;
                }
                if (m_count == 3)
                {
                    m_sin_flag = false;
                    m_sin_false = false;
                    m_count = 0;
                }
            }
        }
        //采集数据并进行正弦拟合
        if (!m_sin_flag)
        {
            if (m_vanes.size() == 0)
            {
                m_vanes.push_back(vane);
            }
            else if (m_vanes.size() < m_sample_size)
            {
                if (fabs(vane.m_angle - m_vanes[m_vanes.size() - 1].m_angle) < 20 || fabs(vane.m_angle - m_vanes[m_vanes.size() - 1].m_angle) > 340)
                {
                    m_vanes.push_back(vane);
                }
                else
                {
                    m_vanes.clear();
                }
            }
            //正弦拟合
            if (m_vanes.size() == m_sample_size)
            {
                Mat angle(1, m_sample_size, CV_64FC1);
                Mat time(1, m_sample_size, CV_64FC1);
                Mat difference(1, m_sample_size, CV_64FC1);
                cv::Scalar mean_tmp;
                cv::Scalar stddev_tmp;
                double stddev = 1000.0;
                time.at<double>(0) = 0;
                angle.at<double>(0) = m_vanes[0].m_angle;
                //数据预处理，使角度连续
                for (size_t i = 1; i < m_sample_size; i++)
                {
                    time.at<double>(i) = m_vanes[i].m_time - m_vanes[0].m_time;
                    if ((m_vanes[i].m_angle - angle.at<double>(i - 1)) > 340)
                    {
                        angle.at<double>(i) = m_vanes[i].m_angle - 360;
                    }
                    else if ((m_vanes[i].m_angle - angle.at<double>(i - 1)) < -340)
                    {
                        if (stddev_tmp[0] < stddev)
                            angle.at<double>(i) = m_vanes[i].m_angle + 360;
                    }
                    else
                    {
                        angle.at<double>(i) = m_vanes[i].m_angle;
                    }
                }
                //通过方差最小判断最优拟合
                for (float i = 0; i < CV_PI / 0.942; i += 0.01)
                {
                    for (size_t j = 0; j < m_sample_size; j++)
                    {
                        if (m_rot_direction == 1)
                            difference.at<double>(j) = (-0.785 / 1.884 * cos(1.884 * (time.at<double>(j) + i)) + 1.305 * time.at<double>(j)) * 180 / CV_PI - angle.at<double>(j);
                        if (m_rot_direction == -1)
                            difference.at<double>(j) = (0.785 / 1.884 * cos(1.884 * (time.at<double>(j) + i)) - 1.305 * time.at<double>(j)) * 180 / CV_PI - angle.at<double>(j);
                    }
                    meanStdDev(difference, mean_tmp, stddev_tmp);
                    if (stddev_tmp[0] < stddev)
                    {
                        m_function_param = i;
                        stddev = stddev_tmp[0];
                    }
                }
                m_time_param = m_vanes[0].m_time;
                m_angle_param = m_vanes[0].m_angle / 180 * CV_PI;

                if (m_debug)
                {
                    cout << '\n'
                         << "function_param:" << m_function_param << " time_param:" << m_time_param << " angle_param:" << m_angle_param << endl;
                    cout << "duration:" << time.at<double>(m_sample_size - 1) << endl;
                }
                m_sin_flag = true;
                m_vanes.clear();
            }
        }
    }

    void RuneDetector::findCenter()
    {
        //根据未激活扇叶和装甲板的位置关系进行目标区域选择
        RotatedRect center_rect = m_final_target.m_big_rect;
        float k = 2.2;
        Point2f center_tmp = (k + 1) * m_final_target.m_big_rect.center - k * m_final_target.m_small_rect.center;
        center_rect.center = center_tmp;
        center_rect.size.width = max(m_final_target.m_big_rect.size.width, m_final_target.m_big_rect.size.height) / 3 * 2;
        center_rect.size.height = max(m_final_target.m_big_rect.size.width, m_final_target.m_big_rect.size.height) / 3 * 2;

        Point2f points_tmp[4];
        center_rect.points(points_tmp);
        if (m_debug)
        {
            for (int k = 0; k < 4; k++)
            {
                line(m_src_img, points_tmp[k] + cv::Point2f(m_roi_rect_last.tl()), points_tmp[(k + 1) % 4] + cv::Point2f(m_roi_rect_last.tl()), Scalar(0, 255, 255), 2);
            }
        }
        vector<Point2f> center_boundary;
        for (size_t i = 0; i < 4; i++)
        {
            center_boundary.push_back(points_tmp[i]);
        }

        vector<Point2f> centers;

        for (size_t i = 0; i < m_contours.size(); i++)
        {
            if (contourArea(m_contours[i]) > m_center_area_max || contourArea(m_contours[i]) < m_center_area_min)
                continue;
            Point2f center_tmp;
            float radius_tmp;
            minEnclosingCircle(m_contours[i], center_tmp, radius_tmp);
            if (pointPolygonTest(center_boundary, center_tmp, false) != 1)
                continue;
            centers.push_back(center_tmp);
        }
        if (centers.size() != 1)
        {
            m_center_isDetected = false;
            m_center = cv::Point2f(-1, -1);
            return;
        }
        m_center = centers.at(0);
        m_center_isDetected = true;
    }

    cv::Mat RuneDetector::armorCut(RotatedRect &rect)
    {
        //将图像旋转后切割
        cv::Mat rot_mat = getRotationMatrix2D(rect.center, rect.angle, 1.0);
        cv::Mat rot_image;
        cv::Size dst_size(m_binary_img.size());
        warpAffine(m_binary_img, rot_image, rot_mat, dst_size);
        return rot_image(cv::Rect(rect.center.x - rect.size.width * 0.5, rect.center.y - rect.size.height * 0.5, rect.size.width, rect.size.height));
    }

    double RuneDetector::judge(cv::Mat &src)
    {
        cv::Mat image = src;
        int colornum = 0;

        int rows = image.rows;
        int cols = image.cols;

        if (image.isContinuous())
        {
            rows = 1;
            cols = image.cols * image.rows;
        }
        for (size_t i = 0; i < rows; i++)
        {
            const uchar *inData = image.ptr<uchar>(i);
            for (size_t j = 0; j < cols; j++)
            {
                if (*inData > 200)
                    colornum++;
                inData++;
            }
        }
        return (double)colornum / ((double)image.cols * (double)image.rows);
    }

    cv::Point3f RuneDetector::getAbsolutePosition()
    {
        float width = m_final_target.m_small_rect.size.width > m_final_target.m_small_rect.size.height ? m_final_target.m_small_rect.size.width : m_final_target.m_small_rect.size.height;
        float height = m_final_target.m_small_rect.size.width < m_final_target.m_small_rect.size.height ? m_final_target.m_small_rect.size.width : m_final_target.m_small_rect.size.height;
        vector<Point2f> image_points;
        image_points.push_back(cv::Point2f(m_armor_point.x + m_roi_rect_last.tl().x - width / 2, m_armor_point.y + m_roi_rect_last.tl().y - height / 2));
        image_points.push_back(cv::Point2f(m_armor_point.x + m_roi_rect_last.tl().x - width / 2, m_armor_point.y + m_roi_rect_last.tl().y + height / 2));
        image_points.push_back(cv::Point2f(m_armor_point.x + m_roi_rect_last.tl().x + width / 2, m_armor_point.y + m_roi_rect_last.tl().y + height / 2));
        image_points.push_back(cv::Point2f(m_armor_point.x + m_roi_rect_last.tl().x + width / 2, m_armor_point.y + m_roi_rect_last.tl().y - height / 2));
        vector<Point3f> object_points;
        float width_obj = m_armor_width;
        float height_obj = width_obj * height / width;
        object_points.push_back(cv::Point3f(-width_obj / 2.0, -height_obj / 2.0, 0.0));
        object_points.push_back(cv::Point3f(-width_obj / 2.0, height_obj / 2.0, 0.0));
        object_points.push_back(cv::Point3f(width_obj / 2.0, height_obj / 2.0, 0.0));
        object_points.push_back(cv::Point3f(width_obj / 2.0, -height_obj / 2.0, 0.0));
        cv::Mat rvec, tvec;
        bool ret = cv::solvePnP(cv::Mat(object_points), cv::Mat(image_points), m_camera_matrix, m_distortion_coefficients, rvec, tvec);
        cv::Point3f position;
        if (ret)
            position = cv::Point3f(tvec.at<double>(2, 0), -tvec.at<double>(0, 0), -tvec.at<double>(1, 0));
        else
            position = cv::Point3f(-1, -1, -1);
        if (m_debug)
            std::cout << _blue(m_lable + "_single_position :") << position << std::endl;
        return position;
    }

    double RuneDetector::getDistance(cv::Point2f &point_a, cv::Point2f &a_point_b)
    {
        return sqrtf(powf((point_a.x - a_point_b.x), 2) + powf((point_a.y - a_point_b.y), 2));
    }

    bool RuneDetector::isResectable(RotatedRect &rect)
    {
        if (rect.center.x - rect.size.width * 0.5 > 0 && rect.center.y - rect.size.height * 0.5 > 0 && rect.center.x + rect.size.width * 0.5 < m_result_img.cols && rect.center.y + rect.size.height * 0.5 < m_result_img.rows)
            return true;
        else
            return false;
    }

    float RuneDetector::makeAngleRegular(float angle)
    {
        float angle_tmp;
        angle_tmp = fmod(angle, 360);

        if (angle_tmp < 0)
            angle_tmp += 360;

        return angle_tmp;
    }
} // namespace wmj