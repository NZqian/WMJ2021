#include "../include/AngleAimer.hpp"

namespace wmj
{
    AngleAimer::AngleAimer() : Aimer()
    {
        setParam();
        m_abs_angle_filter      = std::make_shared<wmj::KalmanFilter>(1e-3, 1e-4, 45 * PI / 180);
        m_palstance_filter      = std::make_shared<wmj::KalmanFilter>(1e-4, 1e-2, 0.15 * PI);
        m_height_filter[0]      = std::make_shared<wmj::KalmanFilter>(1e-5, 1e-3, 0.3);
        m_height_filter[1]      = std::make_shared<wmj::KalmanFilter>(1e-5, 1e-3, 0.3);
        m_radius_filter[0]      = std::make_shared<wmj::KalmanFilter>(1e-5, 1e-3, 1.0);
        m_radius_filter[1]      = std::make_shared<wmj::KalmanFilter>(1e-5, 1e-3, 1.0);
        m_center_predicter      = std::make_shared<wmj::MotionPredict>();
        if (m_debug)
            std::cout << "[Top] Filter init" << std::endl;
    }

    void AngleAimer::setParam()
    {
        m_file = open("top.csv", O_WRONLY | O_CREAT, 0777);
        cv::FileStorage fs(TOP_CFG, cv::FileStorage::READ);
        fs["angle"]["use_fluent_angle"]         >> m_use_fluent_angle;
        fs["angle"]["time_off"]                 >> m_time_off;
        fs["angle"]["abs_angle_off"]            >> m_abs_angle_off;
        fs["angle"]["angle_shoot_off"]          >> m_angle_shoot_off;

        fs["angle"]["time_switch"]              >> m_time_switch;
        fs["angle"]["angle_diff_shift"]         >> m_angle_diff_shift;
        fs["angle"]["angle_diff_static"]        >> m_angle_diff_static;
        fs["angle"]["angle_predict_max"]        >> m_angle_predict_max;
        fs["angle"]["angle_follow_shoot_max"]   >> m_angle_follow_shoot_max;
        fs["angle"]["angle_focus_shoot_max"]    >> m_angle_focus_shoot_max;
        fs["angle"]["palstance_min"]            >> m_palstance_min;
        fs["angle"]["palstance_diff_ratio_max"] >> m_palstance_diff_ratio_max;
        fs["angle"]["center_diff_ratio_max"]    >> m_center_diff_ratio_max;
        fs["angle"]["infantry_radius_min"]      >> m_infantry_radius_min;
        fs["angle"]["infantry_radius_max"]      >> m_infantry_radius_max;
        fs["angle"]["hero_radius_min"]          >> m_hero_radius_min;
        fs["angle"]["hero_radius_max"]          >> m_hero_radius_max;
        m_abs_angle_off             *= PI / 180;
        m_angle_shoot_off           *= PI / 180;
        m_angle_diff_shift          *= PI / 180;
        m_angle_diff_static         *= PI / 180;
        m_angle_predict_max         *= PI / 180;
        m_angle_follow_shoot_max    *= PI / 180;
        m_angle_focus_shoot_max     *= PI / 180;
        m_palstance_min             *= PI;
        fs.release();
        if (m_debug)
            std::cout << "[Top] Angle param set" << std::endl;

        m_lost_cnt = 0;
        initData();
    }

    void AngleAimer::initData()
    {
        m_shoot_flag = false;
        m_cur_pair_num = 0;
        m_cycle_cnt = 0;
        m_palstance = 0;
        m_armor_height[0] = 0, m_armor_height[1] = 0;
        m_radiuses[0] = m_radius, m_radiuses[1] = m_radius;
        m_approx_center = cv::Point2d(0, 0);
        if (m_debug)
            std::cout << "[Top] Data init" << std::endl;
    }

    void AngleAimer::buildModel(Armor &armor)
    {
        armor.m_time_seq = wmj::now();

        Armor abs_armor = getAbsArmor(armor);
        if (!m_armor_seq.empty())
        {
            double abs_angle_diff = abs_armor.m_yaw_angle - m_abs_armor_seq.back().m_yaw_angle;
            if (m_debug)
                std::cout << "[Top] abs_angle_diff " << R2D(abs_angle_diff) << std::endl;
            // 若目标装甲板转换，将上一周期作为样本进行建模
            if (abs(abs_angle_diff) > m_angle_diff_shift)
            {
                if (m_debug)
                {
                    std::cout << "[Top] vec_size: " << m_armor_seq.size() << std::endl;
                    std::cout << "[Top] Changed to another armor !" << std::endl;
                }

                if (m_armor_seq.size() > m_min_vec_size)
                {
                    // 计算模型数据
                    getPalstance();
                    getHeight();
                    getCircle();
                    // 切换装甲板组
                    m_cur_pair_num = (m_cur_pair_num + 1) % 2;
                    if (m_debug)
                        std::cout << "[Top] Model update" << std::endl;

                    // 达到陀螺速度认为模型建立成功
                    if(abs(m_palstance) > m_palstance_min)
                    {
                        if (m_debug)
                            std::cout << "[Top] Model confirmed" << std::endl;
                        m_model_ready = true;
                        if (m_cycle_cnt < 2)
                        {
                            m_cycle_cnt++;
                        }
                    }
                    else
                    {
                        m_model_ready = false;
                        m_cycle_cnt = 0;
                    }
                }
                m_armor_seq.clear();
                m_abs_armor_seq.clear();
                m_pose_seq.clear();
                m_lost_cnt = 0;
            }
            // 未开启小陀螺
            else if (abs(abs_angle_diff) < m_angle_diff_static)
            {
                if (m_debug)
                    std::cout << "[Top] Not spinning " << std::endl;
                m_lost_cnt += 1;
                // 丢失模型
                if (m_lost_cnt > 15)
                {
                    if (m_debug)
                        std::cout << "[Top] Model clear" << std::endl;
                    m_armor_seq.clear();
                    m_abs_armor_seq.clear();
                    m_pose_seq.clear();
                    m_model_ready = false;
                    m_lost_cnt = 0;
                    initData();
                }
            }
        }
        m_armor_seq.emplace_back(armor);
        m_abs_armor_seq.emplace_back(abs_armor);
        m_pose_seq.emplace_back(m_cur_pose);
    }

    Armor AngleAimer::getAbsArmor(Armor& armor)
    {
        Armor abs_armor = armor;
        abs_armor.m_position = m_angle_solver->cam2abs(armor.m_position, m_cur_pose);

        // 云台系坐标
        cv::Point3f gun_position = m_angle_solver->cam2gun(armor.m_position);
        // 相对角度
        double alpha = armor.m_yaw_angle;
        // 装甲板在相机系的偏转角
        double omega = atan(gun_position.y / sqrt(pow(gun_position.x, 2) + pow(gun_position.z, 2)));
        // 装甲板到自己旋转中心和对方旋转中心的张角
        double theta = PI - (alpha - omega);
        // 己方旋转中心到装甲板的距离
        double d = sqrt(pow(gun_position.x, 2) + pow(gun_position.y, 2) + pow(gun_position.z, 2));
        // 己方旋转中心到对方旋转中心的距离
        double D = sqrt(pow(d, 2) + pow(m_radiuses[m_cur_pair_num], 2) - 2 * d * m_radiuses[m_cur_pair_num] * cos(theta));
        // 对方旋转中心到装甲板和己方旋转中心的张角，即装甲板绝对角度
        double beta = asin((d / D) * sin(theta));
        // 根据上一个装甲板信息修正当前角度
        double abs_cur_angle;
        if (m_use_fluent_angle &&
            !m_armor_seq.empty() && m_palstance_stable)
        {
            double abs_cur_angle_from_last = m_abs_last_armor.m_yaw_angle + (armor.m_time_seq - m_abs_last_armor.m_time_seq) * m_palstance;
            if (abs_cur_angle_from_last * beta < 0)
            {
                if (abs_cur_angle_from_last > D2R(30))
                {
                    abs_cur_angle_from_last -= PI /2;
                }
                else if (abs_cur_angle_from_last < D2R(-30))
                {
                    abs_cur_angle_from_last += PI /2;
                }
            }
            abs_cur_angle = abs_cur_angle_from_last * 0.5 + beta * 0.5;
            if (m_debug)
            {
                std::cout << "fix_angle_calc: " << R2D(m_abs_last_armor.m_yaw_angle) << " + (" << armor.m_time_seq - m_abs_last_armor.m_time_seq << ") * " << m_palstance << std::endl;
                std::cout << "fix_angle: " << R2D(abs_cur_angle_from_last) << " " << R2D(beta) << " " << R2D(abs_cur_angle) << std::endl;
            }
        }
        else
        {
            abs_cur_angle = beta;
        }
        // 滤波角度
        abs_armor.m_yaw_angle = m_abs_angle_filter->predict(abs_cur_angle);
        if (m_debug)
        {
            std::cout << "[Top] abs_angle_calc: " << R2D(theta) << " " << d << " " << D << " " << R2D(beta) << std::endl;
            std::cout << "[Top] abs_angle: " << R2D(armor.m_yaw_angle) << "\t  " << R2D(beta) << "\t" << R2D(abs_armor.m_yaw_angle) << std::endl;
        }

        m_abs_last_armor.m_yaw_angle = abs_armor.m_yaw_angle;
        m_abs_last_armor.m_time_seq = abs_armor.m_time_seq;
        return abs_armor;
    }

    void AngleAimer::getPalstance()
    {
        // 求角速度，最小二乘法
        double palstance;
        // 拟合直线公式 y = k * x + b ，其中 y 为 angle，x 为 time，k 为 palstance
        double avg_x  = 0;
        double avg_x2 = 0;
        double avg_y  = 0;
        double avg_xy = 0;
        double time_first = m_armor_seq.front().m_time_seq;
        for (int i = 0; i < m_armor_seq.size(); i++)
        {
            avg_x  += m_armor_seq[i].m_time_seq - time_first;
            avg_x2 += pow(m_armor_seq[i].m_time_seq - time_first, 2);
            avg_y  += m_abs_armor_seq[i].m_yaw_angle;
            avg_xy += (m_armor_seq[i].m_time_seq - time_first) * m_abs_armor_seq[i].m_yaw_angle;
        }
        avg_x  /= m_armor_seq.size();
        avg_x2 /= m_armor_seq.size();
        avg_y  /= m_armor_seq.size();
        avg_xy /= m_armor_seq.size();
        palstance = (avg_xy - avg_x * avg_y) / (avg_x2 - pow(avg_x, 2));
        if (m_debug)
            std::cout << "[Top] calc_palstance: " << avg_x << " " << avg_x2 << " " << avg_y << " " << avg_xy << "->" << palstance << std::endl;
        // 加权
        if (abs((m_palstance - palstance) / m_palstance) < m_palstance_diff_ratio_max)
        {
            m_palstance_stable = true;
        }
        else
        {
            m_palstance_stable = false;
        }
        if (m_cycle_cnt >= 2)
        {
            m_palstance = m_palstance_filter->predict(palstance);
        }
        else
        {
            m_palstance = palstance;
        }
        m_rotate_direction = m_palstance == 0 ? 0 : abs(m_palstance) / m_palstance;
        if (m_debug)
            std::cout << "[Top] palstance: " << palstance << "->" << m_palstance << " " << m_rotate_direction << std::endl;
    }

    void AngleAimer::getHeight()
    {
        // 求平均高度
        double height = 0;
        for (int i = 0; i < m_armor_seq.size(); i++)
        {
            m_armor_height[m_cur_pair_num] = m_height_filter[m_cur_pair_num]->predict(m_abs_armor_seq[i].m_position.z);
        }
        if (m_armor_height[(m_cur_pair_num + 1) % 2] == 0)
        {
            m_armor_height[(m_cur_pair_num + 1) % 2] = m_armor_height[m_cur_pair_num];
        }
        if (m_debug)
            std::cout << "[Top] height: " << height << std::endl;
    }

    void AngleAimer::getCircle()
    {
        // 求半径和旋转中心
        double radius;
        cv::Point2d calc_center;

        for (int i = 0; i < m_abs_armor_seq.size() / 2; i++)
        {
            // 逐差法取两块装甲板
            Armor abs_armor1 = m_abs_armor_seq[i];
            Armor abs_armor2 = m_abs_armor_seq[i + m_abs_armor_seq.size() / 2];

            // 弦长
            double l = getDistance(abs_armor1.m_position - abs_armor2.m_position);
            // 张角
            double theta = abs(abs_armor1.m_yaw_angle - abs_armor2.m_yaw_angle);
            // 半径
            radius = l / (2 * sin(theta / 2));
            if ((m_armor_seq.back().m_armor_type == wmj::ARMOR_SMALL && radius > m_infantry_radius_min && radius < m_infantry_radius_max) ||
                (m_armor_seq.back().m_armor_type == wmj::ARMOR_LARGE && radius > m_hero_radius_min && radius < m_hero_radius_max))
            {
                m_radiuses[m_cur_pair_num] = m_radius_filter[m_cur_pair_num]->predict(radius);
            }
            if (m_debug)
                std::cout << "[Top] calc_radiuses: " << l << " " << theta << " " << radius << std::endl;
            // 弦心距
            double h = m_radiuses[m_cur_pair_num] * cos(theta / 2);
            // 弦心距旋转角
            double phi = atan2((abs_armor1.m_position.y - abs_armor2.m_position.y), (abs_armor1.m_position.x - abs_armor2.m_position.x)) - PI / 2;
            // 弦中点
            cv::Point2d M{(abs_armor1.m_position.x + abs_armor2.m_position.x) / 2,
                          (abs_armor1.m_position.y + abs_armor2.m_position.y) / 2};
            // 圆心
            cv::Point2d center1{M.x + h * cos(phi), M.y + h * sin(phi)};
            cv::Point2d center2{M.x - h * cos(phi), M.y - h * sin(phi)};
            // 认为较远的为圆心
            cv::Point2d center = getDistance(center1) > getDistance(center2) ?
                                 center1 : center2;
            calc_center = calc_center / (i + 1) * i + center / (i + 1);
        }

        // Ransac 算法求拟合圆
        // std::tie(radius, calc_center) = m_ransac_solver->fitCircle(m_abs_armor_seq);
        // m_radiuses[m_cur_pair_num] = m_radius_filter[m_cur_pair_num]->predict(radius);

        if (m_debug)
            std::cout << "[Top] radiuses: " << m_radiuses[0] << " " << m_radiuses[1] << std::endl;

        cv::Point3f approx_center3f = m_center_predicter->predict(cv::Point3f(calc_center.x, calc_center.y, 0));
        double center_diff = sqrt(pow(m_approx_center.x - approx_center3f.x, 2) + pow(m_approx_center.y - approx_center3f.y, 2));
        if (m_debug)
            std::cout << "[Top] spin_center_move: " << center_diff << std::endl;
        if (center_diff / getDistance(m_approx_center) < m_center_diff_ratio_max)
        {
            m_center_stable = true;
        }
        else
        {
            m_center_stable = false;
        }
        m_approx_center = cv::Point2f(approx_center3f.x, approx_center3f.y);
        if (m_debug)
            std::cout << "[Top] spin_center: " << calc_center << " " << m_approx_center << std::endl; 
    }

    Armor AngleAimer::getTargetArmor(Armor &armor)
    {
        // 当前装甲板
        Armor cur_armor = m_armor_seq.back();
        Armor abs_cur_armor = m_abs_armor_seq.back();
        // 目标装甲板
        Armor target_armor;
        Armor abs_target_armor;
        // 计算子弹飞行时间
        cv::Point3f gun_position = m_angle_solver->cam2gun(armor.m_position);
        double time_interval = getDistance(gun_position) / m_bullet_speed;
        // 当前装甲板当前角度
        double cur_angle = cur_armor.m_yaw_angle;
        double abs_cur_angle = abs_cur_armor.m_yaw_angle;
        // 当前装甲板到被击中时刻所处角度
        double abs_target_angle = abs_cur_angle + (time_interval + m_time_off) * m_palstance + m_abs_angle_off;
        if (m_debug)
        {
            std::cout << "[Top] predict_time: (" << getDistance(gun_position) << " / " << m_bullet_speed << " + " << m_time_off << ") * " << m_palstance << std::endl;
            std::cout << "[Top] predict: " << R2D(abs_cur_angle) << " " << R2D(abs_target_angle) << " " << time_interval << std::endl;
        }

        // 判断当前识别装甲板高低
        double cur_armor_height = abs_cur_armor.m_position.z;
        if (abs(cur_armor_height - m_armor_height[0]) < abs(cur_armor_height - m_armor_height[1]))
            m_target_pair_num = 0;
        else
            m_target_pair_num = 1;

        // 窗口修正
        if (abs_target_angle > m_angle_predict_max)
        {
            while (abs_target_angle > m_angle_predict_max)
            {
                abs_target_angle -= PI / 2;
                m_target_pair_num = (m_target_pair_num + 1) % 2;
            }
        }
        else
        {
            while (abs_target_angle < -m_angle_predict_max)
            {
                abs_target_angle += PI / 2;
                m_target_pair_num = (m_target_pair_num + 1) % 2;
            }
        }
        if (m_debug)
            std::cout << "[Top] pair_num: " << m_cur_pair_num << " " << m_target_pair_num << std::endl;

        // 判断击发窗口
        double angle_shoot_max = 0;
        switch (m_track_mode)
        {
        case FOLLOW:
            angle_shoot_max = m_angle_follow_shoot_max;
            break;
        case FOCUS:
            angle_shoot_max = m_angle_focus_shoot_max;
            break;
        default:
            break;
        }
        if ((abs(abs_target_angle + m_angle_shoot_off * m_rotate_direction) < angle_shoot_max &&        // 选取正面窗口
            m_cycle_cnt >= 2 && m_palstance_stable && m_center_stable) ||                               // 模型稳定
            getDistance(m_approx_center) < 1)                                                           // 一米内不限
        {
            m_shoot_flag = true;
        }
        else
        {
            m_shoot_flag = false;
        }
        if (m_debug)
            std::cout << "[Top] shoot_window: " << R2D(abs_target_angle) << "->" << R2D(abs_target_angle + m_angle_shoot_off * m_rotate_direction) << " " << R2D(angle_shoot_max) << "\t"
                                                << m_cycle_cnt << " " << m_palstance_stable << " " << m_center_stable << " " 
                                                << m_shoot_flag << std::endl;

        // 预测装甲板坐标
        // 两方旋转中心连线的旋转角
        double phi = atan2(m_approx_center.y, m_approx_center.x);
        if (m_debug)
            std::cout << "[Top] phi: " << phi << std::endl;
        // 坐标系变换矩阵
        cv::Mat_<double> T = (cv::Mat_<double>(2, 2) << cos(phi), -sin(phi),
                                                        sin(phi), cos(phi));
        // 目标系装甲板坐标
        cv::Mat_<double> C;
        switch (m_track_mode)
        {
        case FOLLOW:
            C = (cv::Mat_<double>(2, 1)
                << getDistance(m_approx_center) - m_radiuses[m_target_pair_num] * cos(abs_target_angle),
                                                - m_radiuses[m_target_pair_num] * sin(abs_target_angle));
            break;
        case FOCUS:
            C = (cv::Mat_<double>(2, 1)
                << getDistance(m_approx_center) - m_radiuses[m_target_pair_num], 0);
            break;
        default:
            C = (cv::Mat_<double>(2, 1)
                << getDistance(m_approx_center), 0);
            break;
        }
        
        // 世界系装甲板坐标
        cv::Mat_<double> C_ = T * C;
        if (m_debug)
        {
            std::cout << "[Top] target_sys_location: " << C.at<double>(0) << " " << C.at<double>(1) << std::endl;
            std::cout << "[Top] world_sys_location: " << C_.at<double>(0) << " " << C_.at<double>(1) << std::endl;
        }
        abs_target_armor.m_position.x = C_.at<double>(0);
        abs_target_armor.m_position.y = C_.at<double>(1);

        // 取目标装甲板绝对高度
        if (m_armor_height[m_target_pair_num] < 1e8)
        {
            abs_target_armor.m_position.z = m_armor_height[m_target_pair_num];
        }
        else
        {
            abs_target_armor.m_position.z = abs_cur_armor.m_position.z;
        }
        if (m_debug)
            std::cout << "[Top] final_height: " << m_armor_height[0] << " " << m_armor_height[1] << " " << abs_target_armor.m_position.z << " " << abs_cur_armor.m_position.z << std::endl;

		// 相机系装甲板坐标
        target_armor.m_position = m_angle_solver->abs2cam(abs_target_armor.m_position, m_cur_pose);

        if (m_write_file)
        {
            char buf[128];
            sprintf(buf, "%lf, %lf, %lf, %lf, %lf, %lf, %lf\n",
                    cur_angle,
                    cur_armor.m_position.x, cur_armor.m_position.y, cur_armor.m_position.z,
                    m_cur_pose.pitch, m_cur_pose.yaw,
                    cur_armor.m_time_seq);
            int n = write(m_file, buf, strlen(buf));
        }

        return target_armor;
    }

    GimbalPose AngleAimer::getTargetPose(Armor &armor)
    {
        return m_angle_solver->getAngle(getTargetArmor(armor).m_position, m_cur_pose, m_bullet_speed);
    }
} // namespace wmj
