#include "../include/StaticAimer.hpp"

namespace wmj
{
    inline double GetDistance(cv::Point3f point)
    {
        return sqrt(point.x * point.x + point.y * point.y);
    }
    StaticAimer::StaticAimer() : Aimer()
    {
        setParam();
        std::cout << "set param ok" << std::endl;
    }

    void StaticAimer::setParam()
    {
        std::cout << "setting param in child class" << std::endl;
        cv::FileStorage fs(TOP_CFG, cv::FileStorage::READ);
        fs["static"]["horizontal_diff_max"] >> m_horizontal_diff_max;
        fs["static"]["horizontal_diff_min"] >> m_horizontal_diff_min;
        fs["static"]["angle_predict"]       >> m_angle_predict;
        fs.release();
    }

    void StaticAimer::buildModel(wmj::Armor &armor)
    {
        // 转绝对坐标
        m_abs_armor.m_position = m_angle_solver->cam2abs(armor.m_position, m_cur_pose);
        if (m_armor_seq.size())
        {
            cv::Point3f pos_diff = m_abs_armor.m_position - m_armor_seq.back().m_position;
            double horizontal_diff = getDistance(pos_diff);
            std::cout << "[Armor] horizontal_diff " << horizontal_diff << std::endl;
            // 装甲板切换，即一个周期结束
            if (horizontal_diff > m_horizontal_diff_max)
            {
                std::cout << "[Armor] changed to another armor !" << std::endl;
                m_min_dist = INFINITY;
                std::cout << "[Armor] vecSize " << m_armor_seq.size() << std::endl;
                if (m_armor_seq.size() >= m_min_vec_size)
                {
                    m_armor_cur_target.m_time_seq = wmj::now();
                    m_armor_target[m_armor_no] = m_armor_cur_target;
                    m_pose_target[m_armor_no] = m_pose_cur_target;
                    // 两个周期后认为建模完成
                    if(!m_model_ready && m_armor_no == 1)
                    {
                        m_model_ready = true;
                        std::cout << "[Aimer] model ready!" << std::endl;
                    }
                    if(m_model_ready)
                    {
                        // 计算角速度大小
                        m_time_diff = m_armor_target[m_armor_no].m_time_seq - m_armor_target[(m_armor_no + 1) % 2].m_time_seq;
                        m_palstance = (PI / 2) / m_time_diff;
                        // 判断角速度方向
                        double y_start = m_angle_solver->abs2cam(m_armor_seq.front().m_position, m_pose_cur_target).y;
                        double y_end   = m_angle_solver->abs2cam(m_armor_seq.back().m_position,  m_pose_cur_target).y;
                        m_palstance *= (y_start - y_end) / abs(y_start - y_end);
                        std::cout << "[Aimer] palstance = " << m_palstance << std::endl;
                    }
                    m_armor_no = (m_armor_no + 1) % 2;
                }
                else
                {
                    m_model_ready = false;
                    m_armor_no = 0;
                    m_shoot_flag = 0;
                }
                m_armor_seq.clear();
            }
            // 认为装甲板静止
            else if (horizontal_diff < m_horizontal_diff_min)
            {
                std::cout << "[Armor] not spining " << horizontal_diff << std::endl;
                m_lost_cnt++;
                if(m_lost_cnt > 10)
                {
                    m_armor_seq.clear();
                    m_model_ready = false;
                    m_lost_cnt = 0;
                }
            }
        }
        m_armor_seq.emplace_back(m_abs_armor);

        // 找最近装甲板，即正面位置
        double dist = getDistance(m_abs_armor.m_position);
        if (dist < m_min_dist)
        {
            m_min_dist = dist;
            m_armor_cur_target = m_abs_armor;
            m_pose_cur_target = m_angle_solver->getAngle(armor.m_position, m_cur_pose, m_bullet_speed);
        }
    }

    GimbalPose StaticAimer::getTargetPose(wmj::Armor &)
    {
        int no = m_armor_no;
        GimbalPose final_pose;
        double time_now = wmj::now();
        // 预计击中时刻
        double time_next = time_now + m_time_delay + m_armor_target[m_armor_no].m_position.x / m_bullet_speed;
        // 提前量时长
        double time_len = (m_angle_predict * PI / 180) / m_palstance;

        Armor armor_target = m_armor_target[m_armor_no];
        GimbalPose pose_target = m_pose_target[m_armor_no];

        // 下一装甲板正对时刻
        double time_target = armor_target.m_time_seq + m_time_diff;
        // cv::Point3f pos_next = armor_target.m_position;

        while(time_next > time_target + time_len)
        {
            no = (no + 1) % 2;
            armor_target = m_armor_target[no];
            //pose_target = m_pose_target[(m_armor_no + 1) % 2];
            time_target += m_time_diff;
        }
        
        // if(time_next <= time_target - time_len)
        // {
        //     std::cout << "[Armor] waiting" << std::endl;
        //     //sleep(time_target - time_len - time_next);
        //     armor_target.m_position.y -= sin(time_len * m_palstance) * m_radius;
        //     m_shoot_flag = 0;
        // }
        
        // 不精准计算
        // armor_target.m_position.x += ?
        armor_target.m_position.y += sin((time_next - time_target) * m_palstance) * m_radius;

        // 判断发射窗口
        if(abs(time_next - time_target) < time_len)
        {
            std::cout << "[Armor] shooting" << std::endl;
            m_shoot_flag = 1;
        }

        std::cout << "[Armor] predict cord after" << armor_target.m_position << std::endl;
        armor_target.m_position = m_angle_solver->abs2cam(armor_target.m_position, m_cur_pose);
        final_pose = m_angle_solver->getAngle(armor_target.m_position, m_cur_pose, m_bullet_speed);

        return final_pose;
    }
}; //namespace wmj
