#include "../include/Aimer.hpp"

namespace wmj
{
    Ransac::Ransac()
    :   m_point_need{2}
    {
        setParam();
    }

    void Ransac::setParam()
    {
        cv::FileStorage fs(TOP_CFG, cv::FileStorage::READ);
        fs["ransac"]["debug"]               >> m_debug;
        fs["ransac"]["iteration_max"]       >> m_iteration_max;
        fs["ransac"]["model_confidence"]    >> m_model_confidence;
        fs["ransac"]["radius_min"]          >> m_radius_min;
        fs["ransac"]["radius_max"]          >> m_radius_max;
        fs["ransac"]["inlier_dist_ratio"]   >> m_inlier_dist_ratio;
        fs["ransac"]["inlier_dist_min"]     >> m_inlier_dist_min;
        fs["ransac"]["inlier_dist_max"]     >> m_inlier_dist_max;
        fs.release();

        std::cout << "[Ransac] Ransac param set" << std::endl;
    }

    void Ransac::initData()
    {
        m_iteration_cnt = 0;
        m_cur_iteration = m_iteration_max;

        m_bset_inlier_proportion = 0;
        m_best_radius = 0;
        m_best_center = cv::Point2d(0, 0);

        std::cout << "[Ransac] Data init" << std::endl;
    }

    /**
     * @brief 根据绝对装甲板拟合圆
     * 
     * @param armor_seq 绝对装甲板序列
     * 
     * @return 半径&圆心
     */
    std::pair<double, cv::Point2d> Ransac::fitCircle(std::vector<Armor> &armor_seq)
    {
        // 初始化数据
        m_vec_size = armor_seq.size();
        initData();

        // 迭代
        while (m_iteration_cnt < m_cur_iteration)
        {
            // 随机取两个点
            int i = m_rng.uniform(0, armor_seq.size());
            int j = m_rng.uniform(0, armor_seq.size());
            int k = m_rng.uniform(0, armor_seq.size());
            if (i == j || j == k || k == i) continue;

            // 计算一个圆
            double radius;
            cv::Point2d center;
            std::tie(radius, center) = getCircle(armor_seq[i], armor_seq[j], armor_seq[k]);
            m_iteration_cnt++;
            if (m_debug)
                std::cout << "[Ransac] circle: " << radius << " " << center << std::endl;

            if (radius < m_radius_min || radius > m_radius_max) continue;

            // 计算内点比例
            float inlier_proportion = verifyCircle(radius, center, armor_seq);
            if (inlier_proportion > m_bset_inlier_proportion)
            {
                // 记录为最佳圆
                m_bset_inlier_proportion = inlier_proportion;
                m_best_radius = radius;
                m_best_center = center;
                // 更新迭代次数
                m_cur_iteration = log(1 - m_model_confidence) / log(1 - pow(inlier_proportion, 2));
            }
        }
        if (m_debug)
            std::cout << "[Ransac] best: " << m_bset_inlier_proportion << " " << m_best_radius << " " << m_best_center << " "
                                           << m_cur_iteration << std::endl;

        return std::make_pair(m_best_radius, m_best_center);
    }

    // /**
    //  * @brief 根据两个装甲板计算一个圆
    //  * 
    //  * @return 半径&圆心
    //  */
    // std::pair<double, cv::Point2d> Ransac::getCircle(Armor & armor1, Armor &armor2)
    // {
    //     double radius;
    //     cv::Point2d center;

    //     return std::make_pair(radius, center);
    // }
    /**
     * @brief 根据三个装甲板计算一个圆
     * 
     * @return 半径&圆心
     */
    std::pair<double, cv::Point2d> Ransac::getCircle(Armor &armor1, Armor &armor2, Armor &armor3)
    {
        float x1 = armor1.m_position.x;
        float x2 = armor2.m_position.x;
        float x3 = armor3.m_position.x;

        float y1 = armor1.m_position.y;
        float y2 = armor2.m_position.y;
        float y3 = armor3.m_position.y;

        double radius;
        cv::Point2d center;

        center.x = (x1 * x1 + y1 * y1) * (y2 - y3) + (x2 * x2 + y2 * y2) * (y3 - y1) + (x3 * x3 + y3 * y3) * (y1 - y2);
        center.x /= (2 * (x1 * (y2 - y3) - y1 * (x2 - x3) + x2 * y3 - x3 * y2));

        center.y = (x1 * x1 + y1 * y1) * (x3 - x2) + (x2 * x2 + y2 * y2) * (x1 - x3) + (x3 * x3 + y3 * y3) * (x2 - x1);
        center.y /= (2 * (x1 * (y2 - y3) - y1 * (x2 - x3) + x2 * y3 - x3 * y2));

        radius = sqrt((center.x - x1) * (center.x - x1) + (center.y - y1) * (center.y - y1));
        
        return std::make_pair(radius, center);
    }

    /**
     * @brief 计算一个圆包含的内点数目
     * 
     * @return 包含内点比例
     */
    float Ransac::verifyCircle(double radius, cv::Point2d center, std::vector<Armor> &armor_seq)
    {
        int inlier_cnt = 0;
        double inlier_dist = m_inlier_dist_ratio * radius;
        inlier_dist = (inlier_dist < m_inlier_dist_min ? m_inlier_dist_min : inlier_dist) > m_inlier_dist_max ? m_inlier_dist_max : inlier_dist;
        for (int i = 0; i < m_vec_size; i++)
        {
            // 目标点到拟合圆的距离，圆外为正，圆内为负
            double dist = sqrt(pow(center.x - armor_seq[i].m_position.x, 2) + pow(center.y - armor_seq[i].m_position.y, 2)) - radius;
            if (abs(dist) < inlier_dist)
            {
                inlier_cnt++;
            }
        }
        if (m_debug)
            std::cout << "[Ransac] inlier: " << inlier_cnt << " / " << m_vec_size << std::endl;

        return (float)inlier_cnt / m_vec_size;
    }
}   // namespace wmj