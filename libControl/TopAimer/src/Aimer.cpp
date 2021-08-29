#include "../include/Aimer.hpp"

namespace wmj
{
    Aimer::Aimer()
    :   m_model_ready{false},
        m_shoot_flag{false},
        m_track_mode{FOLLOW}
    {
        m_angle_solver  = std::make_shared<wmj::AngleSolver>();
        m_ransac_solver = std::make_shared<wmj::Ransac>();
        setParam();
    }

    void Aimer::setParam()
    {
        cv::FileStorage fs(TOP_CFG, cv::FileStorage::READ);
        fs["aimer"]["debug"]        >> m_debug;
        fs["aimer"]["write_file"]   >> m_write_file;

        fs["aimer"]["min_vec_size"] >> m_min_vec_size;
        fs["aimer"]["delay"]        >> m_time_delay;
        fs["aimer"]["radius"]       >> m_radius;
        fs["aimer"]["bullet_speed"] >> m_bullet_speed;
        fs.release();

        std::cout << "[Top] Top param set" << std::endl;
    }

    /**
    * @brief 主循环调用，返回目标装甲板
    * 
    * @param cur_pose 当前位姿
    * @param armor 当前装甲板
    */
    Armor Aimer::getFinalArmor(GimbalPose &cur_pose, Armor &armor)
    {
        m_cur_pose = cur_pose;
        Armor target_armor;

        buildModel(armor);
        if (m_model_ready)
        {
            target_armor = getTargetArmor(armor);
            std::cout << _lightcyan("<<<<<< Top model built >>>>>>");
            if (m_shoot_flag)
            {
                std::cout << _cyan("\t<<<<<< Enable shoot >>>>>>") << std::endl;
            }
            else
            {
                std::cout << std::endl;
            }
        }
        else
        {
            target_armor = armor;
            m_shoot_flag = false;
        }
        
        return target_armor; 
    }
    /**
    * @brief 主循环调用，返回目标装甲板
    * 
    * @param cur_pose 当前位姿
    * @param armor 当前装甲板
    * @param bullet_speed 当前射速
    */
    Armor Aimer::getFinalArmor(GimbalPose &cur_pose, Armor &armor, float bullet_speed)
    {
        setBulletSpeed(bullet_speed);
        return getFinalArmor(cur_pose, armor);
    }

    /**
    * @brief 主循环调用，返回目标位姿。受跟随模式影响
    * 
    * @param cur_pose 当前位姿
    * @param armor 当前装甲板
    */
    GimbalPose Aimer::getFinalPose(GimbalPose &cur_pose, Armor &armor)
    {
        m_cur_pose = cur_pose;
        GimbalPose target_pose;
        
        if (m_debug)
        {
            if (m_track_mode == FOLLOW)
                std::cout << _yellow("[Top] Track mode: FOLLOW") << std::endl;
            else if (m_track_mode == FOCUS)
                std::cout << _yellow("[Top] Track mode: FOCUS") << std::endl;
        }

        buildModel(armor);
        if (m_model_ready)
        {
            target_pose = getTargetPose(armor);
            std::cout << _lightcyan("<<<<<< Top model built >>>>>>");
            if (m_shoot_flag)
            {
                std::cout << _cyan("\t<<<<<< Enable shoot >>>>>>") << std::endl;
            }
            else
            {
                std::cout << std::endl;
            }
        }
        else
        {
            switch (m_track_mode)
            {
            case FOLLOW:
                target_pose = m_angle_solver->getAngle(armor.m_position, cur_pose, m_bullet_speed);
                break;
            case FOCUS:
                target_pose = m_angle_solver->getAngle(armor.m_position, cur_pose, m_bullet_speed);
                break;
            default:
                target_pose = cur_pose;
                break;
            }
            m_shoot_flag = false;
        }

        return target_pose;
    }
    /**
    * @brief 主循环调用，返回目标位姿。受跟随模式影响
    * 
    * @param cur_pose 当前位姿
    * @param armor 当前装甲板
    * @param bullet_speed 当前射速
    */
    GimbalPose Aimer::getFinalPose(GimbalPose &cur_pose, Armor &armor, float bullet_speed)
    {
        setBulletSpeed(bullet_speed);
        return getFinalPose(cur_pose, armor);
    }

    /**
     * @brief 设置射速
     * 
     * @param bullet_speed 射速
     */
    void Aimer::setBulletSpeed(float bullet_speed)
    {
        m_bullet_speed = bullet_speed;
    }

    /**
     * @brief 设置跟随模式
     * 
     * @param track_mode FOLLOW: 跟随装甲板；
     *                   FOCUS: 保持云台指向敌方中心，选择最佳角度射击。
     */
    void Aimer::setTrackMode(TrackMode track_mode)
    {
        m_track_mode = track_mode;
    }

    bool Aimer::shootable()
    {
        return m_shoot_flag;
    }
} // namespace wmj
