#pragma once
#include "../../libVision/Armor/include/ArmorDetector.hpp"
#include "../../libVision/Rune/include/ArmorTrigger.hpp"
#include "../../libControl/PID/include/SpeedResolver.hpp"
#include "../../libControl/Pose/include/AngleSolver.hpp"
#include "../../libControl/Predict/include/MotionPredict.hpp"
#include "../../libHardware/UsbCapture/include/UsbCapture.h"
#include "../../libHardware/Control/include/WMJRobotControl.h"
#include "../../libControl/TopAimer/include/StaticAimer.hpp"
#include "../../libControl/TopAimer/include/AngleAimer.hpp"

class StateMachine
{
public:
    StateMachine();
    void setParam();
    void armorStrike();
    void runeStrike(bool state);
    void stateMachineLoop();
    void setArmorExposure();

private:
    std::shared_ptr<wmj::UsbCapture>           m_usb_capture;
    std::shared_ptr<wmj::SpeedResolver>        m_speed_resolver;
    std::shared_ptr<wmj::MotionPredict>        m_motion_predictor;
    std::shared_ptr<wmj::PosePredict>          m_pose_predictor;
    std::shared_ptr<wmj::WMJRobotControl>      m_robot_control;
    std::shared_ptr<wmj::ArmorDetectorDouble>  m_armor_detector;
    std::shared_ptr<wmj::ArmorTrigger>         m_rune_detector;
    std::shared_ptr<wmj::AngleSolver>          m_angle_solver;
    std::shared_ptr<wmj::Aimer>                m_top_aimer;

    std::vector<wmj::MatWithTime> m_frames;
    std::vector<cv::VideoWriter> m_recorders;

    enum PREDICT_MOD {POINT, POSE};
    PREDICT_MOD m_predict_mod;

    int              m_default_exposure;
    bool             m_auto_exposure_finished = false;
    int              m_auto_exposure_value;
    int              m_repeat_exposure_time = 0;

    bool             m_debug = true;
    bool             m_sm_status;
    bool             m_filter_init;
    float            m_shoot_offset_coe;
    long             m_time = 0;
    int              m_exposure = 0;
    //int              m_detectID = -1;
    wmj::Armor       m_armor_last;
    wmj::Armor       m_final_armor;
    cv::Point3f      m_armor_abs_point;
    wmj::GimbalPose  m_cur_pose;
    wmj::GimbalPose  m_target_pose;
    wmj::GimbalPose  m_gm_speed;
    wmj::ROBO_STATE  m_robo_state;
    wmj::ROBO_STATE  m_current_state;

    // 帧率控制
    wmj::Rate m_rate_control{100};
    wmj::Rate m_rate_smach{200};
};
