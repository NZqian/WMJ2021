# WMJ预测器代码

## 主要接口

```c++
    void MotionPredict::setShootSpeed(int a_shoot_level)
```

参数：

- a_shoot_level: 射速等级



```c++
    cv::Point3f MotionPredict::predict(cv::Point3f a_position)
```

参数：

- a_position: 当前装甲绝对坐标

返回：

​	预测后的绝对坐标



```c++
    cv::Point3f PosePredict::predict(cv::Point3f a_point)
```

参数：

- a_position: 当前装甲绝对坐标

返回：

​	预测后的绝对坐标



```c++
    wmj::GimbalPose PosePredict::predict(wmj::GimbalPose a_pose)
```

参数：

- a_pose: 当前云台位姿

返回：

​	预测后的云台位姿



```c++
    double KalmanFilter::predict(double a_val)
```

参数：

- a_val: 当前测量量

返回：

​	预测后的量

## 主要流程

1. 初始化参数，根据测距精度(7m误差<10cm)，设置过程噪声`processNoiseCov`较小(1e-5)，测量噪声`measerementNoiseCov`适中(1e-3)；从参数文件`Predict.yaml`读取`m_init_count_threshold`第一帧预测次数，`predict_coe`预测数度增益量，`control_freq`控制频率；外部调用`setShootSpeed(int)`设置当前射速等级
2. 外部直接调用`predict()`获得预测后的装甲坐标，内部判断调用`initFilter()`初始化卡尔曼滤波器，并用第一帧输入的装甲绝对坐标进行多次预测使其收敛
3. 判断当前帧和上一帧装甲相对距离，根据设置的阈值(target_change_threshold=0.25m)判断装甲板是否切换，装甲板切换需重新初始化滤波器
4. 得到预测结果，击打远距离装甲板子弹飞行时间更长，不同射速等级子弹飞行时间不同，所以加入装甲距离和射速调节预测量大小

```c++
        return cv::Point3f(prediction.at<float>(0) + m_predict_coe * m_cur_armor_dis / m_cur_max_shoot_speed * prediction.at<float>(3),
                           prediction.at<float>(1) + m_predict_coe * m_cur_armor_dis / m_cur_max_shoot_speed * prediction.at<float>(4),
                           prediction.at<float>(2) + m_predict_coe * m_cur_armor_dis / m_cur_max_shoot_speed * prediction.at<float>(5));
```

## 调参方法

1. 测距精度达到要求，先调试PID参数，完成后开始调试预测
2. 提前测算自瞄循环的帧率，主要调试predict_coe预测量和control_freq控制频率
3. 对于较快速的线性运动，有时需要增大target_change_threshold避免滤波器重复初始化造成跟随过程中的抖动，但该阈值过大容易在装甲板切换时预测出错

