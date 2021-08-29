# PID调节器与速度解算器

## 主要接口

```c++
wmj::GimbalPose wmj::SpeedResolver::resolve(wmj::GimbalPose target_pose, wmj::GimbalPose cur_pose, double cur_time)
```

参数：

- target_pose: 根据识别结果解算的目标位姿
- cur_pose: 根据陀螺仪回传的云台当前位姿
- cur_time: 当前时间

返回：

​	云台速度

```c++
wmj::GimbalPose wmj::PidResolver::resolve(wmj::GimbalPose cur_error)
```

参数：

- cur_error: 当前误差

返回：

​	速度修正值

## 工作原理

1. 将目标位姿和当前位姿相减得到位姿误差，作为pid调节器的输入量
2. pid调节器根据已记录的误差和当前误差分别计算比例、积分和微分调节量，相加后得到调节修正值。如有必要，可以设置阈值进行分段调节
3. 根据上一帧数据和当前数据计算出基准速度，然后加上速度修正值得到最终速度并返回。

## 调参方法

1. 先调节比例系数kp，大约给30左右，云台越轻给越小，若跟随慢则适当增大kp
2. 除非云台调平问题严重，否则一般ki给0即可
3. kd视情况调节，若震荡大则适当增大kd
4. 加入卡尔曼预测后需重新微调pid参数
5. 一般pitch轴以位置环控制，所以主要调节yaw轴参数即可