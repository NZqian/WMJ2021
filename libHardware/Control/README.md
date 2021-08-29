# WMJ机器人控制模块

## 主要功能

​	此模块为基于通讯底层的机器人控制模块，根据最底层通讯的收发包接口进行各种功能的封装并提供简化的接口，包括弹丸发射、云台控制、比赛状态获取等。

## 主要接口说明

```c++
    void WMJRobotControl::SetGimbalAngle(float pitch_angle, float yaw_angle)
```

参数：

- 俯仰角
- 偏航角



```c++
    void WMJRobotControl::SetGimbalSpeed(float pitch_speed, float yaw_speed)
```

参数：

- 俯仰方向速度
- 偏航方向速度



```c++
    void WMJRobotControl::SetGimbal_YawSpeed_PitchAngle(float pitch_angle, float yaw_speed)
```

参数：

- 俯仰角
- 偏航方向速度



```c++
    GimbalPose WMJRobotControl::GetGimbalAngle()
```

返回：

- 当前云台位姿



```c++
    GimbalPose WMJRobotControl::GetGimbalSpeed()
```

返回：

- 当前云台速度



```c++
    void WMJRobotControl::navigate(float x_speed, float y_speed, float angular_speed, bool power)
```

参数：

- x速度
- y速度
- 角速度
- 缓冲功率使能标志



```c++
    void WMJRobotControl::ShootSome(int BulletNum)
```

参数：

- 要连发的子弹数



```c++
    void WMJRobotControl::KeepShoot()
```

功能：

- 保持连发直到解使能



```c++
    void WMJRobotControl::StopShoot()
```

功能：

- 停止发射，摩擦轮和拨弹轮解使能



```c++
    void WMJRobotControl::ShootNone()
```

功能：

- 不发射，但摩擦轮使能



```c++
    void WMJRobotControl::ShootOnce()
```

功能：

- 单发



```c++
    void WMJRobotControl::openBox()
```

功能：

- 开/关弹仓



```c++
    void WMJRobotControl::SwitchBaseCoor(bool Mode)
```

参数：

- 是否切换地面系标志位



```c++
    ROBO_STATE WMJRobotControl::GetRobotStatus()
```

返回：

- 键鼠模式获取当前机器人状态

```c++
    ROBO_ENERGY WMJRobotControl::getEnergyStatus()
```

返回：

- 从裁判系统获取能量机关状态



```c++
    float WMJRobotControl::getShootSpeedValue()
```

返回：

- 从裁判系统获取当前射速并记录，计算平均值回传



## 调试说明

1. 需要增删键位和状态时，可在src/WMJRobotControl.cpp的GetRobotStatus()和GetKeyboardInfo()中修改
2. 参数文件Control.yaml中，port的值决定通信方式，0、1分别对应串口、U转CAN
3. 另外三个参数各自对应其弹速等级下的实际射速，如15m射速上限实际射速13.7左右
