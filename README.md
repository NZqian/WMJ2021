# WMJ战队 2021赛季视觉代码

西北工业大学纵维立方WMJ战队2021赛季视觉完整功能代码

## 仓库说明

### 仓库目录

```
.
├── libBase                     // 基础单元库，包含时间、键盘事件、彩色字符相关等常用函数和PI、GimbalPose等常用定义
├── libControl                  // 控制单元库，包含各种控制解算器
│   ├── PID                 	// PID速度环解算器类
│   ├── Pose                 	// 云台位姿解算器类
│   ├── Predict                 // 卡尔曼线性预测器类
│   └── TopAimer                // 反陀螺预测器类
├── libFSM                  	// 有限状态机库，包含状态机主循环线程和各线程核心代码，如装甲板自瞄、大符、自动曝光等
├── libHardware                 // 硬件单元库，包含与硬件和电控相关的代码库
│   ├── Control               	// 机器人控制类，包括云台位姿控制、发射控制、按键信息等
│   ├── Transport               // 机器人通讯类
│   │   ├── include				// 包含通讯基类的头文件和通讯协议
│   │   ├── CanPort				// can通讯类
│   │   └── SerailPort			// 串口通讯类
│   └── UsbCapture              // 大恒相机驱动
├── libVision                  	// 识别单元库，包含与识别相关的类
│   ├── Armor                 	// 装甲板识别类
│   ├── Bino                 	// 双目测距类
│   └── Rune                	// 大符识别与解算类
├── shell              			// 用于存放调试常用脚本，包括watchdog
├── video              			// 用于存放录制视频
├── wmj_cfg	              		// 曾用于存放所有参数文件，现仅存放标定参数文件
├── CMakeLists.txt              // CMake工程文件
├── Main.cpp                    // 主函数
├── README.md                   // 项目说明文件
└── setup                       // 初始化仓库脚本
```

### 运行环境

| 操作系统     | 依赖库                         | 硬件                                                 | Tool                   |
| ------------ | ------------------------------ | ---------------------------------------------------- | ---------------------- |
| Ubuntu 18.04 | OpenCV 3.4.x<br />大恒相机驱动 | IntelNUC<br />大恒工业相机<br />USB转can，或USB转TTL | g++/clang++<br />CMake |

### 运行方式

#### 调试

```shell
mkdir build
cd build
cmake ..
make -j
```

*或者直接运行setup脚本*

编译完成后根据需要执行测试程序或主程序。

#### 比赛

在开机脚本/etc/rc.local中加入如下行

```shell
cd /home/wmj/WMJ2021/build
./watchdog
```

### 代码原理

*详见各模块说明*

## 测试视频

链接：https://pan.baidu.com/s/1eZzshtl1IiLblyvBxiR2tA 
提取码：nwpu