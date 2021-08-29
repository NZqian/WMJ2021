# UsbCaptureWithThread
### 使用相机时，注意要修改UsbCapture.yaml里的序列号
```yaml
camera:
    camera0: camera0
    camera1: camera1

    # 相机序列号，每个相机不同，要做一定的修改
    left_cam_serial_num  : KE0190060245
    right_cam_serial_num : KE0190060246
```

相机工作流程图

![](https://s3.bmp.ovh/imgs/2021/08/7ff43da05e166905.png)

### 初始化
```c++

// 使用例
std::shared_ptr<wmj::UsbCapture> camera = std::make_shared<wmj::UsbCapture>(); // 初始化

camera->setExposureTime(2000, wmj::MANUAL, "left"); // 曝光时间，有AUTO和MANUAL两种模式，第三个参数不写默认双相机一起设置，left表示调整左相机，right调节右相机，下同
```

### 调节参数
```c++

// 白平衡
camera->setWhiteBalance(wmj::RED, 1.3433, "left");
camera->setWhiteBalance(wmj::GREEN, LwhitebalanceG, "right");
camera->setWhiteBalance(wmj::BLUE, LwhitebalanceB, "all");

// 增益
camera->setGain(wmj::ALL, 10.0);
```

### 获取信息和图像
```c++
// 查看相机信息，主要是看序列号，先插一个相机，看完再看另一个，防止搞混
camera->getCameraInfo();

// 获取左右图像
std::vector<wmj::MatWithTime> frames;
cv::Mat leftMat, rightMat;
*camera >> frames;

// 默认0索引为左图，1索引为右图
cv::cvtColor(frames[0].m_img, frames[0].m_img, cv::COLOR_RGB2BGR); // 获取到图像记得转BGR
cv::cvtColor(frames[1].m_img, frames[1].m_img, cv::COLOR_RGB2BGR);
leftMat = frames[0].m_img;
rightMat = frames[1].m_img;
```

### 相机参数模式切换
```c++
// 配置文件在UsbCapture.yaml里面，如下显示了识别装甲板和大符的两种不同相机模式
// 使用camera->cameraMode([mode]) 进行切换，默认normal模式
/*
armor:
    # 曝光
    exposure_mode : 0 # 0 MANUAL; 1 AUTO
    exposure      : 3000

    # 增益
    gain: 10.0

    # 白平衡
    auto_wb  : 0 # 0 手动; 1 自动白平衡
    wb_red   : 1.6484
    wb_green : 1.000
    wb_blue  : 1.5664

rune:
    # 曝光
    exposure_mode : 0 # 0 MANUAL; 1 AUTO
    exposure      : 800

    # 增益
    gain: 10.0

    # 白平衡
    auto_wb  : 0 # 0 手动; 1 自动白平衡
    wb_red   : 1.6484
    wb_green : 1.000
    wb_blue  : 1.5664
 */
camera->cameraMode("armor"); // 切换到识别装甲板时的状态
camera->cameraMode("rune");  // 切换到识别大符时的的状态
```
