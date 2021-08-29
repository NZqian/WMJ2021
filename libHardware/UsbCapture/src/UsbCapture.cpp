#include "../include/UsbCapture.h"

namespace wmj
{
    /**
     *@brief: 获取错误信息
     *
     *@param: 报错码
     *
     *@return: 错误信息
     */
    std::string getErrorString(GX_STATUS error_status)
    {
        char *error_info = NULL;
        size_t size = 0;
        GX_STATUS status = GX_STATUS_SUCCESS;
        std::string error_msg;

        status = GXGetLastError(&error_status, NULL, &size);
        error_info = new char[size];
        if( error_info == NULL )
        {
            std::cout << "Failed to allocate memory" << std::endl;
            return std::string("None");
        }

        status = GXGetLastError(&error_status, error_info, &size);
        if( status != GX_STATUS_SUCCESS )
        {
            std::cout << "GXGetLastError call failed" << std::endl;
        }
        else
        {
            error_msg = std::string(error_info);
        }

        if( error_info != NULL )
            delete [] error_info;

        return error_msg;
    }

    /**
     *@brief: 检测错误
     *
     *@param: 错误状态，报错信息
     *
     *@return: 函数状态
     */
    int checkError(GX_STATUS error_status, std::string error_info)
    {
        if(error_status != GX_STATUS_SUCCESS)
        {
            std::cout << error_info << "\n";
            std::cout << getErrorString(error_status) << "\n";
            return -1;
        }
        return 0;
    }

    /**
     *@brief: 相机参数读取
     */
    UsbParam::UsbParam()
    {
        this->setParam();
    }

    /**
     *@brief: 设置相机参数
     *
     *@param: 相机模式
     */
    void UsbParam::setParam(std::string mode)
    {
        cv::FileStorage fs(USBCAPTURE_CFG, cv::FileStorage::READ);
        fs[mode]["gain"]          >> this->gain;
        fs[mode]["exposure_mode"] >> this->exposure_mode;
        fs[mode]["exposure"]      >> this->exposure;
        fs[mode]["auto_wb"]       >> this->auto_wb;
        fs[mode]["wb_red"]        >> this->wb_red;
        fs[mode]["wb_green"]      >> this->wb_green;
        fs[mode]["wb_blue"]       >> this->wb_blue;
        fs.release();
    }

    UsbCapture::UsbCapture(int device_number):
        m_initialized(false),
        m_device_open(false),
        m_capture_state(1),
        m_acq_transfer_size(65536),
        m_acq_transfer_number_urb(64),
        m_pixel_format(GX_PIXEL_FORMAT_BAYER_RG8),
        m_color_filter(GX_COLOR_FILTER_BAYER_RG),
        m_device_number(device_number),
        m_image_width(1280),
        m_image_height(1024)
    {
        initDriver();
        openDevice();
        initDevice();
        initBuffer();
        startStream();
        getDeviceName();
        cameraMode();
        for (int i = 0; i < m_device_number; i++)
            std::thread(std::bind(&UsbCapture::imageLoop, this, m_device_id[i])).detach();
    }

    UsbCapture::~UsbCapture()
    {
        m_sign_mutex.lock();
        m_capture_state = 2;
        m_sign_mutex.unlock();
        for (int i = 0; i < m_device_number; ++i)
        {
            free(m_raw8_buffer[i]);
            free(m_rgb_frame_data[i]);
            m_raw8_buffer[i] = m_rgb_frame_data[i] = NULL;
        }
        stopStream();
        closeDevice();
        closeDriver();
    }

    /**
     *@brief: 获取带时间戳图像给外部
     *
     *@param: std::vector<MatWithTime>& 图像向量
     *
     *@return: int
     */
    int UsbCapture::operator>>(std::vector<MatWithTime>& image)
    {
        image.resize(m_device_number);
        uint8_t cnt = 0;
        while(true)
        {
            for (int i = 0; i < m_device_number; i++)
            {
                m_data_mutex[i].lock();
                if(m_device_number == 1)
                {
                    m_img_buf[i].m_img.copyTo(image[i].m_img);
                    image[i].m_time_stamp = m_capture_stamp[i];
                    m_data_mutex[i].unlock();
                    return 0;
                }

                if(m_updated[i] == true)
                {
                    if(m_serial_number[i] == m_left_cam_serial_num)
                    {
                        m_img_buf[i].m_img.copyTo(image[0].m_img);
                        image[0].m_time_stamp = m_capture_stamp[i];
                        image[0].m_orientation = "left";
                        m_orientation[i]       = "left";
                    }
                    else if(m_serial_number[i] == m_right_cam_serial_num)
                    {
                        m_img_buf[i].m_img.copyTo(image[1].m_img);
                        image[1].m_time_stamp = m_capture_stamp[i];
                        image[1].m_orientation = "right";
                        m_orientation[i]       = "right";
                    }

                    m_updated[i] = false;
                    m_data_mutex[i].unlock();
                    continue;
                }
                else
                {
                    cnt++;
                    m_data_mutex[i].unlock();
                    usleep(1000);
                }

                if(cnt > 20)
                {
                    std::cout << "Fetch image time out\n";
                    m_data_mutex[i].unlock();
                    return -1;
                }
            }
            break;
        }
        return 0;
    }

    /**
     *@brief: 同>>
     */
    int UsbCapture::getImg(std::vector<MatWithTime>& image)
    {
        return *this >> image;
    }

    /**
     *@brief: 设置曝光时间
     *
     *@param: int 曝光时长(微秒为单位)，曝光模式(自动曝光后value无意义，但要写一个)，string 选择左右相机，不写默认一起设置
     *
     *@return: what function returns
     */
    int UsbCapture::setExposureTime( int value, EXPOSURE_MODE mode, std::string orientation)
    {
        for (int i = 0; i < m_device_number; ++i)
        {
            if(orientation == "all" || m_orientation[i] == orientation)
            {
                GX_STATUS status = GX_STATUS_SUCCESS;
                status = GXSetEnum(m_device_handle[i], GX_ENUM_EXPOSURE_MODE, GX_EXPOSURE_MODE_TIMED);
                if(checkError(status, "Set exposure mode to timed error!")) return -1;

                if ( mode == AUTO )
                {
                    status = GXSetEnum(m_device_handle[i], GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_CONTINUOUS);
                    status = GXSetInt(m_device_handle[i], GX_INT_GRAY_VALUE, value);
                    if(checkError(status, "Set auto exposure and expected gray value error!")) return -2;
                }
                else
                {
                    status = GXSetEnum(m_device_handle[i], GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF);
                    status = GXSetFloat(m_device_handle[i], GX_FLOAT_EXPOSURE_TIME, (float)value);
                    if(checkError(status, "Set manual exposure and expected gray value error!")) return -3;
                }
            }
        }
        return 0;
    }

    /**
     *@brief: 设置白平衡
     *
     *@param: COLOR 选择颜色通道(AUTOWB表示自动白平衡，此时rate无意义，但要写)，float 比例值，string 选择左右相机，不写默认一起设置
     *
     *@return: int
     */
    int UsbCapture::setWhiteBalance(COLOR channel, float rate, std::string orientation)
    {
        for (int i = 0; i < m_device_number; i++)
        {
            if(orientation == "all" || m_orientation[i] == orientation)
            {
                GX_STATUS status = GX_STATUS_SUCCESS;
                if(channel == AUTOWB)
                {
                    status = GXSetEnum(m_device_handle[i], GX_ENUM_AWB_LAMP_HOUSE, GX_AWB_LAMP_HOUSE_ADAPTIVE);
                    status = GXSetEnum(m_device_handle[i], GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS);
                    continue;
                }
                else
                    status = GXSetEnum(m_device_handle[i], GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_OFF);

                if(channel == RED)
                    status = GXSetEnum(m_device_handle[i], GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_RED);
                else if(channel == GREEN)
                    status = GXSetEnum(m_device_handle[i], GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_GREEN);
                else if(channel == BLUE)
                    status = GXSetEnum(m_device_handle[i], GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_BLUE);


                if(checkError(status, "Set White balance channel Error!")) return -1;

                status = GXSetFloat(m_device_handle[i], GX_FLOAT_BALANCE_RATIO, (float)rate);
                if(checkError(status, "Set White balance ratio Error!")) return -2;
            }
        }
        return 0;
    }

    /**
     *@brief: 设置增益
     *
     *@param: COLOR 选择颜色通道，float 增益值，string 选择左右相机，不写默认一起设置
     *
     *@return: int
     */
    int UsbCapture::setGain(COLOR channel, float value, std::string orientation)
    {
        for (int i = 0; i < m_device_number; i++)
        {
            if(orientation == "all" || m_orientation[i] == orientation)
            {
                GX_STATUS status = GX_STATUS_SUCCESS;
                status = GXSetEnum(m_device_handle[i], GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_OFF);
                if(checkError(status, "Set manual gain Error!")) return -1;

                if ( channel == RED )
                {
                    status = GXSetEnum(m_device_handle[i], GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_RED);
                    status = GXSetFloat(m_device_handle[i], GX_FLOAT_GAIN, value);
                    if(checkError(status, "Set Gain red Error!")) return -2;
                }
                else if ( channel == GREEN )
                {
                    status = GXSetEnum(m_device_handle[i], GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_GREEN);
                    status = GXSetFloat(m_device_handle[i], GX_FLOAT_GAIN, value);
                    if(checkError(status, "Set Gain green Error!")) return -3;
                }
                else if ( channel == BLUE )
                {
                    status = GXSetEnum(m_device_handle[i], GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_BLUE);
                    status = GXSetFloat(m_device_handle[i], GX_FLOAT_GAIN, value);
                    if(checkError(status, "Set Gain blue Error!")) return -4;
                }
                else
                {
                    status = GXSetEnum(m_device_handle[i], GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_ALL);
                    status = GXSetFloat(m_device_handle[i], GX_FLOAT_GAIN, value);
                    if(checkError(status, "Set Gain all Error!")) return -5;
                }
            }
        }
        return 0;
    }

    /**
     *@brief: 设置锐度
     *
     *@param: float 锐度值，string 选择左右相机，不写默认一起设置
     *
     *@return: int
     */
    int UsbCapture::setSharpness(float value, std::string orientation)
    {
        for (int i = 0; i < m_device_number; i++)
        {
            if(orientation == "all" || m_orientation[i] == orientation)
            {
                GX_STATUS status = GX_STATUS_SUCCESS;
                status = GXSetEnum(m_device_handle[i], GX_ENUM_SHARPNESS_MODE, GX_SHARPNESS_MODE_ON);
                if(checkError(status, "Set sharpness on failed!")) return -1;

                status = GXSetFloat(m_device_handle[i], GX_FLOAT_SHARPNESS, value);
                if(checkError(status, "Set sharpness value failed!")) return -2;
            }
        }
        return 0;
    }

    /**
     *@brief: 设置对比度
     *
     *@param: int 对比度值，string 选择左右相机，不写默认一起设置
     *
     *@return: int
     */
    int UsbCapture::setContrast(int value, std::string orientation)
    {
        for (int i = 0; i < m_device_number; i++)
        {
            if(orientation == "all" || m_orientation[i] == orientation)
            {
                GX_STATUS status = GX_STATUS_SUCCESS;
                status = GXSetInt(m_device_handle[i], GX_INT_CONTRAST_PARAM, value);
                if(checkError(status, "Set contrast value failed!")) return -1;
            }
        }
        return 0;
    }

    /**
     *@brief: 获取相机一些信息
     *
     *@param: null
     *
     *@return: void
     */
    void UsbCapture::getCameraInfo()
    {
        GX_STATUS status = GX_STATUS_SUCCESS;
        size_t str_size = 0;
        for(int i = 0; i < m_device_number; i++)
        {
            status = GXGetStringMaxLength(m_device_handle[i], GX_STRING_DEVICE_SERIAL_NUMBER, &str_size);
            char *serial_number = new char[str_size];
            status = GXGetString(m_device_handle[i], GX_STRING_DEVICE_SERIAL_NUMBER, serial_number, &str_size);
            if(checkError(status, "Get serial number failed!")) return;

            std::cout << "设备序列号 : "   << std::string(serial_number) << "\n";

            delete[] serial_number;
        }
    }

    /**
     *@brief: 从UsbCapture.yaml读取参数
     */
    void UsbCapture::getDeviceName()
    {
        cv::FileStorage fs(USBCAPTURE_CFG, cv::FileStorage::READ);
        fs["left_cam_serial_num"]  >> m_left_cam_serial_num;
        fs["right_cam_serial_num"] >> m_right_cam_serial_num;
        for (int i = 0; i < m_device_number; i++)
        {
            fs["camera" + std::to_string(i)] >> m_device_id[i];
            std::cout << "Get device ==> " << m_device_id[i] << "\n";
            if(m_serial_number[i] == m_left_cam_serial_num)
                std::cout << m_device_id[i] << "这是left相机 : " << m_serial_number[i] << "\n";
            else if(m_serial_number[i] == m_right_cam_serial_num)
                std::cout << m_device_id[i] << "这是right相机 : " << m_serial_number[i] << "\n";
        }
        fs.release();
    }

    /**
     *@brief: 初始化相机库等设置
     *
     *@return: int
     */
    int UsbCapture::initDriver()
    {
        GX_STATUS status = GXInitLib();
        if(checkError(status, "InitLib Error!")) return -1;

        uint32_t device_number;
        status = GXUpdateDeviceList(&device_number, 1000);
        if(checkError(status, "Update device list Error!")) return -2;

        if (m_device_number < 0)
            m_device_number = (int)device_number;
        std::cout << "device_number ==> " << m_device_number << "\n";

        m_updated.resize(m_device_number, false);
        m_device_id.resize(m_device_number);
        m_orientation.resize(m_device_number);
        m_serial_number.resize(m_device_number);
        m_device_handle.resize(m_device_number);
        m_capture_stamp.resize(m_device_number, 0);
        m_capture_stamp_his.resize(m_device_number, 0);

        // resize Buffers
        m_img_buf.resize(m_device_number);
        m_frame_data.resize(m_device_number);
        m_frame_buf.resize(m_device_number);
        m_raw8_buffer.resize(m_device_number);
        m_rgb_frame_data.resize(m_device_number);

        if ( m_device_number <= 0 )
        {
            std::cout << "No device\n";
            return -3;
        }

        //m_open_param.accessMode = GX_ACCESS_EXCLUSIVE;
        m_open_param.accessMode = GX_ACCESS_CONTROL;
        m_open_param.openMode = GX_OPEN_INDEX;
        return 0;
    }

    /**
     *@brief: 打开相机
     *
     *@return: int
     */
    int UsbCapture::openDevice()
    {
        GX_STATUS status = GX_STATUS_SUCCESS;
        for (int i = 0; i < m_device_number; ++i)
        {
            m_open_param.pszContent = const_cast<char *>(std::to_string(i + 1).c_str()); // string to char*
            status = GXOpenDevice(&m_open_param, &m_device_handle[i]);
            if(checkError(status, "Open Device failed!")) return -1;
        }

        for (int i = 0; i < m_device_number; ++i)
        {
            size_t numberSize = 0;
            status = GXGetStringMaxLength(m_device_handle[i], GX_STRING_DEVICE_SERIAL_NUMBER, &numberSize);
            char *serial_number = new char[numberSize];
            status = GXGetString(m_device_handle[i], GX_STRING_DEVICE_SERIAL_NUMBER, serial_number, &numberSize);
            m_serial_number[i] = serial_number;
            std::cout << "Open Device [" << i << "] Succeeded\n";
            if(checkError(status, "Get Device Serial Number failed!")) return -1;

            delete[] serial_number;
        }

        m_device_open = true;
        return 0;
    }

    /**
     *@brief: 初始化相机各种参数
     *
     *@return: int
     */
    int UsbCapture::initDevice()
    {
        GX_STATUS status = GX_STATUS_SUCCESS;
        for (int i = 0; i < m_device_number; ++i)
        {
            status = GXSetEnum(m_device_handle[i], GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
            if(checkError(status, "Set Enum Acq mode failed!")) return -1;

            // 传输数据大小
            status = GXSetInt(m_device_handle[i], GX_DS_INT_STREAM_TRANSFER_SIZE, m_acq_transfer_size);
            if(checkError(status, "Set Int stream transfer size failed!")) return -2;

            // 请求块大小
            status = GXSetInt(m_device_handle[i], GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, m_acq_transfer_number_urb);
            if(checkError(status, "Set Int stream transfer number urb failed!")) return -3;

            // 像素格式化
            status = GXSetEnum(m_device_handle[i], GX_ENUM_PIXEL_FORMAT, m_pixel_format);
            if(checkError(status, "Query pixel format failed!")) return -4;

            // 颜色滤镜
            status = GXGetEnum(m_device_handle[i], GX_ENUM_PIXEL_COLOR_FILTER, &m_color_filter);
            if(checkError(status, "Query color filter failed!")) return -5;
        }

        m_initialized = true;
        return 0;
    }

    /**
     *@brief: 初始化缓冲区
     *
     *@return: int
     */
    int UsbCapture::initBuffer()
    {
        GX_STATUS status = GX_STATUS_SUCCESS;
        int64_t payloadSize = 0;

        for (int i = 0; i < m_device_number; ++i)
        {
            // 申请内存
            status = GXGetInt(m_device_handle[i], GX_INT_PAYLOAD_SIZE, &payloadSize);
            if(checkError(status, "Get payload size failed!")) return -1;

            // 申请帧存
            m_frame_data[i].pImgBuf = malloc(payloadSize);
            if (m_frame_data[i].pImgBuf == NULL)
            {
                std::cout << "Failed to allocate memory for image buffer\n";
                return -2;
            }

            // 申请原始数据内存
            m_raw8_buffer[i] = malloc(payloadSize);
            if (m_raw8_buffer[i] == NULL)
            {
                std::cout << "Failed to allocate memory for raw data buffer\n";
                return -3;
            }

            // 申请处理后图片内存
            m_rgb_frame_data[i] = malloc(payloadSize * 3);
            if (m_rgb_frame_data[i] == NULL)
            {
                std::cout << "Failed to allocate memory for rgb frame data\n";
                return -4;
            }
            std::cout << "initBuffer " << std::to_string(i) << " ok\n";
        }
        return 0;
    }

    /**
     *@brief: 开始捕获图像
     *
     *@return: int
     */
    int UsbCapture::startStream()
    {
        for (int i = 0; i < m_device_number; ++i)
        {
            GX_STATUS status = GXSendCommand(m_device_handle[i], GX_COMMAND_ACQUISITION_START);
            if(checkError(status, "Start video stream Error!")) return -1;
        }
        std::cout << "Stream started\n";
        m_capture_state = 1;
        return 0;
    }

    /**
     *@brief: 获取图像
     *
     *@param: string 设备索引
     */
    void UsbCapture::imageLoop(std::string device_id)
    {
        GX_STATUS status = GX_STATUS_SUCCESS;
        int index = device_id[6] - 48;

        while (true)
        {
            if (m_capture_state == 1) // 相机还开着
            {
                m_data_mutex[index].lock();
                status = GXGetImage(m_device_handle[index], &m_frame_data[index], 100);
                m_capture_stamp[index]     = wmj::now();
                m_capture_stamp_his[index] = m_capture_stamp[index];
                if (status == GX_STATUS_SUCCESS)
                {
                    imageDecode(index);
                    m_frame_buf[index]           = cv::Mat(m_frame_data[index].nHeight, m_frame_data[index].nWidth, CV_8UC3, m_rgb_frame_data[index]);
                    m_img_buf[index].m_img       = m_frame_buf[index];
                    m_img_buf[index].m_time_stamp = m_capture_stamp[index];
                    m_updated[index]             = true;
                    m_data_mutex[index].unlock();
                }
                else
                {
                    std::cout << "Get image error!\n";
                    std::cout << getErrorString(status) << std::endl;
                    m_data_mutex[index].unlock();
                }
            }
            usleep(500);
        }
    }

    /**
     *@brief: 图像转码
     *
     *@param: int 索引
     */
    void UsbCapture::imageDecode(int i)
    {
        switch (m_pixel_format)
        {
            case GX_PIXEL_FORMAT_BAYER_GB12:
            case GX_PIXEL_FORMAT_BAYER_GR12:
            case GX_PIXEL_FORMAT_BAYER_RG12:
            case GX_PIXEL_FORMAT_BAYER_BG12:
                DxRaw16toRaw8(m_frame_data[i].pImgBuf, m_raw8_buffer[i], m_image_width, m_image_height, DX_BIT_4_11);
                DxRaw8toRGB24(m_raw8_buffer[i], m_rgb_frame_data[i], m_image_width, m_image_height, RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(m_color_filter), false);
                break;
            case GX_PIXEL_FORMAT_BAYER_RG10:
            case GX_PIXEL_FORMAT_BAYER_GB10:
            case GX_PIXEL_FORMAT_BAYER_GR10:
            case GX_PIXEL_FORMAT_BAYER_BG10:
                DxRaw16toRaw8(m_frame_data[i].pImgBuf, m_raw8_buffer[i], m_image_width, m_image_height, DX_BIT_2_9);
                DxRaw8toRGB24(m_raw8_buffer[i], m_rgb_frame_data[i], m_image_width, m_image_height, RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(m_color_filter), false);
                break;
            case GX_PIXEL_FORMAT_BAYER_BG8:
            case GX_PIXEL_FORMAT_BAYER_GB8:
            case GX_PIXEL_FORMAT_BAYER_GR8:
            case GX_PIXEL_FORMAT_BAYER_RG8:
                DxRaw8toRGB24(m_frame_data[i].pImgBuf, m_rgb_frame_data[i], m_image_width, m_image_height, RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(m_color_filter), false);
                break;
            default:
                break;
        }
    }

    /**
     *@brief: 停止捕获
     */
    int UsbCapture::stopStream()
    {
        for (int i = 0; i < m_device_number; ++i)
        {
            GX_STATUS status = GX_STATUS_SUCCESS;
            status = GXSendCommand(m_device_handle[i], GX_COMMAND_ACQUISITION_STOP);
            if(checkError(status, "Stop video stream Error!")) return -1;
        }
        return 0;
    }

    /**
     *@brief: 关闭相机
     */
    int UsbCapture::closeDevice()
    {
        for (int i = 0; i < m_device_number; ++i)
        {
            GX_STATUS status = GX_STATUS_SUCCESS;
            status = GXCloseDevice(m_device_handle[i]);
            if(checkError(status, "Close device failed!")) return -1;
        }
        return 0;
    }

    /**
     *@brief: 关闭相机库
     */
    void UsbCapture::closeDriver()
    {
        for (int i = 0; i < m_device_number; ++i)
            if (m_device_handle[i] != NULL)
                m_device_handle[i] = NULL;
        GXCloseLib();
    }

    /**
     *@brief: 设置相机白平衡，曝光等参数
     */
    void UsbCapture::cameraMode(std::string mode)
    {
        m_param.setParam(mode);
        // 曝光
        if(m_param.exposure_mode == AUTO)
            this->setExposureTime(100, wmj::AUTO);
        else
            this->setExposureTime(m_param.exposure, wmj::MANUAL);

        // 增益
        this->setGain(wmj::ALL, m_param.gain);

        // 白平衡
        if(m_param.auto_wb)
            this->setWhiteBalance(wmj::AUTOWB, 1.2);
        else
        {
            this->setWhiteBalance(wmj::RED, m_param.wb_red);
            this->setWhiteBalance(wmj::GREEN, m_param.wb_green);
            this->setWhiteBalance(wmj::BLUE, m_param.wb_blue);
        }
    }
}
