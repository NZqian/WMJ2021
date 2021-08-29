#pragma once
#include "../../../libBase/include/common.h"
#include <iostream>
#include <sys/time.h>
#include <functional>
#include <thread>
#include <unistd.h>
#include <mutex>
#include <GxIAPI.h>
#include <DxImageProc.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <string>

namespace wmj
{
    std::string getErrorString(GX_STATUS error_status);
    int checkError(GX_STATUS error_status, std::string error_info);

    typedef enum
    {
        RED,
        BLUE,
        GREEN,
        ALL,
        AUTOWB
    } COLOR;

    typedef enum
    {
        MANUAL,
        AUTO
    } EXPOSURE_MODE;

    struct UsbParam
    {
        bool exposure_mode;
        float exposure;
        float gain;

        bool auto_wb;
        float wb_red;
        float wb_blue;
        float wb_green;
        UsbParam();
        void setParam(std::string mode = "normal");
    };

    class UsbCapture
    {
    private:
        // flags
        bool              m_initialized;
        bool              m_device_open;
        std::vector<bool> m_updated;
        GX_OPEN_PARAM     m_open_param;
        UsbParam          m_param;

        uint8_t  m_capture_state;
        uint16_t m_image_width;
        uint16_t m_image_height;
        int32_t  m_acq_transfer_size;
        int32_t  m_acq_transfer_number_urb;
        int64_t  m_pixel_format;
        int64_t  m_color_filter;

        std::mutex m_sign_mutex;
        std::mutex m_data_mutex[2];

        std::vector<GX_DEV_HANDLE> m_device_handle;
        std::vector<GX_FRAME_DATA> m_frame_data;

        // buffers
        std::vector<void *> m_raw8_buffer;
        std::vector<void *> m_rgb_frame_data;

        // image buffers
        std::vector<cv::Mat>          m_frame_buf;
        std::vector<wmj::MatWithTime> m_img_buf;

        void getDeviceName();
        int initDriver();            // 1
        int openDevice();            // 2
        int initDevice();            // 3
        int initBuffer();            // 4
        int startStream();           // 5
        void imageLoop(std::string); // 6
        void imageDecode(int i);     // 6.5
        int stopStream();            // 7
        int closeDevice();           // 8
        void closeDriver();          // 9

    public:
        UsbCapture(int device_number = -1);
        ~UsbCapture();

        // device name
        int                      m_device_number;
        std::string              m_left_cam_serial_num;
        std::string              m_right_cam_serial_num;
        std::vector<double>      m_capture_stamp;
        std::vector<double>      m_capture_stamp_his;
        std::vector<std::string> m_orientation;
        std::vector<std::string> m_device_id;
        std::vector<std::string> m_serial_number;

        int operator>>(std::vector<MatWithTime>& image);
        int getImg(std::vector<MatWithTime>& image);

        //////Advance/////
        int setExposureTime(int value, EXPOSURE_MODE mode, std::string orientation = "all");
        int setWhiteBalance(COLOR channel, float rate, std::string orientation = "all");
        int setGain(COLOR channeal, float value, std::string orientation = "all");
        int setSharpness(float value, std::string orientation = "all");
        int setContrast(int value, std::string orientation = "all");
        void getCameraInfo();
        void cameraMode(std::string mode = "armor");
    };
}
