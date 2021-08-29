#pragma once
#include "../../include/Transport.h"
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <queue>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <fcntl.h>
#include <unistd.h>


namespace wmj
{
    class SerialPort : public Port
    {
    public:
        SerialPort();
        ~SerialPort();

        Buffer readFrame(int flag);
        bool sendFrame(Buffer& data);

    private:
        int    m_debug;

        int    m_baud_write;
        int    m_baud_read;

        Buffer m_temp_buffer;
        Buffer m_buffer;

        Buffer m_read_maincontrl_buffer;
        Buffer m_read_gimbal_buffer;
        Buffer m_read_chassis_buffer;
        Buffer m_read_gyro_buffer;
        Buffer m_read_shooter_buffer;
        Buffer m_read_judge_buffer;
        Buffer m_read_communicate_buffer;
        Buffer m_read_gyro_angle_buffer;
        Buffer m_read_gyro_angular_velocity_buffer;
        Buffer m_read_gyro_acce_buffer;

        std::string m_read_device;
        std::string m_write_device;
        std::queue<Buffer> m_send_data_seq;

        boost::mutex m_serial_recv_mutex;
        boost::mutex m_serial_send_mutex;
        boost::mutex m_buffer_decode_mutex;
        boost::mutex m_buffer_recv_mutex;
        boost::mutex m_buffer_send_mutex;

        boost::shared_ptr<boost::asio::io_service>  m_sp_input_service;
        boost::shared_ptr<boost::asio::io_service>  m_sp_output_service;
        boost::shared_ptr<boost::asio::serial_port> m_sp_recv_port;
        boost::shared_ptr<boost::asio::serial_port> m_sp_send_port;
        boost::shared_ptr<boost::thread>            m_sp_read_t;
        boost::shared_ptr<boost::thread>            m_sp_write_t;

        void getInfo();
        void checkPort();
        void startThread();
        void stopThread();
        void startRead();
        void startWrite();
        void encode(Buffer& data, Buffer& frame); // 修正格式
        void decode();
        bool checkFrame(Buffer frame);
        void decodeFrame(Buffer frame);
        void initCanMode();
    };
}
