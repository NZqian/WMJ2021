#include "../include/SerialPort.hpp"

namespace wmj
{
    SerialPort::SerialPort()
    {
        getInfo();
        checkPort();
        m_temp_buffer.resize(34, 0x00);
        m_buffer.reserve(512);
        m_read_gimbal_buffer.resize(8);
        m_read_maincontrl_buffer.resize(8);
        m_read_chassis_buffer.resize(8);
        m_read_gyro_buffer.resize(8);
        m_read_shooter_buffer.resize(8);
        m_read_judge_buffer.resize(8);
        m_read_communicate_buffer.resize(8);
        m_read_gyro_angle_buffer.resize(8);
        m_read_gyro_angular_velocity_buffer.resize(8);
        m_read_gyro_acce_buffer.resize(8);
        this->startThread();
    }

    void SerialPort::startThread()
    {
        std::cout << "set serial param.\n";
        m_sp_input_service = boost::make_shared<boost::asio::io_service>();
        m_sp_output_service = boost::make_shared<boost::asio::io_service>();

        m_sp_recv_port = boost::make_shared<boost::asio::serial_port>(*m_sp_input_service, m_read_device);
        m_sp_send_port = boost::make_shared<boost::asio::serial_port>(*m_sp_output_service, m_write_device);

        m_sp_recv_port->set_option(boost::asio::serial_port::baud_rate(m_baud_read));                                           // 特率
        m_sp_recv_port->set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));  // 流量控制
        m_sp_recv_port->set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));              // 奇偶校验
        m_sp_recv_port->set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));         // 停止位
        m_sp_recv_port->set_option(boost::asio::serial_port::character_size(8));                                           // 数据位

        m_sp_send_port->set_option(boost::asio::serial_port::baud_rate(m_baud_write));                                          // 特率
        m_sp_send_port->set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none)); // 流量控制
        m_sp_send_port->set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));             // 奇偶校验
        m_sp_send_port->set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));        // 停止位
        m_sp_send_port->set_option(boost::asio::serial_port::character_size(8));                                          // 数据位
        this->initCanMode();
        m_sp_read_t = boost::make_shared<boost::thread>([&]()
        {
            startRead();
            boost::asio::io_service::work work(*m_sp_input_service);
            m_sp_input_service->run();
        });
        m_sp_write_t = boost::make_shared<boost::thread>([&]()
        {
            startWrite();
            boost::asio::io_service::work work(*m_sp_output_service);
            m_sp_output_service->run();
        });
        std::cout << _warning("thread started.") + "\n";
    }

    SerialPort::~SerialPort()
    {
        stopThread();
    }

    void SerialPort::startRead()
    {
        boost::mutex::scoped_lock lock(m_serial_recv_mutex);
        m_sp_recv_port->async_read_some(boost::asio::buffer(m_temp_buffer, 128),
                                     [&](boost::system::error_code ec, std::size_t bytes_transferred)
        {
            boost::mutex::scoped_lock lock(m_buffer_decode_mutex);
            for (int i = 0; i < bytes_transferred; ++i)
                m_buffer.push_back(m_temp_buffer[i]);

            if(m_debug)
            {
                std::cout << "debug read buffer is ";
                for(auto i : m_buffer)
                    std::cout << std::hex << (int)i << " ";
                std::cout << "\n";
            }
            decode();
            this->startRead();
        });
    }

    void SerialPort::startWrite()
    {
        boost::mutex::scoped_lock lock(m_serial_send_mutex);
        if (m_send_data_seq.empty() == false)
        {
            if(m_debug)
            {
                std::cout << "debug write frame is ";
                for(int i = 0; i < 17; i++)
                    std::cout << std::hex << (int)m_send_data_seq.front()[i] << " ";
                std::cout << std::endl;
            }

            boost::asio::async_write(*m_sp_send_port, boost::asio::buffer(m_send_data_seq.front()),
                                     [&](boost::system::error_code ec, std::size_t bytes_transferred)
            {
                boost::mutex::scoped_lock lock(m_buffer_send_mutex);
                if (m_send_data_seq.empty() == false)
                {
                    m_send_data_seq.pop();
                    startWrite();
                }
            });
        }
    }

    /**
     *@brief: 根据格式编码
     *
     *@param: Buffer& 要传输的数据 Buffer& 编好码的数据
     */
    void SerialPort::encode(Buffer& data, Buffer& frame)
    {
        frame.resize(17);//AT(2) ID(4) Len(1) Data(8) /r/n(2)
        int idx, len;
        len = data.size();

        if (len < 2)
            return;
        idx = 1;

        uint16_t CAN_ID;

        switch (data[0])
        {
            case Gimbal:
                CAN_ID = 0x311;
                break;
            case Shooter:
                CAN_ID = 0x321;
                break;
            case MainControl:
                CAN_ID = 0x354;
                break;
            case Draw:
                CAN_ID = 0x348;
                break;
            case ComSend:
                CAN_ID = 0x340;
                break;
            case Chassis:
                CAN_ID = 0x301;
                break;
            default:
                break;
        }

        frame[0] = 0x41;
        frame[1] = 0x54;
        frame[2] = (uint8_t)(CAN_ID >> 3);
        frame[3] = ((uint8_t)CAN_ID & 0x0007) << 5;
        frame[4] = 0x00;
        frame[5] = 0x00;
        frame[6] = 0x08;
        for(int i = 7; i < 15; i++)
            frame[i] = data[idx++];
        frame[15] = 0x0D;
        frame[16] = 0x0A;
    }

    void SerialPort::decode()
    {
        while (m_buffer.size() > 16)
        {
            if (m_buffer[0] != 0x41 || m_buffer[1] != 0x54 || m_buffer[15] != 0x0D
                    || m_buffer[16] != 0x0A || m_buffer[6] != 0x08)
            {
                m_buffer.erase(m_buffer.begin());
                if(m_debug)
                {
                    std::cout << "not a frame" << std::endl;
                }
                continue;
            }
            else
            {
                decodeFrame({m_buffer.begin(), m_buffer.begin() + 17});
                m_buffer.erase(m_buffer.begin(), m_buffer.begin() + 17); // 同上面构造函数
            }
        }
    }

    void SerialPort::decodeFrame(Buffer frame)
    {
        // transform the canID
        uint8_t tempID[2];
        uint16_t CAN_ID;

        tempID[0] = frame[3];
        tempID[1] = frame[2];

        memcpy(&CAN_ID, tempID, sizeof(tempID));
        CAN_ID = CAN_ID >> 5;

        if(m_debug)
            std::cout << _cyan("debug CAN_ID : ") << std::hex << (int)CAN_ID << "\n";

        Buffer *target_buffer;
        switch (CAN_ID)
        {
            case 0x314: // 云台
                target_buffer = &m_read_gimbal_buffer;
                break;
            case 0x316: // 陀螺仪
                target_buffer = &m_read_gyro_buffer;
                break;
           // case 0x317: // 云台                        
              //  target_buffer = &m_read_gimbal_buffer; 
             //   break;                                
            case 0x324: // 枪管
                target_buffer = &m_read_shooter_buffer;
                break;
            case 0x334: // 主控
                target_buffer = &m_read_maincontrl_buffer;
                break;
            case 0x344: // 裁判系统
                target_buffer = &m_read_judge_buffer;
                break;
            case 0x342: // 己方通讯
                target_buffer = &m_read_communicate_buffer;
                break;
            case 0x307: //陀螺仪角度
                target_buffer = &m_read_gyro_angle_buffer;
                break;
            case 0x308:
                target_buffer = &m_read_gyro_angular_velocity_buffer;
                break;
            case 0x309:
                target_buffer = &m_read_gyro_acce_buffer;
                break;
            default:
                Buffer tmp_buf{8};
                target_buffer = &tmp_buf;
                break;
        }
        std::copy(frame.begin() + 7, frame.begin() + 15, (*target_buffer).begin());
    }

    Buffer SerialPort::readFrame(int flag)
    {
        Buffer value ;
        boost::mutex::scoped_lock lock(m_buffer_recv_mutex);
        switch (flag)
        {
            case 0:
                return Buffer{0} ;
            case Gimbal:
                value = m_read_gimbal_buffer;
                return value;
            case Shooter:
                value = m_read_shooter_buffer;
                return value;
            case MainControl:
                value = m_read_maincontrl_buffer;
                return value;
            case Gyro:
                value = m_read_gyro_buffer;
                return value;
            case Judge:
                value = m_read_judge_buffer;
                return value;
            case ComRecv:
                value = m_read_communicate_buffer;
                return value;
            case GyroAngle:
                value = m_read_gyro_angle_buffer;
                return value;
            case GyroAngularVelocity:
                value = m_read_gyro_angular_velocity_buffer;
                return value;
            case GyroAcce:
                value = m_read_gyro_acce_buffer;
                return value;
            default:
                return Buffer{0xff};
        }
    }

    bool SerialPort::sendFrame(Buffer& data)
    {
        boost::mutex::scoped_lock lock(m_buffer_send_mutex);
        Buffer frame;
        encode(data, frame);

        try
        {
            m_send_data_seq.push(frame);
        }
        catch (std::exception& ex)
        {
            return false;
        }

        if (m_send_data_seq.size() > 5)
            m_send_data_seq.pop();
        this->startWrite();
        return true;
    }

    void SerialPort::initCanMode()
    {
        Buffer frame;
        frame.resize(7);
        frame[0] = 0x41;
        frame[1] = 0x54;
        frame[2] = 0x2B;
        frame[3] = 0x41;
        frame[4] = 0x54;
        frame[5] = 0x0D;
        frame[6] = 0x0A;
        m_send_data_seq.push(frame);
        m_sp_recv_port->write_some(boost::asio::buffer(frame));
    }

    void SerialPort::stopThread()
    {
        m_sp_input_service->stop();
        m_sp_output_service->stop();
    }

    void SerialPort::getInfo()
    {
        std::cout << "getinfo" << std::endl;
        cv::FileStorage fs(PORT_CFG, cv::FileStorage::READ);
        fs["SerialPort"]["read"]  >> m_read_device;
        fs["SerialPort"]["write"] >> m_write_device;
        fs["SerialPort"]["debug"] >> m_debug;
        fs["SerialPort"]["baud_read"] >> m_baud_read;
        fs["SerialPort"]["baud_write"] >> m_baud_write;
        std::cout << m_baud_read << std::endl;
        fs.release();
    }

    void SerialPort::checkPort()
    {
        int fd   = open(m_read_device.c_str(), O_EXCL, NULL);
        bool ret = false;
        if(fd < 0)
        {
            std::cout << _warning("读串口 [") + _warning(m_read_device) + _warning("] 它无了!!!!!!!!!!!!!!!!!!!") << "\n";
            ret = true;
        }
        close(fd);

        fd = open(m_write_device.c_str(), O_EXCL, NULL);
        if(fd < 0)
        {
            std::cout << _warning("写串口 [") + _warning(m_write_device) + _warning("] 它无了!!!!!!!!!!!!!!!!!!!") << "\n";
            ret = true;
        }
        close(fd);
        if(ret)
        {
            std::cout << _warning("当前存在的串口 : ") << "\n";
            system("ls -l /dev/ttyUSB*");
            exit(0);
        }
        std::cout << "读串口 : " << _yellow(m_read_device) << "\n";
        std::cout << "写串口 : " << _yellow(m_write_device) << "\n";
    }
}
