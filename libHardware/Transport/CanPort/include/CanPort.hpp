#pragma once
/* Copyright (C)
 * 2018 - XiaMo
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 */

#include "../../include/Transport.h"
#include <linux/can/raw.h>

#include <cstring>
#include <ctime>
#include <cerrno>
#include <mutex>
#include <thread>
#include <memory>
#include <queue>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <net/if.h>

namespace wmj
{
    class CanPort : public Port
    {
    public:
        CanPort(std::string);
        CanPort(const CanPort&);
        virtual ~CanPort();
        virtual bool sendFrame(Buffer&);
        virtual Buffer readFrame(int);
        void startThread();
        void stopThread();

    private:
        int          m_sock{};
        sockaddr_can m_addr{};
        iovec        m_iov{};
        ifreq        m_ifr{};

        std::mutex m_can_mutex{};
        std::mutex m_read_buffer_mutex{};
        std::mutex m_write_buffer_mutex{};

        std::thread m_readThread{};
        std::thread m_writeThread{};

        canfd_frame m_send_frame{};
        canfd_frame m_read_frame{};

        Buffer m_read_maincontrol_buffer{};
        Buffer m_read_judge_buffer{};
        Buffer m_read_gimbal_buffer{};
        Buffer m_read_chassis_buffer{};
        Buffer m_read_gyro_buffer{};
        Buffer m_read_shooter_buffer{};
        Buffer m_read_communicate_buffer{};
        Buffer m_read_buffer{};

        bool m_thread_run{true};
        std::queue<Buffer> m_write_buffer{};

        bool tryWrite();
        bool tryRead();
        void readRun();
        void writeRun();

        uint8_t can_dlc2len(uint8_t);
        /* map the sanitized data length to an appropriate data length code */
        uint8_t can_len2dlc(uint8_t);
        /* parse ASCII hex character to dec number */
        uint8_t asc2nibble(uint8_t);

        int parseData(Buffer&, canfd_frame&);
        int parseCanFrame(Buffer&, canfd_frame&);
    };
}
