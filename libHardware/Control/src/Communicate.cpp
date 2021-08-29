#include "../include/WMJRobotControl.h"

namespace wmj
{
    uint8_t WMJRobotControl::recvFromRobot()
    {
        Buffer data_from_read = canBus->readFrame(ComRecv);
        return (uint8_t)data_from_read[1];

    }

    void WMJRobotControl::sendToRobot(ROBO_ID id, uint8_t test)
    {
        CCMPack control_data{};
        Buffer data_to_send{};

        control_data.msg.info = (uint8_t)id;
        control_data.msg.test = test;
        control_data.msg.placehoder1 = 0;
        control_data.msg.placehoder2 = 0;
        control_data.msg.placehoder3 = 0;
        control_data.msg.placehoder4 = 0;
        control_data.msg.placehoder5 = 0;
        control_data.msg.placehoder6 = 0;

        data_to_send.push_back((uint8_t)ComSend);
        for(auto c : control_data.data)
        {
            data_to_send.push_back(c);
        }

        canBus->sendFrame(data_to_send);
    }
}
