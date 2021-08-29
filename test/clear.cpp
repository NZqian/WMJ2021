// 让机器人停止所有运动
#include "../libHardware/Control/include/WMJRobotControl.h"

int main(int argc, char **argv)
{
    std::shared_ptr<wmj::WMJRobotControl> control = std::make_shared<wmj::WMJRobotControl>();

    for(int i = 0; i < 100; ++i)
    {
        usleep(100);
        control->SetGimbalSpeed(0, 0);
        control->StopShoot();
    }
    return 0;
}
