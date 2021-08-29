/**
 * 云台位置环测试文件
 * 测试pc对云台的控制，同时也可以查看客户端发送的指令
 */
#include "../libHardware/Control/include/WMJRobotControl.h"

int main()
{
    char c = 'f';
    std::thread keyThread(monitorKeyboard, &c);
    std::shared_ptr<wmj::WMJRobotControl> control = std::make_shared<wmj::WMJRobotControl>() ;

    float pitch = 0.0;
    float yaw   = 0.0;
    bool coor = false;
    wmj::GimbalPose recv_gp;
    wmj::GimbalPose send_gp;

    while(c != 27)
    {
        switch(c)
        {
            case 'w':
                //0.08
                pitch += 0.04;
                break;
            case 's':
                //0.08
                pitch -= 0.04;
                break;
            case 'a':
                yaw -= 0.08;
                break;
            case 'd':
                yaw += 0.08;
                break;
            case 'q':
                yaw = 0.8;
                break;
            case 'e':
                yaw = -0.8;
                break;
            case 'z':
                pitch = 0.4;
                break;
            case 'x':
                pitch = -0.4;
                break;
            case 'r':
                pitch = 0.0;
                yaw = 0.0;
                break;
            case '1':
                control->ShootOnce();
                break;
            case '2':
                control->ShootSome(7);
                break;
            case '3':
                control->KeepShoot();
                break;
            case '4':
                control->StopShoot();
                break;
            case '5':
                control->openBox();
                break;
            case '6':
                control->ShootNone();
                break;
            case 'v':
                coor = !coor;
            default:
                break;
        }
        c = 'f' ;
        usleep(1000);

        control->SwitchBaseCoor(coor) ;

        send_gp.pitch = pitch;
        send_gp.yaw   = yaw;
        recv_gp       = control->GetGimbalAngle();

        std::cout << _blue("recv ")   << recv_gp << "\n\n";
        std::cout << _yellow("send ") << send_gp << "\n";
        control->SetGimbalAngle(pitch, yaw) ;


        if(control->HighShootSpeed())
        {
            std::cout << _underline(_yellow("High!!!!!! 高射速模式")) << "\n";
        }
        else
        {
            std::cout << _underline(_yellow("Low!!!!!!! 低射速模式")) << "\n";
        }

        if(control->GetRobotStatus() == wmj::ROBO_STATE::STATE_RUNE)
        {
            std::cout << _lightpurple("击打大符状态") << std::endl;
        }
        std::cout << "**********************************\n";
    }
    keyThread.join();
    control->SetGimbalAngle(0, 0) ;
    control->StopShoot();
    return 0;
}
