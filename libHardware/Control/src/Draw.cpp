#include "../include/WMJRobotControl.h"

namespace wmj
{
    void WMJRobotControl::drawShape(ROBO_OPT opt, ROBO_SHAPE shape, ROBO_LAYER layer, ROBO_COLOR color, cv::Point start_point, cv::Point end_point, ROBO_DELETE del)
    {
        DCMPack control_data{};
        Buffer data_to_send{};

        control_data.msg.image_opt1                 = (uint8_t)(color << 6) | (uint8_t)(layer << 4) | (uint8_t)(shape << 2) | (uint8_t)opt;
        control_data.msg.image_opt2                 = (uint8_t)del;
        control_data.msg.start_x_low8               = (uint8_t)start_point.x;
        control_data.msg.start_x_high4_start_y_low4 = ((uint8_t)(start_point.x >> 8) | (uint8_t)(start_point.y << 4));
        control_data.msg.start_y_high8              = (uint8_t)(start_point.y >> 4);
        control_data.msg.end_x_low8                 = (uint8_t)end_point.x;
        control_data.msg.end_x_high4_end_y_low4     = ((uint8_t)(end_point.x >> 8) | (uint8_t)(end_point.y << 4));
        control_data.msg.end_y_high8                = (uint8_t)(end_point.y >> 4);

        data_to_send.push_back((uint8_t)Draw);
        for(auto c : control_data.data)
        {
            data_to_send.push_back(c);
            std::cout <<  std::hex << (int)c << " ";
        }
        std::cout << "\n";

        canBus->sendFrame(data_to_send);
    }

    void WMJRobotControl::drawCircle(ROBO_LAYER layer, ROBO_COLOR color, cv::Point center_point, int radius)
    {
        drawShape(OPT_ADD, SHAPE_OVAL, layer, color, center_point, cv::Point(radius, radius));
    }

    void WMJRobotControl::drawLine(ROBO_LAYER layer, ROBO_COLOR color, cv::Point center_point, cv::Point end_point)
    {
        drawShape(OPT_ADD, SHAPE_LINE, layer, color, center_point, end_point);
    }

    void WMJRobotControl::drawRect(ROBO_LAYER layer, ROBO_COLOR color, cv::Point center_point, cv::Point diag_point)
    {
        drawShape(OPT_ADD, SHAPE_RECT, layer, color, center_point, diag_point);
    }

    void WMJRobotControl::drawOval(ROBO_LAYER layer, ROBO_COLOR color, cv::Point center_point, cv::Point width_height)
    {
        drawShape(OPT_ADD, SHAPE_OVAL, layer, color, center_point, width_height);
    }

    void WMJRobotControl::clearAll()
    {
        drawShape(OPT_DELETE, SHAPE_LINE, LAYER_0, COLOR_MAIN, cv::Point(0, 0), cv::Point(0, 0), DELETE_CLEAR);
    }

    void WMJRobotControl::clearLayer(ROBO_LAYER layer)
    {
        drawShape(OPT_DELETE, SHAPE_LINE, layer, COLOR_MAIN, cv::Point(0, 0), cv::Point(0, 0), DELETE_DEL);
    }
}
