#include "../include/Binocular.h"

namespace wmj
{
    void Binocular::setParam()
    {
        cv::FileStorage fs(BINO_CFG, cv::FileStorage::READ);
        fs["M1"] >> m_camera_matrix_left;
        fs["D1"] >> m_dist_coeffs_left;
        fs["M2"] >> m_camera_matrix_right;
        fs["D2"] >> m_dist_coeffs_right;
        fs["R"]  >> m_rotation;
        fs["T"]  >> m_translation;
        fs["R1"] >> m_R1;
        fs["R2"] >> m_R2;
        fs["P1"] >> m_P1;
        fs["P2"] >> m_P2;
        fs.release();
    }

    Binocular::Binocular(double f)
    {
        setParam();
        double x, y, z;
        m_f                       = f;
        x                         = m_translation.at<double>(0, 0);
        y                         = m_translation.at<double>(1, 0);
        z                         = m_translation.at<double>(2, 0);
        m_camera_dist             = sqrt(x * x + y * y + z * z);
        m_new_camera_matrix_left  = cv::getOptimalNewCameraMatrix(m_camera_matrix_left, m_dist_coeffs_left, cv::Size(1280, 1024), 0, cv::Size(1280, 1024));
        m_new_camera_matrix_right = cv::getOptimalNewCameraMatrix(m_camera_matrix_right, m_dist_coeffs_right, cv::Size(1280, 1024), 0, cv::Size(1280, 1024));
    }

    cv::Point2f Binocular::pixelToImagePlane(cv::Point2f& input_point, cv::Mat camera_matrix)
    {
        cv::Point2f output_point;

        double cx           = camera_matrix.at<double>(0, 2);
        double cy           = camera_matrix.at<double>(1, 2);
        double pix_per_mm_x = camera_matrix.at<double>(0, 0) / m_f;
        double pix_per_mm_y = camera_matrix.at<double>(1, 1) / m_f;
        double u            = input_point.x;
        double v            = input_point.y;

        output_point.x = (input_point.x - cx) / pix_per_mm_x;
        output_point.y = (input_point.y - cy) / pix_per_mm_y;
        return output_point;
    }

    cv::Point2f Binocular::distortionCorrection(cv::Point2f& input_point, bool direction)
    {
        cv::Mat RR;
        cv::Mat PP;
        cv::Mat camera_matrix, dist_coeffs, new_camera_matrix;
        cv::Point2f output_point;
        if (direction == 0)
        {
            RR                = m_R1;
            PP                = m_P1;
            camera_matrix     = m_camera_matrix_left;
            dist_coeffs       = m_dist_coeffs_left;
            new_camera_matrix = m_new_camera_matrix_left;
        }
        else
        {
            RR                = m_R2;
            PP                = m_P2;
            camera_matrix     = m_camera_matrix_right;
            dist_coeffs       = m_dist_coeffs_right;
            new_camera_matrix = m_new_camera_matrix_right;
        }

        cv::Point2f image_plane_point;
        std::vector<cv::Point2f> input_point_vec{input_point}, undistort_point_vec;
        cv::undistortPoints(input_point_vec, undistort_point_vec, camera_matrix, dist_coeffs, RR, new_camera_matrix);
        output_point = pixelToImagePlane(undistort_point_vec[0], new_camera_matrix);
        return output_point;
    }

    cv::Point3f Binocular::getPosition(cv::Point2f point_avg_left, cv::Point2f point_avg_right, bool direction)
    {
        cv::Point3f position;
        cv::Point2f corrected_point_left;
        cv::Point2f corrected_point_right;

        corrected_point_left  = distortionCorrection(point_avg_left, 0);
        corrected_point_right = distortionCorrection(point_avg_right, 1);
        double x_diff         = abs(corrected_point_left.x - corrected_point_right.x);
        double distance       = m_camera_dist * m_f / x_diff;
        double y,z;
        if(direction)
        {
            y = distance * corrected_point_right.x / m_f;
            z = distance * corrected_point_right.y / m_f;
        }
        else
        {
            y = distance * corrected_point_left.x / m_f;
            z = distance * corrected_point_left.y / m_f;  
        }
        if (distance > 0)
        {
            position.x = distance;
            position.y = -y;
            position.z = -z; // z轴向下
        }
        return position;
    }
    std::pair<cv::Point3f, double> Binocular::getPositionAngle(std::pair<cv::Point2f, cv::Point2f> point_avg_left, std::pair<cv::Point2f, cv::Point2f> point_avg_right){
        cv::Point3f position_left[2], position_right[2], position;
        double angle;
        //以左目为基准
        position_left[0] = this->getPosition(point_avg_left.first, point_avg_right.first,0);
        position_right[0] = this->getPosition(point_avg_left.second, point_avg_right.second,0);
        //以右目为基准
        // position_left[1] = this->getPosition(point_avg_left.first, point_avg_right.first,1);
        // position_right[1] = this->getPosition(point_avg_left.second, point_avg_right.second,1);
        position = (position_left[0] + position_right[0]) / 2;
        angle = atan2(position_right[0].x - position_left[0].x, position_left[0].y - position_right[0].y);

        return std::make_pair(position, angle);
    }
    std::pair<cv::Point3f, cv::Point3d> Binocular::getPositionAngle(std::vector<cv::Point2f> points_left,std::vector<cv::Point2f> points_right){
        cv::Point3f position_left[4], position_right[4], position;
        cv::Point3d angle_final;
        //以左目为基准
        position_left[0] = this->getPosition(points_left[0], points_right[0],0);
        position_left[1] = this->getPosition(points_left[1], points_right[1],0);
        position_left[2] = this->getPosition(points_left[2], points_right[2],0);
        position_left[3] = this->getPosition(points_left[3], points_right[3],0);   
        position = (position_left[0] + position_left[1] + position_left[2] + position_left[3]) / 4;
        
        cv::Point3d a = position_left[1] - position_left[0];
        cv::Point3d b = position_left[2] - position_left[0];
        cv::Point3d c = position_left[3] - position_left[0];
        cv::Point3d angle = a.cross(b);
        angle /= sqrt(pow(angle.x,2) + pow(angle.y,2) + pow(angle.z,2));

        angle_final.x = acos(angle.x);
        angle_final.y = acos(angle.y);
        angle_final.z = acos(angle.z);

        if(c.ddot(angle) != 0)
        {
            cv::Point3d angle1 = a.cross(c);
            angle1 /= sqrt(pow(angle1.x,2) + pow(angle1.y,2) + pow(angle1.z,2));
            angle1.x = acos(angle1.x);
            angle1.y = acos(angle1.y);
            angle1.z = acos(angle1.z);
            cv::Point3d angle2 = b.cross(c);
            angle2 /= sqrt(pow(angle2.x,2) + pow(angle2.y,2) + pow(angle2.z,2));
            angle2.x = acos(angle2.x);
            angle2.y = acos(angle2.y);
            angle2.z = acos(angle2.z);

            angle_final += angle1;
            angle_final += angle2;
            angle_final /= 3;
        }
        angle_final.y -= PI/2;
 
        return std::make_pair(position, angle_final);
    }
}
