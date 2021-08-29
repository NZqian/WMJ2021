/**
 * 单目标定程序
 * 先运行TakePhoto拍摄一定数量的照片，建议30组，一组左右各一张
 * 再运行本程序，本程序关联文件../wmj_cfg/calib_img.xml(准备弃用wmj_cfg了，暂时先放这)
 * 可在calib_img.xml里修改照片数量
 */
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

// 获取文件名
bool readStringList( const std::string& filename, std::vector<std::string>& l )
{
    l.resize(0);
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if( !fs.isOpened() )
        return false;
    cv::FileNode n = fs.getFirstTopLevelNode();
    if( n.type() != cv::FileNode::SEQ )
        return false;
    cv::FileNodeIterator it = n.begin(), it_end = n.end();
    for( ; it != it_end; ++it )
        l.push_back((std::string)*it);
    fs.release();
    return true;
}

int main()
{

    std::string filePath = "../wmj_cfg/calib_img.xml";
    std::vector<std::string> fileList;

    if(!readStringList(filePath, fileList) || fileList.empty())
    {
        std::cout << "can not open " <<  filePath << " or the string list is empty, 泥给路达油~~\n";
        return 0;
    }

    //读取每一幅图像，从中提取出角点，然后对角点进行亚像素精确化
    std::cout << "开始提取角点………………";
    int image_count = 0;                                    // 图像数量
    cv::Size image_size;                                    // 图像的尺寸
    cv::Size board_size = cv::Size(9, 6);                   // 标定板上每行、列的角点数
    std::vector<cv::Point2f> image_points_buf;              // 缓存每幅图像上检测到的角点
    std::vector<std::vector<cv::Point2f>> image_points_seq; // 保存检测到的所有角点
    std::string filename;


    for(auto filename : fileList)
    {
        image_count++; // 用于观察检验输出
        std::cout << "image_count = " << image_count << "\n";
        cv::Mat imageInput = cv::imread(filename);
        if (image_count == 1)  //读入第一张图片时获取图像宽高信息
        {
            image_size.width = imageInput.cols;
            image_size.height = imageInput.rows;
            std::cout << "image_size.width = " << image_size.width << std::endl;
            std::cout << "image_size.height = " << image_size.height << std::endl;
        }

        // 提取角点
        if (0 == findChessboardCorners(imageInput, board_size, image_points_buf))
        {
            std::cout << "没找到棋盘角点，摸了!\n";
            exit(1);
        }
        else
        {
            cv::Mat view_gray;
            cv::cvtColor(imageInput, view_gray, CV_RGB2GRAY);
            find4QuadCornerSubpix(view_gray, image_points_buf, cv::Size(11, 11));     // 对粗提取的角点进行亚像素精确化
            image_points_seq.push_back(image_points_buf);                             // 保存亚像素角点
            drawChessboardCorners(view_gray, board_size, image_points_buf, true);     // 用于在图片中标记角点
            imshow("Camera Calibration", view_gray);
            cv::waitKey(100);                                                         // 暂停0.1S
        }
    }

    int total = image_points_seq.size();
    std::cout << "提取到的角点总数 = " << total << "\n角点提取完成！\n开始标定………………\n";

    //棋盘三维信息
    cv::Point2f square_size = cv::Point2f(2.4, 2.4);  // 实际测量得到的标定板上每个棋盘格的大小
    std::vector<std::vector<cv::Point3f>> object_points; // 保存标定板上角点的三维坐标

    // 内外参数
    cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0)); // 摄像机内参数矩阵
    cv::Mat distCoeffs   = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0)); // 摄像机的5个畸变系数：k1,k2,p1,p2,k3
    std::vector<int> point_counts;                                      // 每幅图像中角点的数量
    std::vector<cv::Mat> rvecsMat;                                      // 每幅图像的旋转向量
    std::vector<cv::Mat> tvecsMat;                                      // 每幅图像的平移向量

    // 初始化标定板上角点的三维坐标
    int i, j, t;
    for (t = 0; t < image_count; t++)
    {
        std::vector<cv::Point3f> tempPointSet;
        for (i = 0; i < board_size.height; i++)
            for (j = 0; j < board_size.width; j++)
            {
                cv::Point3f realPoint;
                // 假设标定板放在世界坐标系中z=0的平面上
                realPoint.x = i * square_size.x;
                realPoint.y = j * square_size.y;
                realPoint.z = 0;
                tempPointSet.push_back(realPoint);
            }
        object_points.push_back(tempPointSet);
    }

    // 初始化每幅图像中的角点数量，假定每幅图像中都可以看到完整的标定板
    for (i = 0; i < image_count; i++)
        point_counts.push_back(board_size.width * board_size.height);

    // 开始标定
    cv::calibrateCamera(object_points, image_points_seq, image_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, 0);
    std::cout << "标定完成！\n";

    //对标定结果进行评价
    std::cout << "开始评价标定结果………………\n";
    double total_err = 0.0; // 所有图像的平均误差的总和
    double err = 0.0; // 每幅图像的平均误差
    std::vector<cv::Point2f> image_points2; // 保存重新计算得到的投影点
    std::cout << "\t每幅图像的标定误差：\n";

    for (i = 0; i < image_count; i++)
    {
        std::vector<cv::Point3f> tempPointSet = object_points[i];

        // 通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点
        projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, image_points2);

        // 计算新的投影点和旧的投影点之间的误差
        std::vector<cv::Point2f> tempImagePoint = image_points_seq[i];
        cv::Mat tempImagePointMat = cv::Mat(1, tempImagePoint.size(), CV_32FC2);
        cv::Mat image_points2Mat = cv::Mat(1, image_points2.size(), CV_32FC2);

        for (int j = 0; j < tempImagePoint.size(); j++)
        {
            image_points2Mat.at<cv::Vec2f>(0, j)  = cv::Vec2f(image_points2[j].x, image_points2[j].y);
            tempImagePointMat.at<cv::Vec2f>(0, j) = cv::Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
        }
        err = cv::norm(image_points2Mat, tempImagePointMat, cv::NORM_L2);
        total_err += err /= point_counts[i];
        std::cout << "第" << i + 1 << "幅图像的平均误差：" << err << "像素\n";
    }
    std::cout << "总体平均误差：" << total_err / image_count << "像素\n评价完成！\n";

    //保存定标结果
    std::cout << "开始保存定标结果………………\n";
    cv::Mat rotation_matrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0)); // 保存每幅图像的旋转矩阵
    cv::FileStorage fs("calib_res.yaml", cv::FileStorage::WRITE);
    if(fs.isOpened())
        fs << "cameraMatrix" << cameraMatrix << "distCoeffs" << distCoeffs;

    for (int i = 0; i < image_count; i++)
    {
        fs << "img_" + std::to_string( i + 1) + "_rotVec" << rvecsMat[i];
        Rodrigues(rvecsMat[i], rotation_matrix); // 将旋转向量转换为相对应的旋转矩阵
        fs << "img_" + std::to_string( i + 1) + "_rotMat" << rotation_matrix;
        fs << "img_"  + std::to_string( i + 1) + "_tvec" << tvecsMat[i];
    }
    std::cout << "完成保存" << "\n";
    return 0;
}
