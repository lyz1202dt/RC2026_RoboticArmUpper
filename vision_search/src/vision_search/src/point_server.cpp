#include <memory>
#include <chrono>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "inference.h"
#include <librealsense2/rs.hpp>
#include "rclcpp/rclcpp.hpp"
#include "vision_search/srv/set_point.hpp"

using SetPoint = vision_search::srv::SetPoint;

// ------------------------ 结构体用于返回三维坐标和姿态 ------------------------
struct Pose3D
{
    float x = 0, y = 0, z = 0;          // 三维坐标
    float roll = 0, pitch = 0, yaw = 0; // 欧拉角
};

// ------------------------ RealSense 封装 ------------------------
class D435Capture
{
public:
    D435Capture(int width = 640, int height = 480, int fps = 60)
        : width_(width), height_(height), fps_(fps)
    {
        cfg_.enable_stream(RS2_STREAM_COLOR, width_, height_, RS2_FORMAT_BGR8, fps_);
        cfg_.enable_stream(RS2_STREAM_DEPTH, width_, height_, RS2_FORMAT_Z16, fps_);
        pipe_.start(cfg_);
    }
    ~D435Capture() { pipe_.stop(); }

    bool read(cv::Mat &colorFrame, cv::Mat &depthFrame)
    {
        try
        {
            rs2::frameset frames = pipe_.wait_for_frames();
            rs2::video_frame color = frames.get_color_frame();
            rs2::depth_frame depth = frames.get_depth_frame();
            if (!color || !depth)
                return false;

            colorFrame = cv::Mat(cv::Size(color.get_width(), color.get_height()),
                                 CV_8UC3, (void *)color.get_data(), cv::Mat::AUTO_STEP)
                             .clone();
            depthFrame = cv::Mat(cv::Size(depth.get_width(), depth.get_height()),
                                 CV_16UC1, (void *)depth.get_data())
                             .clone();
            return true;
        }
        catch (const rs2::error &e)
        {
            std::cerr << "RealSense 错误: " << e.what() << std::endl;
            return false;
        }
    }

    void getCameraIntrinsics(cv::Mat &cameraMatrix, cv::Mat &distCoeffs)
    {
        auto profile = pipe_.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
        auto intr = profile.get_intrinsics();
        cameraMatrix = (cv::Mat_<double>(3, 3) << intr.fx, 0, intr.ppx, 0, intr.fy, intr.ppy, 0, 0, 1);
        distCoeffs = cv::Mat::zeros(5, 1, CV_64F);

        fx_ = intr.fx;
        fy_ = intr.fy;
        ppx_ = intr.ppx;
        ppy_ = intr.ppy;
    }

    float getDepth(const cv::Mat &depthFrame, int x, int y)
    {
        x = std::clamp(x, 0, depthFrame.cols - 1);
        y = std::clamp(y, 0, depthFrame.rows - 1);
        return depthFrame.at<uint16_t>(y, x) * 0.001f;
    }

    cv::Point3f getPointXYZ(const cv::Mat &depthFrame, int x, int y)
    {
        float z = getDepth(depthFrame, x, y);
        float X = (x - ppx_) * z / fx_;
        float Y = (y - ppy_) * z / fy_;
        return cv::Point3f(X, Y, z);
    }

private:
    rs2::pipeline pipe_;
    rs2::config cfg_;
    int width_, height_, fps_;
    float fx_, fy_, ppx_, ppy_;
};

// ------------------------ PnP + 坐标计算函数 ------------------------
Pose3D computePosePnP(
    const cv::Mat &colorFrame,
    const cv::Mat &depthFrame,
    const cv::Rect &box,
    const cv::Mat &cameraMatrix,
    const cv::Mat &distCoeffs,
    D435Capture &cap)
{
    Pose3D pose;

    // PnP object points
    float objSize = 0.35f; // 根据实际物体尺寸调整
    std::vector<cv::Point3f> objectPoints = {{0, 0, 0}, {objSize, 0, 0}, {objSize, objSize, 0}, {0, objSize, 0}};
    std::vector<cv::Point2f> imagePoints = {
        {float(box.x), float(box.y)},
        {float(box.x + box.width), float(box.y)},
        {float(box.x + box.width), float(box.y + box.height)},
        {float(box.x), float(box.y + box.height)}};

    cv::Mat rvec, tvec;
    if (cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec))
    {
        cv::Mat R;
        cv::Rodrigues(rvec, R);

        // 欧拉角
        pose.roll = atan2(R.at<double>(2, 1), R.at<double>(2, 2)) * 180.0 / CV_PI;
        pose.pitch = atan2(-R.at<double>(2, 0),
                           sqrt(R.at<double>(2, 1) * R.at<double>(2, 1) + R.at<double>(2, 2) * R.at<double>(2, 2))) *
                     180.0 / CV_PI;
        pose.yaw = atan2(R.at<double>(1, 0), R.at<double>(0, 0)) * 180.0 / CV_PI;

        // 框中心三维坐标
        int centerX = box.x + box.width / 2;
        int centerY = box.y + box.height / 2;
        cv::Point3f xyz = cap.getPointXYZ(depthFrame, centerX, centerY);
        pose.x = xyz.x;
        pose.y = xyz.y;
        pose.z = xyz.z;
    }

    return pose;
}

// ------------------------ 服务端节点 ------------------------
class PointServerNode : public rclcpp::Node
{
public:
    PointServerNode()
        : Node("point_server_node"),
          cap_(640, 480, 60),
          inf_("/home/lzy/Desktop/project_yolo/c/cpp/best.onnx", cv::Size(640, 640),
               "/home/lzy/Desktop/project_yolo/c/cpp/classes.txt", true)
    {
        // RealSense 内参
        cap_.getCameraIntrinsics(cameraMatrix_, distCoeffs_);

        // 创建服务
        server_ = this->create_service<SetPoint>(
            "server_point",
            std::bind(&PointServerNode::handle_request, this,
                      std::placeholders::_1, std::placeholders::_2));

        // 实时读帧并显示
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&PointServerNode::update_frame, this));

        RCLCPP_INFO(this->get_logger(), "Service server started.");
    }

private:
    void update_frame()
    {
        if (!cap_.read(last_color_, last_depth_))
            return;

        if (last_color_.empty())
            return;

        cv::Mat frame = last_color_.clone(); // 复制用于绘制

        // YOLO 推理
        auto result = inf_.runInference(frame);

        cv::Point infoPos(10, 30);
        int objectCount = 0;

        for (const auto &det : result)
        {
            cv::rectangle(frame, det.box, det.color, 2);

            // 标签
            std::ostringstream label;
            label.precision(2);
            label << det.className << " " << det.confidence;
            int baseline = 0;
            cv::Size textSize = cv::getTextSize(label.str(), cv::FONT_HERSHEY_DUPLEX, 0.6, 1, &baseline);
            cv::rectangle(frame, cv::Point(det.box.x, det.box.y - textSize.height - 10),
                          cv::Point(det.box.x + textSize.width, det.box.y), det.color, cv::FILLED);
            cv::putText(frame, label.str(), cv::Point(det.box.x, det.box.y - 5),
                        cv::FONT_HERSHEY_DUPLEX, 0.6, cv::Scalar(0, 0, 0), 1);

            // 计算 PnP
            Pose3D pose = computePosePnP(frame, last_depth_, det.box, cameraMatrix_, distCoeffs_, cap_);

            // 坐标轴
            float axisLength = 0.35f;
            std::vector<cv::Point3f> axisPoints = {{0, 0, 0}, {axisLength, 0, 0}, {0, axisLength, 0}, {0, 0, axisLength}};
            std::vector<cv::Point2f> imageAxisPoints;
            cv::Mat rvec, tvec;
            cv::solvePnP(std::vector<cv::Point3f>{{0, 0, 0}, {axisLength, 0, 0}, {axisLength, axisLength, 0}, {0, axisLength, 0}},
                         std::vector<cv::Point2f>{{float(det.box.x), float(det.box.y)},
                                                  {float(det.box.x + det.box.width), float(det.box.y)},
                                                  {float(det.box.x + det.box.width), float(det.box.y + det.box.height)},
                                                  {float(det.box.x), float(det.box.y + det.box.height)}},
                         cameraMatrix_, distCoeffs_, rvec, tvec);
            cv::projectPoints(axisPoints, rvec, tvec, cameraMatrix_, distCoeffs_, imageAxisPoints);
            // 画坐标轴
            cv::line(frame, imageAxisPoints[0], imageAxisPoints[1], cv::Scalar(0, 0, 255), 3); // X 红
            cv::line(frame, imageAxisPoints[0], imageAxisPoints[2], cv::Scalar(0, 255, 0), 3); // Y 绿
            cv::line(frame, imageAxisPoints[0], imageAxisPoints[3], cv::Scalar(255, 0, 0), 3); // Z 蓝

            // 在轴末端标注 X/Y/Z
            int fontFace = cv::FONT_HERSHEY_SIMPLEX;
            double fontScale = 0.7;
            int thickness = 2;

            cv::putText(frame, "X", imageAxisPoints[1], fontFace, fontScale, cv::Scalar(0, 0, 255), thickness);
            cv::putText(frame, "Y", imageAxisPoints[2], fontFace, fontScale, cv::Scalar(0, 255, 0), thickness);
            cv::putText(frame, "Z", imageAxisPoints[3], fontFace, fontScale, cv::Scalar(255, 0, 0), thickness);
            // 显示前两个物体信息
            if (objectCount < 2)
            {
                std::ostringstream coordText;
                coordText.precision(2);
                coordText << "Pos: X=" << pose.x << " Y=" << pose.y << " Z=" << pose.z;
                cv::putText(frame, coordText.str(), infoPos, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2);

                infoPos.y += 25;
                std::ostringstream angleText;
                angleText.precision(2);
                angleText << "Ang: " << pose.roll << "," << pose.pitch << "," << pose.yaw;
                cv::putText(frame, angleText.str(), infoPos, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2);

                infoPos.y += 35;
                objectCount++;
            }

            // 显示框中心深度
            int centerX = det.box.x + det.box.width / 2;
            int centerY = det.box.y + det.box.height / 2;
            std::ostringstream distLabel;
            distLabel.precision(2);
            distLabel << pose.z << " m";
            cv::putText(frame, distLabel.str(), cv::Point(centerX, centerY), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
        }

        cv::imshow("RealSense YOLO + PnP", frame);
        cv::waitKey(1);
    }

    void handle_request(
        const std::shared_ptr<SetPoint::Request> request,
        std::shared_ptr<SetPoint::Response> response)
    {
        (void)request;

        if (last_color_.empty() || last_depth_.empty())
        {
            response->success = false;
            response->message = "No frame available";
            return;
        }

        auto result = inf_.runInference(last_color_);
        if (result.empty())
        {
            response->success = false;
            response->message = "No detection";
            return;
        }

        Pose3D pose = computePosePnP(last_color_, last_depth_, result[0].box, cameraMatrix_, distCoeffs_, cap_);

        response->x = pose.x;
        response->y = pose.y;
        response->z = pose.z;
        response->roll = pose.roll;
        response->pitch = pose.pitch;
        response->yaw = pose.yaw;
        response->success = true;
        response->message = "Point and pose returned successfully.";

        RCLCPP_INFO(this->get_logger(),
                    "Sent point: X=%.3f Y=%.3f Z=%.3f | Roll=%.2f Pitch=%.2f Yaw=%.2f",
                    pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw);
    }

    D435Capture cap_;
    Inference inf_;
    cv::Mat cameraMatrix_, distCoeffs_;
    cv::Mat last_color_, last_depth_;
    rclcpp::Service<SetPoint>::SharedPtr server_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// ------------------------ main ------------------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointServerNode>());
    rclcpp::shutdown();
    return 0;
}
