#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "robot_interfaces/action/catch.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include <array>
#include <deque>
#include <cmath>
#include <thread>
#include <atomic>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace std;
using namespace cv;

// ================================================================
//  常量配置区
// ================================================================
static const float BOX_MM = 350.0f;
static const float HALF = BOX_MM / 2.f;
static const double MIN_AREA = 8000.0;
static const int SMOOTH_N = 6;

static const vector<Point3f> OBJ_PTS = {
    {-HALF, -HALF, 0},  // 0 左上
    { HALF, -HALF, 0},  // 1 右上
    { HALF,  HALF, 0},  // 2 右下
    {-HALF,  HALF, 0}   // 3 左下
};

static const Mat K = (Mat_<double>(3,3) <<
    786.19375828781722, 0., 668.98017421012958,
    0., 791.8946129798486, 373.97215705020159,
    0., 0., 1.);

static const Mat D = (Mat_<double>(1,5) <<
    0.06420428268786578, 0.083133304993222509,
    0.0019411199567928366, -0.0032091194694991478,
    -0.34311667376016997);

// ================================================================
//  卡尔曼滤波器封装
// ================================================================
struct CornerKF {
    KalmanFilter kf;
    bool initialized = false;

    void init(Point2f pt) {
        kf.init(4, 2, 0, CV_64F);
        setIdentity(kf.transitionMatrix);
        kf.transitionMatrix.at<double>(0,2) = 1.0;
        kf.transitionMatrix.at<double>(1,3) = 1.0;

        kf.measurementMatrix = Mat::zeros(2, 4, CV_64F);
        kf.measurementMatrix.at<double>(0,0) = 1.0;
        kf.measurementMatrix.at<double>(1,1) = 1.0;

        setIdentity(kf.processNoiseCov, Scalar(0.05));
        setIdentity(kf.measurementNoiseCov, Scalar(0.8));
        setIdentity(kf.errorCovPost, Scalar(1));

        kf.statePost = (Mat_<double>(4,1) << pt.x, pt.y, 0, 0);
        initialized = true;
    }

    Point2f update(Point2f measured) {
        if (!initialized) init(measured);
        Mat pred = kf.predict();
        Mat meas = (Mat_<double>(2,1) << measured.x, measured.y);
        Mat est = kf.correct(meas);
        return Point2f((float)est.at<double>(0), (float)est.at<double>(1));
    }

    Point2f predictOnly() {
        Mat pred = kf.predict();
        return Point2f((float)pred.at<double>(0), (float)pred.at<double>(1));
    }
};

// ================================================================
//  工具函数
// ================================================================
vector<Point2f> sortCorners(const vector<Point2f>& in) {
    vector<Point2f> r(4);
    vector<float> s(4), d(4);
    for (int i = 0; i < 4; i++) { 
        s[i] = in[i].x + in[i].y; 
        d[i] = in[i].x - in[i].y; 
    }
    int tl=0, br=0, tr=0, bl=0;
    for (int i = 1; i < 4; i++) {
        if (s[i] < s[tl]) tl = i;
        if (s[i] > s[br]) br = i;
        if (d[i] > d[tr]) tr = i;
        if (d[i] < d[bl]) bl = i;
    }
    r[0]=in[tl]; r[1]=in[tr]; r[2]=in[br]; r[3]=in[bl];
    return r;
}

Vec3d euler(const Mat& R) {
    double sy = sqrt(R.at<double>(0,0)*R.at<double>(0,0) +
                     R.at<double>(1,0)*R.at<double>(1,0));
    bool sg = sy < 1e-6;
    double x = sg ? atan2(-R.at<double>(1,2), R.at<double>(1,1))
                  : atan2( R.at<double>(2,1), R.at<double>(2,2));
    double y = atan2(-R.at<double>(2,0), sy);
    double z = sg ? 0 : atan2(R.at<double>(1,0), R.at<double>(0,0));
    return Vec3d(x * 180/CV_PI, y * 180/CV_PI, z * 180/CV_PI);
}

void putLabel(Mat& img, const string& text, Point org,
              double scale=0.65, Scalar color=Scalar(0,255,255), int thick=2) {
    putText(img, text, org, FONT_HERSHEY_SIMPLEX, scale, Scalar(0,0,0), thick+2);
    putText(img, text, org, FONT_HERSHEY_SIMPLEX, scale, color, thick);
}

// ================================================================
//  核心：从彩色帧里提取箱子四角
// ================================================================
bool detectBoxCorners(const Mat& frame, vector<Point2f>& corners, Mat& debugMask) {
    Mat hsv;
    cvtColor(frame, hsv, COLOR_BGR2HSV);
    
    Mat m1, m2, mask;
    inRange(hsv, Scalar(0,  80, 60), Scalar(10,  255, 255), m1);
    inRange(hsv, Scalar(165, 80, 60), Scalar(180, 255, 255), m2);
    mask = m1 | m2;
    
    Mat k5 = getStructuringElement(MORPH_RECT, Size(5,5));
    Mat k3 = getStructuringElement(MORPH_RECT, Size(3,3));
    morphologyEx(mask, mask, MORPH_CLOSE, k5, Point(-1,-1), 2);
    morphologyEx(mask, mask, MORPH_OPEN,  k3, Point(-1,-1), 1);
    
    debugMask = mask.clone();
    
    vector<vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    if (contours.empty()) return false;
    
    int best = 0;
    double maxA = 0;
    for (int i = 0; i < (int)contours.size(); i++) {
        double a = contourArea(contours[i]);
        if (a > maxA) { maxA = a; best = i; }
    }
    if (maxA < MIN_AREA) return false;
    
    vector<Point> hull;
    convexHull(contours[best], hull);
    
    vector<Point> poly;
    double peri = arcLength(hull, true);
    for (double eps = 0.02; eps <= 0.15; eps += 0.01) {
        approxPolyDP(hull, poly, eps * peri, true);
        if ((int)poly.size() <= 5) break;
    }
    
    RotatedRect rr = minAreaRect(hull);
    Point2f rpts[4];
    rr.points(rpts);
    
    vector<Point2f> raw(4);
    if ((int)poly.size() == 4 && isContourConvex(poly)) {
        for (int i = 0; i < 4; i++) raw[i] = Point2f((float)poly[i].x, (float)poly[i].y);
    } else {
        for (int i = 0; i < 4; i++) raw[i] = rpts[i];
    }
    
    raw = sortCorners(raw);
    
    Mat gray;
    cvtColor(frame, gray, COLOR_BGR2GRAY);
    for (auto& p : raw) {
        p.x = clamp(p.x, 0.f, (float)(frame.cols - 1));
        p.y = clamp(p.y, 0.f, (float)(frame.rows - 1));
    }
    TermCriteria tc(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.01);
    cornerSubPix(gray, raw, Size(7, 7), Size(-1, -1), tc);
    
    corners = raw;
    return true;
}

// ================================================================
//  距离 / 位姿平滑
// ================================================================
struct PoseSmootherVec3 {
    deque<Vec3d> buf;
    int maxN;
    
    explicit PoseSmootherVec3(int n) : maxN(n) {}
    
    Vec3d push(Vec3d v) {
        buf.push_back(v);
        if ((int)buf.size() > maxN) buf.pop_front();
        Vec3d sum(0,0,0);
        for (auto& x : buf) sum += x;
        
        return sum * (1.0 / buf.size());
    }
};

// ================================================================
//  ROS 2 节点
// ================================================================
class BoxPnPNode : public rclcpp::Node {
public:
    using Catch = robot_interfaces::action::Catch;
    using GoalHandleCatch = rclcpp_action::ClientGoalHandle<Catch>;

    BoxPnPNode() : Node("box_pnp_node") {
        RCLCPP_INFO(this->get_logger(), "BoxPnPNode 启动");

        // 修复：取消注释以初始化 TF 监听器，否则后续 transform 会崩溃
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("box_pose", 10);
        
        action_client_ = rclcpp_action::create_client<robot_interfaces::action::Catch>(this, "robotic_task");

        RCLCPP_INFO(this->get_logger(), "正在打开摄像头...");
        if (!open_camera_with_fallback()) {
            RCLCPP_ERROR(this->get_logger(), "无法打开摄像头，请检查 /dev/video0 权限或设备占用");
            rclcpp::shutdown();
            return;
        }

        configure_camera_properties();

        cout << "K:\n" << K << "\nD:\n" << D << "\n";

        Mat frame_tmp;
        for (int i = 0; i < 30; ++i) {
            cap_ >> frame_tmp;
            if (!frame_tmp.empty()) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if (frame_tmp.empty()) {
            RCLCPP_ERROR(this->get_logger(), "摄像头已打开但连续取帧失败，请确认驱动和分辨率设置");
            rclcpp::shutdown();
            return;
        }

        Size imgSize = frame_tmp.size();
        newK_ = getOptimalNewCameraMatrix(K, D, imgSize, 0.0, imgSize);
        initUndistortRectifyMap(K, D, Mat(), newK_, imgSize, CV_32FC1, mapX_, mapY_);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(33),
                                         std::bind(&BoxPnPNode::process_frame, this));
        pose_publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&BoxPnPNode::publish_available_pose, this));
        
        RCLCPP_INFO(this->get_logger(), "节点初始化完成");
    }

    ~BoxPnPNode() {
        cap_.release();
        cv::destroyAllWindows();
    }

private:
    void publish_available_pose() {
        if (!has_valid_pose_) {
            return;
        }
        available_pose_.header.stamp = this->now();
        pose_publisher_->publish(available_pose_);
    }

    bool is_pose_valid(const geometry_msgs::msg::Pose& pose) const {
        const auto finite = [](double v) { return std::isfinite(v); };
        const bool position_ok = finite(pose.position.x) && finite(pose.position.y) && finite(pose.position.z);
        const bool quat_ok = finite(pose.orientation.x) && finite(pose.orientation.y) &&
                             finite(pose.orientation.z) && finite(pose.orientation.w);
        if (!position_ok || !quat_ok) {
            return false;
        }
        const double qn = std::sqrt(
            pose.orientation.x * pose.orientation.x +
            pose.orientation.y * pose.orientation.y +
            pose.orientation.z * pose.orientation.z +
            pose.orientation.w * pose.orientation.w);
        return qn > 1e-6;
    }

    bool should_send_goal(const geometry_msgs::msg::Pose& pose, const rclcpp::Time& now) {
        if (!is_pose_valid(pose)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "跳过发送：位姿无效");
            return false;
        }

        if (has_last_goal_send_time_) {
            const auto dt = now - last_goal_send_time_;
            if (dt < rclcpp::Duration::from_seconds(min_goal_send_interval_sec_)) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                    "跳过发送：节流中");
                return false;
            }
        }

        if (has_last_sent_pose_) {
            const double dx = pose.position.x - last_sent_pose_.position.x;
            const double dy = pose.position.y - last_sent_pose_.position.y;
            const double dz = pose.position.z - last_sent_pose_.position.z;
            const double pos_delta = std::sqrt(dx * dx + dy * dy + dz * dz);

            const double dot = std::abs(
                pose.orientation.x * last_sent_pose_.orientation.x +
                pose.orientation.y * last_sent_pose_.orientation.y +
                pose.orientation.z * last_sent_pose_.orientation.z +
                pose.orientation.w * last_sent_pose_.orientation.w);
            const double clamped_dot = std::clamp(dot, 0.0, 1.0);
            const double angle_delta = 2.0 * std::acos(clamped_dot);
        }

        return true;
    }

    bool open_camera_with_fallback() {
        const std::array<int, 3> backends = {CAP_V4L2, CAP_ANY, CAP_GSTREAMER};
        for (int backend : backends) {
            cap_.release();
            if (!cap_.open(4, backend)) {
                RCLCPP_WARN(this->get_logger(), "摄像头打开失败，backend=%d", backend);
                continue;
            }
            std::string backend_name = "unknown";
            try {
                backend_name = cap_.getBackendName();
            } catch (...) {
                backend_name = "unavailable";
            }
            RCLCPP_INFO(this->get_logger(), "摄像头打开成功，backend=%d (%s)",
                        backend, backend_name.c_str());
            return true;
        }
        return false;
    }

    void configure_camera_properties() {
        cap_.set(CAP_PROP_FRAME_WIDTH, 1280);
        cap_.set(CAP_PROP_FRAME_HEIGHT, 720);
        cap_.set(CAP_PROP_FPS, 60);
        cap_.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
        cap_.set(CAP_PROP_BUFFERSIZE, 1);
    }

    bool reopen_camera_if_needed() {
        if (empty_frame_count_ < 30) {
            return false;
        }
        RCLCPP_WARN(this->get_logger(), "连续空帧过多，尝试重连摄像头...");
        if (!open_camera_with_fallback()) {
            RCLCPP_ERROR(this->get_logger(), "重连摄像头失败");
            return false;
        }
        configure_camera_properties();
        empty_frame_count_ = 0;
        return true;
    }

    void process_frame() {
        Mat frame;
        cap_ >> frame;
        if (frame.empty()) {
            ++empty_frame_count_;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                 "读取到空帧（count=%d）", empty_frame_count_);
            reopen_camera_if_needed();
            return;
        }
        empty_frame_count_ = 0;

        // 去畸变
        Mat undistorted;
        remap(frame, undistorted, mapX_, mapY_, INTER_LINEAR);
        Mat show = undistorted.clone();

        Mat debugMask;
        vector<Point2f> rawCorners;
        bool detected = detectBoxCorners(undistorted, rawCorners, debugMask);

        // ----- 卡尔曼平滑角点 -----
        vector<Point2f> smoothCorners(4);
        if (detected) {
            for (int i = 0; i < 4; i++)
                smoothCorners[i] = kfs_[i].update(rawCorners[i]);
        } else {
            for (int i = 0; i < 4; i++) {
                if (kfs_[i].initialized)
                    smoothCorners[i] = kfs_[i].predictOnly();
                else
                    smoothCorners[i] = Point2f(0,0);
            }
        }

        bool anyInited = kfs_[0].initialized;

        // 至少曾经检测到一次角点
        if (anyInited) {
            // ----- 画角点框 -----
            for (int i = 0; i < 4; i++) {
                Point2f a = smoothCorners[i];
                Point2f b = smoothCorners[(i+1)%4];
                line(show, a, b, Scalar(0, 220, 0), 3, LINE_AA);
                circle(show, a, 7, Scalar(0, 0, 255), -1, LINE_AA);
                putLabel(show, to_string(i), a + Point2f(8, -8), 0.65, Scalar(255, 255, 0));
            }

            // ----- solvePnP -----
            Mat rvec, tvec;
            Mat zeroDist = Mat::zeros(1, 5, CV_64F);
            bool pnp_ok = solvePnP(OBJ_PTS, smoothCorners, newK_, zeroDist,
                                   rvec, tvec, false, SOLVEPNP_IPPE);

            if (pnp_ok) {
                Vec3d tv(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
                Vec3d tvSmooth = tvecSmoother_.push(tv);
                
                double X = tvSmooth[0];
                double Y = tvSmooth[1];
                double Z = tvSmooth[2];
                double dist = sqrt(X*X + Y*Y + Z*Z);

                Mat R;
                Rodrigues(rvec, R);
                Vec3d eu = euler(R);

                Mat tvecSmoothed = (Mat_<double>(3,1) << X, Y, Z);
                vector<Point3f> axisPts = {
                    {0,   0,   0},
                    {100, 0,   0},
                    {0,   100, 0},
                    {0,   0,   100}
                };
                vector<Point2f> axisImg;
                projectPoints(axisPts, rvec, tvecSmoothed, newK_, zeroDist, axisImg);

                arrowedLine(show, axisImg[0], axisImg[1], Scalar(0,   0,   255), 3, LINE_AA, 0, 0.2);
                arrowedLine(show, axisImg[0], axisImg[2], Scalar(0,   255, 0  ), 3, LINE_AA, 0, 0.2);
                arrowedLine(show, axisImg[0], axisImg[3], Scalar(255, 0,   0  ), 3, LINE_AA, 0, 0.2);

                putLabel(show, "X", axisImg[1] + Point2f(5, 0), 0.6, Scalar(0,   0,   255));
                putLabel(show, "Y", axisImg[2] + Point2f(5, 0), 0.6, Scalar(0,   255, 0  ));
                putLabel(show, "Z", axisImg[3] + Point2f(5, 0), 0.6, Scalar(255, 100, 0  ));

                vector<Point2f> centerImg;
                projectPoints(vector<Point3f>{{0,0,0}}, rvec, tvecSmoothed, newK_, zeroDist, centerImg);
                if (!centerImg.empty()) {
                    int cx = (int)centerImg[0].x;
                    int cy = (int)centerImg[0].y;
                    line(show, Point(cx-14, cy), Point(cx+14, cy), Scalar(0,255,255), 2, LINE_AA);
                    line(show, Point(cx, cy-14), Point(cx, cy+14), Scalar(0,255,255), 2, LINE_AA);
                    circle(show, centerImg[0], 5, Scalar(0,255,255), -1, LINE_AA);
                }

                {
                    Mat overlay = show.clone();
                    rectangle(overlay, Point(10, 10), Point(500, 185), Scalar(0,0,0), FILLED);
                    addWeighted(overlay, 0.45, show, 0.55, 0, show);

                    int bx = 20, by = 35, dy = 30;
                    char buf[256];

                    sprintf(buf, "Distance : %.1f mm", dist);
                    putLabel(show, buf, Point(bx, by), 0.78, Scalar(0, 255, 100), 2);

                    sprintf(buf, "X=%.1f  Y=%.1f  Z=%.1f  (mm)", X, Y, Z);
                    putLabel(show, buf, Point(bx, by + dy), 0.62, Scalar(0, 220, 255));

                    sprintf(buf, "Roll=%.1f  Pitch=%.1f  Yaw=%.1f  (deg)", eu[0], eu[1], eu[2]);
                    putLabel(show, buf, Point(bx, by + dy*2), 0.62, Scalar(255, 200, 0));

                    string status = detected ? "[ DETECT: OK ]" : "[ DETECT: LOST - KF predict ]";
                    Scalar scol = detected ? Scalar(0,255,0) : Scalar(0,100,255);
                    putLabel(show, status, Point(bx, by + dy*3), 0.62, scol);

                    sprintf(buf, "Corners(px): (%.0f,%.0f) (%.0f,%.0f) (%.0f,%.0f) (%.0f,%.0f)",
                            smoothCorners[0].x, smoothCorners[0].y,
                            smoothCorners[1].x, smoothCorners[1].y,
                            smoothCorners[2].x, smoothCorners[2].y,
                            smoothCorners[3].x, smoothCorners[3].y);
                    putLabel(show, buf, Point(bx, by + dy*4), 0.52, Scalar(180, 180, 180));

                }

                // 构造 camera_optical_frame 坐标系下的位姿
                geometry_msgs::msg::PoseStamped pose_camera;
                // 使用零时间戳请求最新可用 TF，避免仿真时钟和墙钟不一致导致外推失败
                pose_camera.header.stamp = builtin_interfaces::msg::Time();
                pose_camera.header.frame_id = "camera_optical_frame"; 
                pose_camera.pose.position.x = tvSmooth[0] / 1000.0; 
                pose_camera.pose.position.y = tvSmooth[1] / 1000.0;
                pose_camera.pose.position.z = tvSmooth[2] / 1000.0;

                
                tf2::Matrix3x3 tf2_rot(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
                                       R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
                                       R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));
                tf2::Quaternion q;
                tf2_rot.getRotation(q);
                pose_camera.pose.orientation.x = q.x();
                pose_camera.pose.orientation.y = q.y();
                pose_camera.pose.orientation.z = q.z();
                pose_camera.pose.orientation.w = q.w();

                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    3000,
                    "PnP目标位姿(camera_optical_frame): Pos(%.3f, %.3f, %.3f), Rot(%.3f, %.3f, %.3f, %.3f)",
                    pose_camera.pose.position.x,
                    pose_camera.pose.position.y,
                    pose_camera.pose.position.z,
                    pose_camera.pose.orientation.w,
                    pose_camera.pose.orientation.x,
                    pose_camera.pose.orientation.y,
                    pose_camera.pose.orientation.z
                );


                // 尝试 TF 转换到 base_link
                try {
                    // geometry_msgs::msg::TransformStamped t;
                    // t.header.stamp = this->now();
                    // t.header.frame_id = "camera_optical_frame";
                    // t.child_frame_id = "object_frame";
                    // t.transform.translation.x = pose_camera.pose.position.x;
                    // t.transform.translation.y = pose_camera.pose.position.y;
                    // t.transform.translation.z = pose_camera.pose.position.z;
                    // t.transform.rotation = pose_camera.pose.orientation;
                    // tf_broadcaster_.sendTransform(t);

                    geometry_msgs::msg::PoseStamped pose_base = tf_buffer_->transform(
                        pose_camera, "base_link", tf2::durationFromSec(0.1));
                    available_pose_ = pose_base;
                    available_pose_.header.stamp = this->now();
                    has_valid_pose_ = true;
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                     "pnp 结算出来 base_link 下的位姿:Pos(%lf, %lf, %lf) Rot(%lf, %lf, %lf, %lf)",
                     pose_base.pose.position.x,
                     pose_base.pose.position.y,
                     pose_base.pose.position.z,
                     pose_base.pose.orientation.w,
                     pose_base.pose.orientation.x,
                     pose_base.pose.orientation.y,
                     pose_base.pose.orientation.z);
                    
                    
                    if (should_send_goal(pose_base.pose, this->now())) {
                        send_catch_goal(pose_base.pose);
                    }
                    
                } catch (tf2::TransformException &ex) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                       "TF 转换失败：%s", ex.what());
                }
            }
        }

        // ---- 显示 mask 在右下角 ----
        {
            Mat maskBGR, maskSmall;
            cvtColor(debugMask, maskBGR, COLOR_GRAY2BGR);
            resize(maskBGR, maskSmall, Size(320, 180));
            int ox = show.cols - 325;
            int oy = show.rows - 185;
            maskSmall.copyTo(show(Rect(ox, oy, maskSmall.cols, maskSmall.rows)));
            putLabel(show, "RedMask", Point(ox+5, oy+18), 0.55, Scalar(200,200,200));
        }

        imshow("Box PnP", show);
        waitKey(1);
    }

    bool send_catch_goal(const geometry_msgs::msg::Pose& target_pose) {
        ++goal_send_attempt_count_;
        if (goal_in_flight_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "已有抓取目标在途，等待当前任务完成后再发送");
            return false;
        }
        if (!action_server_ready_) {
            if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "Action Server 未连接，等待重试");
                return false;
            }
            action_server_ready_ = true;
            RCLCPP_INFO(this->get_logger(), "Action Server 已连接");
        }

        if (!is_pose_valid(target_pose)) {
            RCLCPP_WARN(this->get_logger(), "目标位姿无效，取消发送");
            return false;
        }

        auto goal_msg = Catch::Goal();
        goal_msg.target_pose = target_pose;
        goal_msg.action_type = 2;

        RCLCPP_INFO(this->get_logger(), "发送抓取目标 attempt=%d", goal_send_attempt_count_);
        auto send_goal_options = rclcpp_action::Client<Catch>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&BoxPnPNode::goal_response_cb, this, std::placeholders::_1);
        send_goal_options.feedback_callback = std::bind(&BoxPnPNode::feedback_cb, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = std::bind(&BoxPnPNode::result_cb, this, std::placeholders::_1);
        action_client_->async_send_goal(goal_msg, send_goal_options);

        goal_in_flight_ = true;

        last_goal_send_time_ = this->now();
        has_last_goal_send_time_ = true;
        last_sent_pose_ = target_pose;
        has_last_sent_pose_ = true;
        return true;
    }

    void goal_response_cb(std::shared_ptr<GoalHandleCatch> handle) {
        if (!handle) {
            ++goal_reject_count_;
            RCLCPP_WARN(this->get_logger(), "目标被拒绝 reject_count=%d", goal_reject_count_);
            goal_in_flight_ = false;
            return;
        }
        ++goal_accept_count_;
        RCLCPP_INFO(this->get_logger(), "目标已被接受 accept_count=%d", goal_accept_count_);
    }

    void feedback_cb(
        std::shared_ptr<GoalHandleCatch> /*unused*/,
        const std::shared_ptr<const Catch::Feedback> feedback)
    {
        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        //                      "收到反馈: state=%d 描述=%s",
        //                      feedback->current_state,
        //                      feedback->state_describe.c_str());
    }

    void result_cb(const GoalHandleCatch::WrappedResult &result) {
        // RCLCPP_INFO(this->get_logger(), "抓取任务完成: %s", result.result->reason.c_str());
        goal_in_flight_ = false;
    }

    VideoCapture cap_;
    Mat newK_, mapX_, mapY_;
    array<CornerKF, 4> kfs_;
    PoseSmootherVec3 tvecSmoother_{SMOOTH_N};
    rclcpp_action::Client<Catch>::SharedPtr action_client_;
    rclcpp::TimerBase::SharedPtr timer_;


    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_{this};

    bool has_valid_pose_ = false;
    bool action_server_ready_ = false;

    int goal_send_attempt_count_ = 0;
    int goal_accept_count_ = 0;
    int goal_reject_count_ = 0;

    double min_goal_send_interval_sec_ = 0.6;
    double goal_pos_threshold_m_ = 0.015;
    double goal_angle_threshold_rad_ = 5.0 * CV_PI / 180.0;

    bool has_last_goal_send_time_ = false;
    rclcpp::Time last_goal_send_time_{0, 0, RCL_ROS_TIME};

    bool has_last_sent_pose_ = false;
    geometry_msgs::msg::Pose last_sent_pose_;

    std::atomic_bool goal_in_flight_{false};






    geometry_msgs::msg::PoseStamped available_pose_;
    rclcpp::TimerBase::SharedPtr pose_publish_timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;








    int empty_frame_count_ = 0;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BoxPnPNode>());
    rclcpp::shutdown();
    return 0;
}

