

#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include <deque>
#include <memory>
#include <string>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

using namespace std;
using namespace cv;

// ================================================================
//  常量配置区 —— 只改这里就能适配你的箱子
// ================================================================
static const float BOX_MM      = 350.0f;      // 箱子顶面边长 mm
static const float HALF        = BOX_MM / 2.f;
static const double MIN_AREA   = 8000.0;       // 最小有效红色面积(像素)
static const int   SMOOTH_N    = 6;            // 均值平滑帧数

// 物体坐标系：顶面中心为原点，单位 mm
static const vector<Point3f> OBJ_PTS = {
    {-HALF, -HALF, 0},  // 0 左上
    { HALF, -HALF, 0},  // 1 右上
    { HALF,  HALF, 0},  // 2 右下
    {-HALF,  HALF, 0}   // 3 左下
};

// ===== USB摄像头标定参数（你的参数）=====
static const Mat K = (Mat_<double>(3,3) <<
    786.19375828781722, 0.,                  668.98017421012958,
    0.,                 791.8946129798486,   373.97215705020159,
    0.,                 0.,                  1.);

static const Mat D = (Mat_<double>(1,5) <<
     0.06420428268786578,
     0.083133304993222509,
     0.0019411199567928366,
    -0.0032091194694991478,
    -0.34311667376016997);

// ================================================================
//  卡尔曼滤波器封装（对每个角点单独建一个 4 状态 KF：[x, y, vx, vy]）
// ================================================================
struct CornerKF
{
    KalmanFilter kf;
    bool         initialized = false;
    
    void init(Point2f pt)
    {
        kf.init(4, 2, 0, CV_64F);
        
        // 状态转移矩阵 A
        setIdentity(kf.transitionMatrix);
        kf.transitionMatrix.at<double>(0,2) = 1.0;
        kf.transitionMatrix.at<double>(1,3) = 1.0;
        
        // 测量矩阵 H
        kf.measurementMatrix = Mat::zeros(2, 4, CV_64F);
        kf.measurementMatrix.at<double>(0,0) = 1.0;
        kf.measurementMatrix.at<double>(1,1) = 1.0;
        
        // 过程噪声 Q
        setIdentity(kf.processNoiseCov, Scalar(0.05));
        
        // 测量噪声 R —— 这个值越大，越平滑但越滞后
        setIdentity(kf.measurementNoiseCov, Scalar(0.8));
        
        // 初始后验误差
        setIdentity(kf.errorCovPost, Scalar(1));
        
        // 初始状态
        kf.statePost = (Mat_<double>(4,1) << pt.x, pt.y, 0, 0);
        initialized = true;
    }
    
    Point2f update(Point2f measured)
    {
        if (!initialized) init(measured);
        
        // 预测
        Mat pred = kf.predict();
        
        // 测量
        Mat meas = (Mat_<double>(2,1) << measured.x, measured.y);
        
        // 更新
        Mat est = kf.correct(meas);
        return Point2f((float)est.at<double>(0), (float)est.at<double>(1));
    }
    
    Point2f predictOnly()
    {
        Mat pred = kf.predict();
        return Point2f((float)pred.at<double>(0), (float)pred.at<double>(1));
    }
};

// ================================================================
//  工具函数
// ================================================================

// 4点排序：返回 [左上, 右上, 右下, 左下]
vector<Point2f> sortCorners(const vector<Point2f>& in)
{
    vector<Point2f> r(4);
    vector<float> s(4), d(4);
    for (int i = 0; i < 4; i++) { 
        s[i] = in[i].x + in[i].y; 
        d[i] = in[i].x - in[i].y; 
    }
    
    int tl=0, br=0, tr=0, bl=0;
    for (int i = 1; i < 4; i++)
    {
        if (s[i] < s[tl]) tl = i;
        if (s[i] > s[br]) br = i;
        if (d[i] > d[tr]) tr = i;
        if (d[i] < d[bl]) bl = i;
    }
    r[0]=in[tl]; r[1]=in[tr]; r[2]=in[br]; r[3]=in[bl];
    return r;
}

// 旋转矩阵 → 欧拉角（ZYX，deg）
Vec3d euler(const Mat& R)
{
    double sy = sqrt(R.at<double>(0,0)*R.at<double>(0,0) +
                     R.at<double>(1,0)*R.at<double>(1,0));
    bool sg = sy < 1e-6;
    double x = sg ? atan2(-R.at<double>(1,2), R.at<double>(1,1))
                  : atan2( R.at<double>(2,1), R.at<double>(2,2));
    double y = atan2(-R.at<double>(2,0), sy);
    double z = sg ? 0 : atan2(R.at<double>(1,0), R.at<double>(0,0));
    return Vec3d(x * 180/CV_PI, y * 180/CV_PI, z * 180/CV_PI);
}

// 文字辅助：带黑色描边，可读性更好
void putLabel(Mat& img, const string& text, Point org,
              double scale=0.65, Scalar color=Scalar(0,255,255), int thick=2)
{
    putText(img, text, org, FONT_HERSHEY_SIMPLEX, scale, Scalar(0,0,0), thick+2);
    putText(img, text, org, FONT_HERSHEY_SIMPLEX, scale, color, thick);
}

// ================================================================
//  核心：从彩色帧里提取箱子四角
//  返回 false 表示本帧检测失败
// ================================================================
bool detectBoxCorners(const Mat& frame,
                      vector<Point2f>& corners, Mat& debugMask)
{
    // ---- 1. HSV 红色分割 ----
    Mat hsv;
    cvtColor(frame, hsv, COLOR_BGR2HSV);
    
    Mat m1, m2, mask;
    inRange(hsv, Scalar(0,  80, 60), Scalar(10,  255, 255), m1);
    inRange(hsv, Scalar(165, 80, 60), Scalar(180, 255, 255), m2);
    mask = m1 | m2;
    
    // ---- 2. 形态学：先闭运算填洞，再开运算去噪 ----
    Mat k5 = getStructuringElement(MORPH_RECT, Size(5,5));
    Mat k3 = getStructuringElement(MORPH_RECT, Size(3,3));
    morphologyEx(mask, mask, MORPH_CLOSE, k5, Point(-1,-1), 2);
    morphologyEx(mask, mask, MORPH_OPEN,  k3, Point(-1,-1), 1);
    
    debugMask = mask.clone();
    
    // ---- 3. 找轮廓，取最大面积 ----
    vector<vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    if (contours.empty()) return false;
    
    int best = 0;
    double maxA = 0;
    for (int i = 0; i < (int)contours.size(); i++)
    {
        double a = contourArea(contours[i]);
        if (a > maxA) { maxA = a; best = i; }
    }
    if (maxA < MIN_AREA) return false;
    
    // ---- 4. 凸包 → 多边形逼近 → 强制4点 ----
    vector<Point> hull;
    convexHull(contours[best], hull);
    
    // 多边形逼近，epsilon 从小到大，直到拿到 <=5 点或 epsilon 超限
    vector<Point> poly;
    double peri = arcLength(hull, true);
    for (double eps = 0.02; eps <= 0.15; eps += 0.01)
    {
        approxPolyDP(hull, poly, eps * peri, true);
        if ((int)poly.size() <= 5) break;
    }
    
    // 不管逼近结果如何，最终用 minAreaRect 兜底拿稳定的4点
    RotatedRect rr = minAreaRect(hull);
    Point2f rpts[4];
    rr.points(rpts);
    
    // 如果逼近到恰好4点且是凸的，就用逼近的结果；否则用旋转矩形
    vector<Point2f> raw(4);
    if ((int)poly.size() == 4 && isContourConvex(poly))
    {
        for (int i = 0; i < 4; i++) raw[i] = Point2f((float)poly[i].x, (float)poly[i].y);
    }
    else
    {
        for (int i = 0; i < 4; i++) raw[i] = rpts[i];
    }
    
    // ---- 5. 排序 [左上 右上 右下 左下] ----
    raw = sortCorners(raw);
    
    // ---- 6. 亚像素精化 ----
    Mat gray;
    cvtColor(frame, gray, COLOR_BGR2GRAY);
    for (auto& p : raw)
    {
        p.x = clamp(p.x, 0.f, (float)(frame.cols - 1));
        p.y = clamp(p.y, (float)0, (float)(frame.rows - 1));
    }
    TermCriteria tc(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.01);
    cornerSubPix(gray, raw, Size(7, 7), Size(-1, -1), tc);
    
    corners = raw;
    return true;
}

// ================================================================
//  距离 / 位姿平滑（对 tvec 做滑动均值）
// ================================================================
struct PoseSmootherVec3
{
    deque<Vec3d> buf;
    int maxN;
    
    explicit PoseSmootherVec3(int n) : maxN(n) {}
    
    Vec3d push(Vec3d v)
    {
        buf.push_back(v);
        if ((int)buf.size() > maxN) buf.pop_front();
        Vec3d sum(0,0,0);
        for (auto& x : buf) sum += x;
        return sum * (1.0 / buf.size());
    }
};

// ================================================================
//  main
// ================================================================
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("pnp_ros_node");
    auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

    const int camera_index = node->declare_parameter<int>("camera_index", 4);
    const int frame_width = node->declare_parameter<int>("frame_width", 1280);
    const int frame_height = node->declare_parameter<int>("frame_height", 720);
    const int camera_fps = node->declare_parameter<int>("camera_fps", 60);
    const string camera_frame_id = node->declare_parameter<string>("camera_frame_id", "camera_link");
    const string object_frame_id = node->declare_parameter<string>("object_frame_id", "target_object");
    const bool show_mask = node->declare_parameter<bool>("show_mask", true);

    // ---------- 启动 USB 摄像头 ----------
    VideoCapture cap;
    const std::array<int, 3> backends = {CAP_V4L2, CAP_ANY, CAP_GSTREAMER};
    bool opened = false;
    for (int backend : backends) {
        cap.release();
        if (!cap.open(camera_index, backend)) {
            cerr << "后端 " << backend << " 打开摄像头索引 " << camera_index
                 << " 失败\n";
            continue;
        }

        std::string backend_name = "unknown";
        try {
            backend_name = cap.getBackendName();
        } catch (...) {
            backend_name = "unavailable";
        }
        cout << "摄像头打开成功，索引=" << camera_index
             << "，后端=" << backend_name << "\n";
        opened = true;
        break;
    }

    if (!opened || !cap.isOpened()) {
        cerr << "无法打开摄像头，请检查设备索引和占用情况\n";
        rclcpp::shutdown();
        return -1;
    }
    
    cap.set(CAP_PROP_FRAME_WIDTH, frame_width);
    cap.set(CAP_PROP_FRAME_HEIGHT, frame_height);
    cap.set(CAP_PROP_FPS, camera_fps);

    RCLCPP_INFO(node->get_logger(),
                "camera_index=%d, frame=%dx%d, fps=%d, tf=%s->%s",
                camera_index,
                frame_width,
                frame_height,
                camera_fps,
                camera_frame_id.c_str(),
                object_frame_id.c_str());
    
    cout << "K:\n" << K << "\nD:\n" << D << "\n";
    
    // 计算去畸变映射
    Mat frame_tmp;
    cap >> frame_tmp;
    if (frame_tmp.empty()) {
        cerr << "摄像头已打开，但读取首帧失败\n";
        cap.release();
        rclcpp::shutdown();
        return -1;
    }
    Size imgSize = frame_tmp.size();
    
    Mat newK = getOptimalNewCameraMatrix(K, D, imgSize, 0.0, imgSize);
    Mat mapX, mapY;
    initUndistortRectifyMap(K, D, Mat(), newK, imgSize, CV_32FC1, mapX, mapY);
    
    // ---------- 卡尔曼：4 个角点各一个 ----------
    array<CornerKF, 4> kfs;
    
    // ---------- tvec 平滑 ----------
    PoseSmootherVec3 tvecSmoother(SMOOTH_N);
    
    Mat zeroDist = Mat::zeros(1, 5, CV_64F);
    
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);

        // ----- 取帧 -----
        Mat frame;
        cap >> frame;
        if (frame.empty()) break;
        
        // 去畸变
        Mat undistorted;
        remap(frame, undistorted, mapX, mapY, INTER_LINEAR);
        
        Mat show = undistorted.clone();
        Mat debugMask;
        vector<Point2f> rawCorners;
        
        bool detected = detectBoxCorners(undistorted, rawCorners, debugMask);
        
        // ----- 卡尔曼平滑角点 -----
        vector<Point2f> smoothCorners(4);
        if (detected)
        {
            for (int i = 0; i < 4; i++)
                smoothCorners[i] = kfs[i].update(rawCorners[i]);
        }
        else
        {
            // 检测失败，纯预测
            for (int i = 0; i < 4; i++)
            {
                if (kfs[i].initialized)
                    smoothCorners[i] = kfs[i].predictOnly();
                else
                    smoothCorners[i] = Point2f(0,0);
            }
        }
        
        bool anyInited = kfs[0].initialized;
        
        // ----- 画角点框 -----
        if (anyInited)
        {
            for (int i = 0; i < 4; i++)
            {
                Point2f a = smoothCorners[i];
                Point2f b = smoothCorners[(i+1)%4];
                
                // 绿色边框
                line(show, a, b, Scalar(0, 220, 0), 3, LINE_AA);
                
                // 角点圆
                circle(show, a, 7, Scalar(0, 0, 255), -1, LINE_AA);
                
                // 角点编号
                putLabel(show, to_string(i), a + Point2f(8, -8),
                        0.65, Scalar(255, 255, 0));
            }
            
            // ----- solvePnP -----
            Mat rvec, tvec;
            bool pnp_ok = solvePnP(OBJ_PTS, smoothCorners, newK, zeroDist,
                                   rvec, tvec, false, SOLVEPNP_IPPE);
            if (pnp_ok)
            {
                // ---- tvec 平滑 ----
                Vec3d tv(tvec.at<double>(0),
                        tvec.at<double>(1),
                        tvec.at<double>(2));
                Vec3d tvSmooth = tvecSmoother.push(tv);
                
                double X    = tvSmooth[0];
                double Y    = tvSmooth[1];
                double Z    = tvSmooth[2];
                double dist = sqrt(X*X + Y*Y + Z*Z);
                
                // ---- 旋转矩阵 / 欧拉角 ----
                Mat R;
                Rodrigues(rvec, R);
                Vec3d eu = euler(R);

                geometry_msgs::msg::TransformStamped transformStamped;
                transformStamped.header.stamp = node->get_clock()->now();
                transformStamped.header.frame_id = camera_frame_id;
                transformStamped.child_frame_id = object_frame_id;
                transformStamped.transform.translation.x = X / 1000.0;
                transformStamped.transform.translation.y = Y / 1000.0;
                transformStamped.transform.translation.z = Z / 1000.0;

                tf2::Matrix3x3 tf_rot(
                    R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
                    R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
                    R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));
                tf2::Quaternion q;
                tf_rot.getRotation(q);
                transformStamped.transform.rotation.x = q.x();
                transformStamped.transform.rotation.y = q.y();
                transformStamped.transform.rotation.z = q.z();
                transformStamped.transform.rotation.w = q.w();
                tf_broadcaster->sendTransform(transformStamped);
                
                // ---- 投影坐标轴（用平滑后的 tvec） ----
                Mat tvecSmoothed = (Mat_<double>(3,1) << X, Y, Z);
                
                vector<Point3f> axisPts = {
                    {0,    0,    0},
                    {100,  0,    0},   // X 红
                    {0,    100,  0},   // Y 绿
                    {0,    0,    100}  // Z 蓝（朝向相机方向）
                };
                vector<Point2f> axisImg;
                projectPoints(axisPts, rvec, tvecSmoothed, newK, zeroDist, axisImg);
                
                // 画坐标轴
                arrowedLine(show, axisImg[0], axisImg[1], Scalar(0,   0,   255), 3, LINE_AA, 0, 0.2);
                arrowedLine(show, axisImg[0], axisImg[2], Scalar(0,   255, 0  ), 3, LINE_AA, 0, 0.2);
                arrowedLine(show, axisImg[0], axisImg[3], Scalar(255, 0,   0  ), 3, LINE_AA, 0, 0.2);
                
                // 坐标轴标签
                putLabel(show, "X", axisImg[1] + Point2f(5, 0),  0.6, Scalar(0,   0,   255));
                putLabel(show, "Y", axisImg[2] + Point2f(5, 0),  0.6, Scalar(0,   255, 0  ));
                putLabel(show, "Z", axisImg[3] + Point2f(5, 0),  0.6, Scalar(255, 100, 0  ));
                
                // ---- 投影箱子中心点 ----
                vector<Point2f> centerImg;
                projectPoints(vector<Point3f>{{0,0,0}},
                             rvec, tvecSmoothed, newK, zeroDist, centerImg);
                if (!centerImg.empty())
                {
                    // 十字准星
                    int cx = (int)centerImg[0].x;
                    int cy = (int)centerImg[0].y;
                    line(show, Point(cx-14, cy),   Point(cx+14, cy),   Scalar(0,255,255), 2, LINE_AA);
                    line(show, Point(cx, cy-14),   Point(cx, cy+14),   Scalar(0,255,255), 2, LINE_AA);
                    circle(show, centerImg[0], 5, Scalar(0,255,255), -1, LINE_AA);
                }
                
                // ---- 信息面板（左上角半透明背景） ----
                {
                    // 画半透明黑底
                    Mat overlay = show.clone();
                    rectangle(overlay, Point(10, 10), Point(500, 185), Scalar(0,0,0), FILLED);
                    addWeighted(overlay, 0.45, show, 0.55, 0, show);
                    
                    int bx = 20, by = 35;
                    int dy = 30;
                    char buf[256];
                    
                    // 距离（最重要，大字）
                    sprintf(buf, "Distance : %.1f mm", dist);
                    putLabel(show, buf, Point(bx, by),
                            0.78, Scalar(0, 255, 100), 2);
                    
                    // XYZ
                    sprintf(buf, "X=%.1f  Y=%.1f  Z=%.1f  (mm)", X, Y, Z);
                    putLabel(show, buf, Point(bx, by + dy),
                            0.62, Scalar(0, 220, 255));
                    
                    // 欧拉角
                    sprintf(buf, "Roll=%.1f  Pitch=%.1f  Yaw=%.1f  (deg)",
                           eu[0], eu[1], eu[2]);
                    putLabel(show, buf, Point(bx, by + dy*2),
                            0.62, Scalar(255, 200, 0));
                    
                    // 检测状态
                    string status = detected ? "[ DETECT: OK ]" : "[ DETECT: LOST - KF predict ]";
                    Scalar  scol  = detected ? Scalar(0,255,0) : Scalar(0,100,255);
                    putLabel(show, status, Point(bx, by + dy*3),
                            0.62, scol);
                    
                    // 角点像素坐标
                    sprintf(buf, "Corners(px): (%.0f,%.0f) (%.0f,%.0f) (%.0f,%.0f) (%.0f,%.0f)",
                           smoothCorners[0].x, smoothCorners[0].y,
                           smoothCorners[1].x, smoothCorners[1].y,
                           smoothCorners[2].x, smoothCorners[2].y,
                           smoothCorners[3].x, smoothCorners[3].y);
                    putLabel(show, buf, Point(bx, by + dy*4),
                            0.52, Scalar(180, 180, 180));
                    
                    // 控制台输出
                    printf("\rDist=%.1fmm  XYZ=[%.1f, %.1f, %.1f]mm  "
                          "RPY=[%.1f, %.1f, %.1f]deg   ",
                          dist, X, Y, Z, eu[0], eu[1], eu[2]);
                    fflush(stdout);
                }
            }
        }
        
        // ---- 显示 ----
        // 把 mask 缩小显示在右下角，不遮挡主画面
        if (show_mask && !debugMask.empty())
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
        
        char key = (char)waitKey(1);
        if (key == 27) break;  // ESC 退出
        
        // 按 r：重置卡尔曼
        if (key == 'r' || key == 'R')
        {
            for (auto& kf : kfs) kf.initialized = false;
            tvecSmoother.buf.clear();
            cout << "\n[INFO] Kalman reset.\n";
        }
    }
    
    cap.release();
    destroyAllWindows();
    rclcpp::shutdown();
    cout << "\n[INFO] Exit.\n";
    return 0;
}











