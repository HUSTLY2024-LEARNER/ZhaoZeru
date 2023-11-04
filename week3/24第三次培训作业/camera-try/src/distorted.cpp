 #include <opencv2/opencv.hpp>
 #include <string>
 using namespace std;
 string image_file = "../../distorted.png"; // 请 确 保 路 径 正 确

 int main(int argc, char **argv) {
 // 本 程 序 实 现 去 畸 变 部 分 的 代 码。 尽 管 我 们 可 以 调 用OpenCV的 去 畸 变， 但 自 己 实 现 一 遍 有 助 于 理 解。
// 畸 变 参 数
  double k1 = -0.28340811, k2 = 0.07395907, p1 = 0.00019359, p2 = 1.76187114e-05;
 // 内 参
 double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375;

 cv::Mat image = cv::imread(image_file, 0); // 图 像 是 灰 度 图，CV_8UC1
 int rows = image.rows, cols = image.cols;
 cv::Mat image_undistort = cv::Mat(rows, cols, CV_8UC1); // 去 畸 变 以 后 的 图

 // 计 算 去 畸 变 后 图 像 的 内 容
 for (int v = 0; v < rows; v++) {
 for (int u = 0; u < cols; u++) {
 // 按 照 公 式， 计 算 点(u,v)对 应 到 畸 变 图 像 中 的 坐 标(u_distorted, v_distorted)
 double x = (u - cx) / fx, y = (v - cy) / fy;  //像素点对应的真实图像中的那个点距离光心的横纵坐标
 double r = sqrt(x * x + y * y);  //该点到中心的距离
 double x_distorted = x * (1 + k1 * r * r + k2 * r * r * r * r) + 2 * p1 * x * y + p2 * (r * r + 2 * x * x);
 double y_distorted = y * (1 + k1 * r * r + k2 * r * r * r * r) + p1 * (r * r + 2 * y * y) + 2 * p2 * x * y;
 double u_distorted = fx * x_distorted + cx;
 double v_distorted = fy * y_distorted + cy;

 // 赋 值 (最 近 邻 插 值)
 if (u_distorted >= 0 && v_distorted >= 0 && u_distorted < cols && v_distorted < rows) {
 image_undistort.at<uchar>(v, u) = image.at<uchar>((int) v_distorted, (int) u_distorted);
 } else {
 image_undistort.at<uchar>(v, u) = 0;
 }
 }
 }

 // 画 图 去 畸 变 后 图 像
 cv::imshow("distorted", image);
 cv::imshow("undistorted", image_undistort);
 cv::waitKey();
 return 0;

 }