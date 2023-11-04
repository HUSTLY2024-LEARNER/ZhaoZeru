#include <opencv2/opencv.hpp>
using namespace cv;

int main()
//void rgb_hsv(void)
{
  //读取原图像
    Mat img = imread("../1.jpg", IMREAD_COLOR);
 
    Mat img_hsv;
    cvtColor(img, img_hsv, COLOR_BGR2HSV);  //将RGB图像转换为HSV图像
 
    Mat img_rgb;
    cvtColor(img_hsv, img_rgb, COLOR_HSV2BGR);   //将HSV图像转换为RGB图像

    resize(img_rgb, img_rgb,Size(img_rgb.cols / 4, img_rgb.rows / 4), INTER_AREA);
    resize(img_hsv, img_hsv,Size(img_hsv.cols / 4, img_hsv.rows / 4), INTER_AREA);
    
    imshow("ori rgb", img);
    imshow("hsv", img_hsv);
    imshow("rgb", img_rgb);
    waitKey(0);
    return 0;
}
