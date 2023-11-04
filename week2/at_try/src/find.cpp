#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
using namespace cv;
using namespace std;

int main()
{
    //1.获得图片灰度图
    Mat img = imread("../pictures/3.png", IMREAD_COLOR); //读取图片
    Mat img_gray;
    cvtColor(img, img_gray,COLOR_RGB2GRAY); //转换BGR图片为灰度图
    //imshow("gray", img_gray); //展示灰度图

    //2.提取轮廓，用Sobel算子提取y方向
    Mat y_grad;
    Sobel(img_gray, y_grad, CV_16S, 0, 1, 3);
    convertScaleAbs(y_grad, y_grad);
    imshow("y_grad", y_grad);
    waitKey(0);

    //3.阈值处理,自适应二值化
    Mat binary;
    threshold(img_gray, binary, 127, 255, THRESH_OTSU);
    imshow("binary", binary);

    //4.闭运算处理，图像区域化，便于找到车牌区域，进而得到轮廓
    Mat kernel = getStructuringElement(MORPH_RECT, Size(20,6));
    Mat close;
    morphologyEx(binary, close, MORPH_CLOSE, kernel);
    imshow("close", close);

    //5.腐蚀/膨胀 去噪点得到车牌区域
    //横向 腐蚀+膨胀 
    Mat erodeX, dilateX;
    morphologyEx(close, erodeX, MORPH_ERODE, kernel);
    morphologyEx(erodeX, dilateX, MORPH_DILATE, kernel);
    imshow("dilateX",dilateX);
    
    //纵向腐蚀+膨胀
    Mat erodeY, dilateY;
    morphologyEx(dilateX, dilateY, MORPH_DILATE, kernel);
    morphologyEx(dilateY, erodeY, MORPH_ERODE, kernel);
    imshow("dilateY",dilateY);

    //6.获取外轮廓
    Mat img_f;
    Mat img_copy;
    img.copyTo(img_copy);

    vector<vector<cv::Point> > contours;
    vector<Vec4i> hierarchy;
    dilateY.copyTo(img_f);
    findContours(img_f, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    drawContours(img_copy, contours, -1, Scalar(255, 0, 255), 5, 8, hierarchy); //填BGR数值的时候要加Scalar！！！
    imshow("img",img_copy); //一定要在原图像上画边框！！！
    waitKey(0);

    //7.截取车牌
    Mat license;
    for(int i = 0; i < contours.size(); i++)
    {
        Rect box = boundingRect(contours[i]);
        double ratio = double(box.width) / double(box.height) ;
        if(ratio > 3.65 && ratio < 3.7)
        {
            Rect rect = boundingRect(contours[i]);
            license = img(rect);//ROI
            imshow("License",license);
            //imwrite("License",license);
            waitKey(0);
        }
    }

    if(license.size)
    {
        imwrite("license.png",license);
    }

    return 0;
}


