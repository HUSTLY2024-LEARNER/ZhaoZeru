#include <opencv2/opencv.hpp>
using namespace cv;

int main()
{
    Mat srcImg = imread("../1.jpg", IMREAD_COLOR);
    Mat dstImg ;

    pyrDown(srcImg, dstImg, Size((srcImg.cols + 1) / 2 , (srcImg.rows + 1) / 2), BORDER_DEFAULT);

    imshow("src",srcImg);
    imshow("dst",dstImg);
    waitKey(0);
    return 0;
}