#include <opencv2/opencv.hpp>
using namespace cv;

int main()
{
    Mat srcImg = imread("../1.jpg", IMREAD_COLOR);
    Mat dstImg ;

    resize(srcImg,dstImg,Size(srcImg.cols / 4,srcImg.rows / 4),INTER_AREA);

    imshow("src",srcImg);
    imshow("dst",dstImg);
    waitKey(0);
    return 0;
}