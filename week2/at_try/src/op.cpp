//单通道图像遍历

#include <opencv2/opencv.hpp>
using namespace cv;

int main(int argc, char** argv)
{
    //创建图像picture,默认为纯黑色（即每个点像素值为0）
    Mat picture(200, 200, CV_8UC1);

    //显示图像
    imshow("picture",picture);

    //遍历像素点，将所有点的值修改为150
    for(int i=0;i<200;i++)
    {
        for(int j=0; j < 200; j++)
        {
            picture.at<uchar>(i,j) = 150;
        }
    }

imshow("changed",picture);

waitKey(0);

return 0;
}

