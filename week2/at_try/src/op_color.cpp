//利用at函数处理rgb图像
#include <stdlib.h>
#include <opencv2/opencv.hpp>
using namespace cv;
 
int main(int argc, char** argv)
{
 
	Mat picture(200, 200, CV_8UC3);//新建一个三通道彩色图像，初始默认为纯黑色
	imshow("picture", picture);	
 
	//遍历picture的每个像素点，并将每个通道所有像素点的值修改为0~255中随机一个整数值
	for (int i = 0; i < 200; i++)
	{
		for (int j = 0; j < 200; j++)
		{
 
			picture.at<Vec3b>(i, j)[0] = rand() % 255;//B通道
			picture.at<Vec3b>(i, j)[1] = rand() % 255;//G通道
			picture.at<Vec3b>(i, j)[2] = rand() % 255;//R通道
		}
 
	}
 
	imshow("dst", picture);
	waitKey(0);
	return 0;
 
}