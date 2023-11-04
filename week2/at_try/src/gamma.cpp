#include <opencv2/opencv.hpp>
#include <iostream>
using namespace std;
using namespace cv;

int main()
{
    Mat img = imread("../1.jpg",IMREAD_COLOR);
    imshow("img",img);
    
    
    
    
    waitKey(0);



}