// Reference project
// what you need to do:
// Replace what labeled /*<var>*/ with proper 

//#include "opencv2/opencv.hpp"
#include "opencv2/opencv.hpp"
#include "../include/segment.h"
#include <iostream>

Mat frame,imgThresholded;       
vector<Point2f> point_img;
int iLowH = 0;
int rect_min = 0,rect_max = 0;
void segment_track(int,void *);
void contours_min_track(int,void *);
void contours_max_track(int,void *);
void segment_callback(int ,void*);
using namespace std;
int main(int, char**)
{
	namedWindow("origin", 0);
	namedWindow("threshold",0);
	namedWindow("contours",0);
        frame = imread("/home/zero/opencv_t/segment/test_img.jpg");

        if (!frame.empty()) {
                //std::cout << "Original Image Size: " << frame.size() << std::endl;
               	cv::imshow("origin", frame);
		cv::waitKey(0);

		cv::resize(frame, frame, cv::Size(), 1, 1);
                cv::imshow("origin", frame);
                cv::waitKey(0);
        } else {
                // 图像加载失败
                std::cerr << "Failed to load image." << std::endl;
        }

	createTrackbar("阈值：","threshold", &iLowH, 255, segment_track);
	createTrackbar("min: ","contours", &rect_min, 300000, contours_min_track);
	createTrackbar("max: ","contours", &rect_max, 300000, contours_max_track);

        imgThresholded  = segment_img(frame);

	imshow("threshold", imgThresholded);

        // =======your code here===========
        // get contours, and filter the contours
        vector<vector<Point> > contours;
        vector<int> squar_index;
	//contours_filter(/*<var>*/, /*<var>*/, /*<var>*/);
        contours_filter(imgThresholded, contours, squar_index);
        // ========= end ==================

        // pick point here
	point_img = pick_point(imgThresholded, contours, squar_index);
        if(point_img.empty()){
                return 0;
        }

        // visual the result
	cout << point_img.size() << endl;
        visual_point(frame,point_img);

        waitKey();
	imwrite("threshold.png", imgThresholded);
    	return 0;
}

void segment_track(int,void *)
{
    imgThresholded = segment_img(frame);
    imshow("threshold",imgThresholded);
    vector<vector<Point> > contours;
    vector<int> squar_index;
    contours_filter(imgThresholded, contours, squar_index);
    point_img = pick_point(imgThresholded,contours, squar_index);
    visual_point(frame,point_img);
}


void contours_min_track(int,void *)
{
    vector<vector<Point> > contours;
    vector<int> squar_index;
    contours_filter(imgThresholded, contours, squar_index);
    point_img = pick_point(imgThresholded,contours, squar_index);
    visual_point(frame,point_img);

}

void contours_max_track(int,void *)
{
    vector<vector<Point> > contours;
    vector<int> squar_index;
    contours_filter(imgThresholded, contours, squar_index);
    point_img = pick_point(imgThresholded,contours, squar_index);
    visual_point(frame,point_img);
}

