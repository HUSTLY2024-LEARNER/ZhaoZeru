#include "../include/segment.h"
extern int iLowH;
extern int rect_min,rect_max;
Mat segment_img(Mat &src) 
{
    Mat srcImage=src.clone();
    Mat imgHSV;
    Mat imgThresholded;
    cvtColor(srcImage, imgHSV, COLOR_BGR2HSV);  //BGR转换为HSV空间
    vector<Mat> hsvSplit;
    split(imgHSV, hsvSplit);
    // 直方图均衡化  明度
    equalizeHist(hsvSplit[2], hsvSplit[2]);
    merge(hsvSplit, imgHSV);
    // int iLowH = 1, iLowS = 85, iLowV = 0, iHighH = 60, iHighS = 255, iHighV = 255;//TODO hsv空间黄色范围
    // int iLowS = 0, iLowV = 0, iHighH = 48, iHighS = 255, iHighV = 255;
    int iLowH = 20, iLowS = 100, iLowV = 100, iHighH = 30, iHighS = 255, iHighV = 255;
    inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);//开操作，先腐蚀后膨胀
    morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);//闭操作，先膨胀后腐蚀
    return imgThresholded;
}

void contours_filter(Mat &imgThresholded,vector<vector<Point> > &contours,vector<int> &squar_index)
{
    vector<Vec4i> hierarchy;//“向量内每一个元素包含了4个int型变量”的向量
    findContours(imgThresholded, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_NONE);//求轮廓　
    //检测所有的轮廓，但所有轮廓只建立两个等级关系，外围为顶层，
    //若外围内的内围轮廓还包含了其他的轮廓信息，则内围内的所有轮廓均归属于顶层
    //保存物体边界上所有连续的轮廓点到contours向量内
    //初步筛选符合要求的矩形
    // std::cout << contours.size() << std::endl;
    for (int i = 0; i < contours.size(); i++)
    {
        RotatedRect minrect = minAreaRect(contours[i]);//最小外接斜矩形
        Point2f center;
        float radius = 0;
        minEnclosingCircle(contours[i], center, radius);//求外接圆
        double circle_area = CV_PI * radius*radius;
        double rect_area = minrect.size.area();
        //排除点在边界的情况
        if (center.x < 10 || center.y < 10 || (imgThresholded.rows - center.y) < 10 || (imgThresholded.cols - center.x) < 10)
        {
            //cout << "第 " << i << "个不满足要求!" << endl;
            continue;
        }
        // std::cout << rect_area << std::endl;
        // if (rect_area >= rect_min && rect_area < rect_max)//排除面积小于1000的连通域
        // TODO: 修改的地方
        if (rect_area >= 1000)//排除面积小于1000的连通域
        {
            double ratio = rect_area / circle_area;
            // std::cout << ratio << std::endl;
            if(ratio >= 0.4 && ratio <= 0.9)
            {
                squar_index.push_back(i);//vector类型，插入数值i
            }
        }
    }
	
	Mat contour = Mat::zeros(imgThresholded.rows, imgThresholded.cols,CV_8SC3);//CV_8SC3 8位符号整型3通道矩阵
	drawContours(contour, contours, -1, Scalar(200,200,200), 2);//-1 代表画所有轮廓
	if(!squar_index.empty())
	{
		for(int i = 0;i<squar_index.size();i++)
			drawContours(contour, contours, squar_index[i], Scalar(255,0,0), 5);//contours所有点,squar_index[i]画其中的哪些点
	}	
	imshow("contours", contour);
    waitKey(0);
	imwrite("../contours.png", contour);
}	 

vector<Point2f> pick_point(Mat &srcImage,vector<vector<Point> > &contours,vector<int> squar_index)
{
    vector<Point2f> point_img;
    vector<vector<Point2f> > temp_img;
    // std::cout << contours.size() << std::endl;
    // std::cout << squar_index.size() << std::endl;
    for(size_t c=0;c<squar_index.size();c++){
        vector<Point> hull;
        
        // 求凸包
        convexHull(contours[squar_index[c]],hull);//凸包

        //pick four point
        vector<Point> squar;
        size_t num=hull.size();

        std::cout << num << std::endl;
        
        if(num<4)//顶点数小于３
            continue;
        else if(num>=4){
            float max_area;
            for(int m=0;m<num-3;m++){
                for(int n=m+1;n<num-2;n++){
                    for(int j=n+1;j<num-1;j++){
                        for(int k=j+1;k<num;k++){
                            vector<Point> squar_tmp;
                            squar_tmp.push_back(hull[m]);
                            squar_tmp.push_back(hull[n]);
                            squar_tmp.push_back(hull[j]);
                            squar_tmp.push_back(hull[k]);
                            if(m==0&&n==1&&j==2&&k==3){
                                max_area=fabs(contourArea(Mat(squar_tmp)));
                                squar.clear();
                                squar=squar_tmp;
                            }
                            else{
                                float area=fabs(contourArea(Mat(squar_tmp)));
                                if(area>max_area){
                                    max_area=area;
                                    squar.clear();
                                    squar=squar_tmp;
                                }
                            }
                        }
                    }
                }
            }
        }

        if(squar.size()!=4){//如果顶点数不等于４
            continue;
        }

        int num_board=0;
        for(int i=0;i<squar.size();i++) {
            num_board += (squar[i].x < 10) || (squar[i].x > srcImage.cols - 10) ||
                         (squar[i].y < 10) || (squar[i].y > srcImage.rows - 10);
        }
        if(num_board>0){    //点在边界
            continue;
        }
        //给四点排序
        vector<Point> squar_sort=squar;

        //sort_point(squar,squar_sort,srcImage);

        for(int i=0;i<squar_sort.size();i++){
            point_img.clear();
            for(size_t num_p=0;num_p<squar_sort.size();num_p++) {
                // point_img.push_back(squar_sort[num_p] * (1 / minifactor));
                point_img.push_back(squar_sort[num_p] );
            }
        }
        temp_img.push_back(point_img);
    }
    if(temp_img.size()==0){
        return vector<Point2f>();
    }

    if(temp_img.size()>2){
        point_img.clear();
        // cout<<"识别的框太多！"<<endl;
        return point_img;
    }

    if(temp_img.size()==2) {//如果轮廓数等于2
        double a1 = contourArea(temp_img[0], true);
        double a2 = contourArea(temp_img[1], true);
        // cout << "a1= " << a1 << endl << "a2= " << a2 << endl;
        if (a1 > a2) {
            point_img.clear();
            point_img = temp_img[0];
        }
    }
    vector<Point2f> point_temp=point_img;
    point_img.clear();
    point_img.push_back(point_temp[1]);
    point_img.push_back(point_temp[2]);
    point_img.push_back(point_temp[3]);
    point_img.push_back(point_temp[0]);
    return point_img;
}

void visual_point(Mat src,vector<Point2f> point_image)
{
    src = imread("../test_img.jpg");
	resize(src, src, src.size()/2);
	for (int i = 0;i < point_image.size();i++)
	{
		circle(src, point_image[i], 5, Scalar(255, 200, 0),3);
	}
	imshow("point", src);
}
