 
#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include "elas.h"
#include "rectifyVSpcl.h"


using namespace std;
using namespace cv;

class stereo_camera
{

public:
	stereo_camera();
	~stereo_camera();
	int	openCamera();
	int	getTwoPic(Mat & mat1, Mat & mat2);
	int	getFullPic(Mat & mat);
	int    prossNew(Mat & mat1, Mat& mat2, Mat &dis, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloudPCL);

	recvspcl recpcl;
private:
	Mat frame,dis;
	VideoCapture cap;
};

