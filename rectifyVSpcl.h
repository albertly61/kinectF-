#ifndef __RECTIFY_H__
#define __RECTIFY_H__
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace pcl;
using namespace std;

class recvspcl
{
	
public:
	recvspcl();
	~recvspcl();
	int colorCloud(Mat& img, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloudPCL);
	int vilizeCloud();
	int  saveCloud( );
	int rectify(Mat &mat1, Mat& mat2);
	int readarg(const char* id1, const char* id2);
	int project3D(Mat& dis);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloudPCL;
private:
	Mat xyz;
	int cloudCount;
	bool cloudSave;
	Rect roi1, roi2;
	Mat Q;
	Mat M1, D1, M2, D2;                                        //双相机内参
	Mat R, R11, T, R1, P1, R2, P2;                        //双相机外参
	pcl::visualization::PCLVisualizer::Ptr viewerl;
};
#endif