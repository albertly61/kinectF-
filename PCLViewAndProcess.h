
#ifndef _PCLVP_
#define _PCLVP_

#include <iostream>
#include "pcl_header.h"


class ViewerAndProcesser
{
	int pcdCountBasic = 0, pcdCountColor = 0, pcdCountNorm = 0, pcdCountBasicWithNorm = 0, pcdCountColorWithNorm = 0;
public:
	ViewerAndProcesser();
	~ViewerAndProcesser();

	int countFrame;

	pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr;
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1;
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pointWithNorm_cloud_ptr;
	pcl::PointCloud<pcl::PointNormal>::Ptr basicWithNorm_cloud_ptr;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_stereo_ptr;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewerl;

	int run();
	int runManual();
	DWORD  blash();
	int blashManual();
	int	blashRGBManual();
	int  runRGBManual();
	int setParam(pcl::PointCloud<pcl::PointXYZ>::Ptr& basic_cloud_ptrl, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& point_cloud_ptrl,
		pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals1l);
	int saveBasicCloud();
	int saveColorCloud();
	int saveNormCloud();
	int saveColorWithNormCloud();
	int saveBasicWithNormCloud();
private:

	static DWORD WINAPI ThreadProc(LPVOID lpParameter);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2;


	///time
	int countTime;
	LONG countCall;

};

#endif 