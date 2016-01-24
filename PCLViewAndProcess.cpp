
#include "PCLViewAndProcess.h"
#include <time.h>
// °ïÖú

void printUsage(const char* progName)
{
	std::cout << "\n\nUsage: " << progName << " [options]\n\n"
		<< "Options:\n"
		<< "-------------------------------------------\n"
		<< "-h           this help\n"
		<< "-s           Simple visualisation example\n"
		<< "-r           RGB colour visualisation example\n"
		<< "-c           Custom colour visualisation example\n"
		<< "-n           Normals visualisation example\n"
		<< "-a           Shapes visualisation example\n"
		<< "-v           Viewports example\n"
		<< "-i           Interaction Customization example\n"
		<< "\n\n";
}

ViewerAndProcesser::ViewerAndProcesser() :
point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>),
basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>),
point_cloud_stereo_ptr(new pcl::PointCloud<pcl::PointXYZRGB>),
viewerl(new pcl::visualization::PCLVisualizer("3D Viewer")),
countTime(0),
countFrame(0),
countCall(0)
{
}

ViewerAndProcesser::~ViewerAndProcesser()
{
}


int  ViewerAndProcesser::runManual()
{
	viewerl->setBackgroundColor(0, 0, 0);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> single_color(basicWithNorm_cloud_ptr, 0, 255, 0);
	viewerl->addPointCloud<pcl::PointNormal>(basicWithNorm_cloud_ptr, single_color, "this cloud");
	viewerl->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "this cloud");

	viewerl->addCoordinateSystem(1.0);
	viewerl->initCameraParameters();

	return 1;
}

int  ViewerAndProcesser::runRGBManual()
{
	viewerl->setBackgroundColor(0, 0, 0);

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb3(point_cloud_ptr);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb4(point_cloud_stereo_ptr);

	viewerl->addPointCloud<pcl::PointXYZRGB>(point_cloud_ptr, rgb3, "this cloud");
	viewerl->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "this cloud");

	viewerl->addPointCloud<pcl::PointXYZRGB>(point_cloud_stereo_ptr, rgb4, "stereo cloud");
	viewerl->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "stereo cloud");

	viewerl->addCoordinateSystem(1.0);
	viewerl->initCameraParameters();

	return 1;
}



int ViewerAndProcesser::blashManual()
{
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> single_color(basicWithNorm_cloud_ptr, 0, 255, 0);
	viewerl->updatePointCloud<pcl::PointNormal >(basicWithNorm_cloud_ptr, single_color, "this cloud");

	return 1;
}

int ViewerAndProcesser::blashRGBManual()
{
	countFrame++;
	char name1[50];
	char name2[50];
	sprintf(name1,"cloud%d",countFrame);
	sprintf(name2, "sloud%d", countFrame);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb3(point_cloud_ptr);
	viewerl->addPointCloud<pcl::PointXYZRGB >(point_cloud_ptr, rgb3, name1);

	cout << "kinect has: " << point_cloud_ptr->size() << " points." << endl;
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb4(point_cloud_stereo_ptr);
	viewerl->addPointCloud<pcl::PointXYZRGB >(point_cloud_stereo_ptr, rgb4, name2);
	cout << "stereo has: " << point_cloud_stereo_ptr->size() << " points." << endl;


	return 1;
}

int ViewerAndProcesser::saveBasicCloud()
{
	char path1[100];
	char path2[100];

	pcdCountBasic++;
	sprintf(path1,"data/kinect_basic%d.pcd",pcdCountBasic);
	sprintf(path2, "data/stereo_basic%d.pcd", pcdCountBasic);
	pcl::io::savePCDFileASCII(path1, *basic_cloud_ptr);
	pcl::io::savePCDFileASCII(path2, *point_cloud_stereo_ptr);
	std::cout << "Saved " << basic_cloud_ptr->points.size() << " basic points to :" <<path1<< "."<<std::endl;
	std::cout << "Saved " << point_cloud_stereo_ptr->points.size() << " basic points to :" << path2 << "." << std::endl;

	return 1;
}

int ViewerAndProcesser::saveColorCloud()
{
	char path1[100];
	char path2[100];
	pcdCountColor++; 
	sprintf(path1, "data/kinect_color%d.pcd", pcdCountColor);
	sprintf(path2, "data/stereo_color%d.pcd", pcdCountColor);

	pcl::io::savePCDFileASCII(path1, *point_cloud_ptr);
	pcl::io::savePCDFileASCII(path2, *point_cloud_stereo_ptr);

	std::cout << "Saved " << point_cloud_ptr->points.size() << " colored points to :" << path1 << "." << std::endl;
	return 1;
}

int ViewerAndProcesser::saveNormCloud()
{
	char path1[100];
	char path2[100];
	pcdCountNorm++;
	sprintf(path1, "data/pcd_norm%d.pcd", pcdCountNorm);
	/*sprintf(path2, "data/pcd_norm%d.pcd", pcdCountNorm);*/
	pcl::io::savePCDFileASCII(path1, *cloud_normals1);
	std::cout << "Saved " << cloud_normals1->points.size() << " Norm points to :" << path1<< "." << std::endl;
	return 1;
}

int ViewerAndProcesser::saveBasicWithNormCloud()
{
	char path[100];
	pcdCountBasicWithNorm++;
	sprintf(path, "data/pcd_BasicWithNorm%d.pcd", pcdCountBasicWithNorm);
	pcl::io::savePCDFileASCII(path, *basicWithNorm_cloud_ptr);
	std::cout << "Saved " << basicWithNorm_cloud_ptr->points.size() << "	basicWithNorm points to: " << path << "." << std::endl;

	return 1;
}

int ViewerAndProcesser::saveColorWithNormCloud()
{
	char path[100];
	pcdCountColorWithNorm++;
	sprintf(path, "data/pcd_colorAndNormal%d.pcd", pcdCountColorWithNorm);
	pcl::io::savePCDFileASCII(path, *pointWithNorm_cloud_ptr);
	std::cout << "Saved " << pointWithNorm_cloud_ptr->points.size() << " Norm points to :" << path << "." << std::endl;
	return 1;
}
