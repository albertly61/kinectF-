/*
*  stereo_match.cpp
*  calibration
*
*  Created by Victor  Eruhimov on 1/18/10.
*  Copyright 2010 Argus Corp. All rights reserved.
*
*/

// tsu图像的用sgbm最好的配置：tsuR.png tsuL.png --algorithm=sgbm --max-disparity=32 --blocksize=1 --scale=1 时间为421ms，其分辨率为288*384
//car图像的用sgbm最好的配置：car0.png car1L.png --algorithm=sgbm --max-disparity=32 --blocksize=3 --scale=1 时间为519ms ,其分辨率为240*353
//用var方法比sgbm和什么方法效果好在其边缘没有较大错误，变化比较平缓，但是对于阴影会形成误判

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"

#include "opencv/cv.h"
#include "opencv/cvaux.h"
#include "opencv/cxcore.h"
#include "opencv/highgui.h"
//
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <iostream>
#include <stdio.h>

//#include "glRender1.h"

using namespace cv;
using namespace std;

static void print_help()
{
	printf("\nDemo stereo matching converting L and R images into disparity and point clouds\n");
	printf("\nUsage: stereo_match <left_image> <right_image> [--algorithm=bm|sgbm|hh|var] [--blocksize=<block_size>]\n"
		"[--max-disparity=<max_disparity>] [--scale=scale_factor>] [-i <intrinsic_filename>] [-e <extrinsic_filename>]\n"
		"[--no-display] [-o <disparity_image>] [-p <point_cloud_file>]\n");
}

static void saveXYZ(const char* filename, const Mat& mat, const Mat& img)
{
	const double max_z = 1.0e3;
	FILE* fp = fopen(filename, "wt");
	for (int y = 0; y < mat.rows; y++)
	{
		for (int x = 0; x < mat.cols; x++)
		{
			Vec3f point = mat.at<Vec3f>(y, x);
			const uchar* color = img.ptr<uchar>(y,x);
			if (fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
			fprintf(fp, "%f %f %f %c %c %c\n", point[0], point[1], point[2],color[0],color[1],color[2]);
		}
	}
	fclose(fp);
}

int copyCloud(Mat & cloudMat,pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloudPCL, Mat& img)
{
	Mat nidaye;
	nidaye.create(480, 640, CV_8UC3);

	if (cloudMat.depth() == CV_32FC1 || cloudMat.depth() == CV_64FC1)
	{
		pcl::PointXYZRGB point;
		//pCloudPCL->reserve(cloudMat.rows* cloudMat.cols*4*8);
		for (int i = 0; i < cloudMat.rows; i++)
		{
			float *ptrSource = cloudMat.ptr<float>(i);
			//uchar *color = cloudMat.ptr<uchar>(i);
			//uchar * nidayeC = nidaye.ptr<uchar>(i);

			for (int j = 0; j < cloudMat.cols; j++)
			{
				if (ptrSource[3 * j] < 1500 && ptrSource[3 * j] > -1500 
					&& ptrSource[3 * j + 1] < 1500 && ptrSource[3 * j+1] > -1500
					&&1< ptrSource[3 * j + 2] && ptrSource[3 * j + 2] <= 8000.0)			
				{
					point.x = -ptrSource[3 * j]/1000;
					point.y= ptrSource[3 * j + 1]/1000;
					point.z = ptrSource[3 * j + 2]/1000;
					
		/*			point.b = color[j * 3];
					point.g= color[j * 3+1];
					//point.r = color[j * 3+2];*/
					point.b = (uint8_t)img.data[(i*cloudMat.cols + j) * 3];
					point.g = (uint8_t)img.data[(i*cloudMat.cols + j) * 3 + 1];
					point.r = (uint8_t)img.data[(i*cloudMat.cols + j) * 3 + 2];

					nidaye.data[(i*cloudMat.cols + j) * 3] = img.data[(i*cloudMat.cols + j) * 3];
					nidaye.data[(i*cloudMat.cols + j) * 3 + 1] = img.data[(i*cloudMat.cols + j) * 3 + 1];
					nidaye.data[(i*cloudMat.cols + j) * 3 + 2] = img.data[(i*cloudMat.cols + j) * 3 + 2];
				/*	point.b = 255;
					point.g = 0;
					point.r =0;*/
					pCloudPCL->points.push_back(point);
				}
				else
				{
					nidaye.data[(i*cloudMat.cols + j) * 3] = 0;
					nidaye.data[(i*cloudMat.cols + j) * 3 + 1] = 0;
					nidaye.data[(i*cloudMat.cols + j) * 3 + 2] = 0;
				}
			}
		}
		cout << cloudMat.rows << "            " << img.rows << endl;
		cout << cloudMat.cols << "            " << img.cols << endl;
		pCloudPCL->width=pCloudPCL->size();
		pCloudPCL->height = 1;
	}
	else
		printf("The input Cloud in Mat format is something wrong!/n");
	printf("we gererated one pointCloud with  %d points!/n", pCloudPCL->size());
	imwrite("ndaye.jpg",nidaye);
	return 1;
}

int vilizeCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloudPCL)
{
	pcl::visualization::PCLVisualizer::Ptr viewerl(new pcl::visualization::PCLVisualizer);
	viewerl->setBackgroundColor(0, 0, 0);
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(pCloudPCL);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pCloudPCL);
	viewerl->addPointCloud<pcl::PointXYZRGB>(pCloudPCL, rgb,"sample cloud");
	viewerl->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
//	viewerl->addCoordinateSystem(50.0);
	viewerl->initCameraParameters();


		viewerl->spinOnce(100);
		//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		return 1;
}

int  cloudSave(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloudPCL,int &count)
{
	char path[100];
	count++;
	sprintf(path, "data/pcd_color%d.pcd",count);
	pcl::io::savePCDFileASCII(path, *pCloudPCL);
	std::cout << "Saved " << pCloudPCL->points.size() << " xyz points to :" << path << "." << std::endl;
	return 1;
}

int readYML2MAT(const char *nameYML, Mat & nameMAT)
{
	FileStorage fs(nameYML, CV_STORAGE_READ);
	if (!fs.isOpened())
	{
		printf("Failed to open file %s\n", nameYML);
		getchar();
		return -1;
	}

	fs["ans"] >> nameMAT;
}

int main(int argc, char** argv)
{

	const char* img2_filename = "data/data1/left12.jpg";
    const char* img1_filename = "data/data1/right12.jpg";
	//const char* img1_filename = "test_calib/Left0005.bmp";
	//const char* img2_filename = "test_calib/Right0005.bmp";


	const char* intrinsic_filename = "test_calib13/intrinsics1.yml";
	const char* extrinsic_filename = "test_calib13/extrinsics1.yml";
	const char* point_cloud_filename = "test_recons/BM4.txt";

	/*const char * dis_pic = "test_12/Result_Venus_SMP.png";*/
	const char * dis_pic = "test_calib/disp0005.bmp";
	enum { STEREO_BM = 0, STEREO_SGBM = 1, STEREO_HH = 2, STEREO_VAR = 3,STEREO_GC=4 };
	int alg = 1;
	int SADWindowSize = 25, numberOfDisparities = 80;

	bool no_display = false;
	float scale = 1.0f;
	int saveCloudCount = 0;
	bool saveCloud = true;

	//left01.jpg right01.jpg --algorithm = sgbm --max - disparity = 32 --blocksize = 3 --scale = 0.5 - o dis.jpg - i test_calib / intrinsics.yml - e test_calib / extrinsics.yml - p test_recons / 123.yml
	//相机参数和是否重建标志位
	bool intrinsic =true;
	bool recons = true;

	Mat xyz;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloudPCL(new pcl::PointCloud<pcl::PointXYZRGB>);

	StereoBM bm;
	//StereoBM bm1(StereoBM::BASIC_PRESET, 80, 31);
	StereoSGBM sgbm;
	StereoVar var; 

	int color_mode = alg == STEREO_BM ? 0 : -1;
	Mat img1 = imread(img1_filename, color_mode);
	Mat img2 = imread(img2_filename, color_mode);
	Mat dispic = imread(dis_pic,0);
	Mat disp2;
   dispic.convertTo(disp2,CV_16SC1);
   dispic = disp2;
	bool  disparity = true;

	if (scale != 1.f)                                                        //放缩图相
	{
		Mat temp1, temp2;
		int method = scale < 1 ? INTER_AREA : INTER_CUBIC;
		resize(img1, temp1, Size(), scale, scale, method);
		img1 = temp1;
		resize(img2, temp2, Size(), scale, scale, method);
		img2 = temp2;
	}

	Size img_size = img1.size();

	Rect roi1, roi2;
	Mat Q;

	if (intrinsic)                                                             //读入内参矩阵文件
	{
		// reading intrinsic parameters
		FileStorage fs(intrinsic_filename, CV_STORAGE_READ);
		if (!fs.isOpened())
		{
			printf("Failed to open file %s\n", intrinsic_filename);
			return -1;
		}

		Mat M1, D1, M2, D2;                                        //双相机内参
		fs["M1"] >> M1;
		fs["D1"] >> D1;
		fs["M2"] >> M2;
		fs["D2"] >> D2;

		M1 *= scale;
		M2 *= scale;

		fs.open(extrinsic_filename, CV_STORAGE_READ);
		if (!fs.isOpened())
		{
			printf("Failed to open file %s\n", extrinsic_filename);
			return -1;
		}

		Mat R, R11, T, R1, P1, R2, P2;                                                              //双相机外参
		fs["R"] >> R;
		fs["T"] >> T;

		if (R.rows == 1)
		{
			Rodrigues(R,R11);
			R = R11;
		}

	////stereoRectify(M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2);        //矫正相机使其图像平面平行
	stereoRectify(M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, 0, -1, img_size, &roi1, &roi2);
		Mat map11, map12, map21, map22;
		initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);                                     //初始化两个相机矫正模型
		initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

		Mat img1r, img2r,temp1,temp2;
		remap(img1, img1r, map11, map12, INTER_LINEAR);                                                                               // 矫正两幅图像使其对极线平行
		remap(img2, img2r, map21, map22, INTER_LINEAR);

		//cout << "roi1:x=" << roi1.x << " ,width =" << roi1.width << " ,y = " << roi1.y << ", heigh =" << roi1.height << endl;
		//cout << "roi2:x=" << roi2.x << " ,width =" << roi2.width << ", y =" << roi2.y << ", heigh =" << roi2.height << endl;
		//int a, b, c, d;
		//a = max(roi1.x,roi2.x);
		//b = max(roi1.y,roi2.y);
		//c = min(roi1.x + roi1.width, roi2.x + roi2.width);
		//d = min(roi1.y+roi1.height,roi2.y+roi2.height);
		//temp1 = img1r.rowRange(b, d);
		//img1 = temp1.colRange(a, c);
		//temp2 = img2r.rowRange(b,d);
		//img2 = temp2.colRange(a, c);
		img1 = img1r;
		img2 = img2r;
	}

	numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width / 8) + 15) & -16;

	bm.state->preFilterCap = 25;
	bm.state->preFilterSize = 11;
	//bm.state->SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 9;
	bm.state->SADWindowSize =23;
	bm.state->minDisparity = 0;
	bm.state->numberOfDisparities =96;
	//bm.state->textureThreshold = 10;
	bm.state->textureThreshold = 50;
	bm.state->uniquenessRatio = 1;
	bm.state->speckleWindowSize = 100;
	bm.state->speckleRange = 20;
	bm.state->disp12MaxDiff = 1;


	sgbm.preFilterCap = 15;
	sgbm.SADWindowSize = 3;
	int cn = img1.channels();

	sgbm.P1 = 8 * cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
	sgbm.P2 = 32 * cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
	sgbm.minDisparity = 0;
	sgbm.numberOfDisparities = 80;
	sgbm.uniquenessRatio = 5;
	sgbm.speckleWindowSize = 100;
	sgbm.speckleRange = 32;
	sgbm.disp12MaxDiff = 1;
	sgbm.fullDP = alg == STEREO_HH;

	var.levels = 3;                                 // ignored with USE_AUTO_PARAMS
	//var.pyrScale = 0.5;                             // ignored with USE_AUTO_PARAMS
	var.pyrScale = 1.5;
	//var.nIt = 25;
	var.nIt = 25;
	//var.minDisp = -numberOfDisparities;
	var.minDisp = -64;
	//var.minDisp = -32;
	var.maxDisp = 64;
	//var.poly_n = 3;
	var.poly_n = 7;
	//var.poly_sigma = 0.0;
	var.poly_sigma = 1.5;
	//var.fi = 15.0f;
	var.fi = 15.0f;
	//var.lambda = 0.03f;
	var.lambda = 0.13f;
	//var.penalization = var.PENALIZATION_TICHONOV;   // ignored with USE_AUTO_PARAMS
	var.cycle = var.CYCLE_O;                        // ignored with USE_AUTO_PARAMS
	var.penalization = var.PENALIZATION_PERONA_MALIK;
	//var.flags = var.USE_SMART_ID | var.USE_INITIAL_DISPARITY | var.USE_MEDIAN_FILTERING;
	var.flags = var.USE_SMART_ID | var.USE_AUTO_PARAMS | var.USE_INITIAL_DISPARITY | var.USE_MEDIAN_FILTERING | var.USE_EQUALIZE_HIST;
	Mat disp, disp8;
	//Mat img1p, img2p, dispp;
	//copyMakeBorder(img1, img1p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);
	//copyMakeBorder(img2, img2p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);

	int64 t = getTickCount();
	if (alg == STEREO_BM)
	{
		bm(img1, img2, disp);
	   //	disp = disp.colRange(m_nMaxDisp, img1b.cols);
	}
	else if (alg == STEREO_VAR) 
	{
		var(img1, img2, disp);
	}
	else if (alg == STEREO_SGBM || alg == STEREO_HH)
		sgbm(img1, img2, disp);
	else if (alg==STEREO_GC)
	{
		//IplImage  cv_left_rectified(img1);
		//IplImage  cv_right_rectified(img2);
		IplImage  cv_left_rectified(img1);
		IplImage  cv_right_rectified(img2);

		//Mat disp, disp8;
		printf("输入图像的分辨率位x=%d，y=%d。\n", cv_left_rectified.width, cv_right_rectified.height);

		CvSize size = cvGetSize(&cv_left_rectified);
		//the disparity map is an array h*w, with 16bit signed elements.
		CvMat* disparity_left = cvCreateMat(size.height, size.width, CV_16S);
		CvMat* disparity_right = cvCreateMat(size.height, size.width, CV_16S);

		CvStereoGCState* state = cvCreateStereoGCState(16, 2);

		cvFindStereoCorrespondenceGC(&cv_left_rectified,
			&cv_right_rectified,
			disparity_left,
			disparity_right,
			state,
			0);
		cvReleaseStereoGCState(&state);

		CvMat* disparity_left_visual = cvCreateMat(size.height, size.width, CV_8U);
		cvConvertScale(disparity_left, disparity_left_visual, -16);
		disp = Mat(disparity_left_visual);
		printf("输入图像的分辨率位x=%d，y=%d。\n", disp.rows, disp.cols);

	}
	t = getTickCount() - t;
	printf("Time elapsed: %fms\n", t * 1000 / getTickFrequency());

	//disp = dispp.colRange(numberOfDisparities, img1p.cols);
	if (alg != STEREO_VAR)
		disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities*16.));
	else
		disp.convertTo(disp8, CV_8U);
	if (!no_display)
	{
		namedWindow("left", 1);
		imshow("left", img1);
		namedWindow("right", 1);
		imshow("right", img2);
		namedWindow("disparity", 1);
		imshow("disparity", disp8);
		//printf("press any key to continue...");
		//fflush(stdout);
		//waitKey();
		//printf("\n");
	}

	//printf("视差图的名称=%s\n", disparity_filename);                                      //视差图名字检查
	printf("视差图分辨率为disp8.cols=%d,disp8.rows=%d.", disp8.cols, disp8.rows);  

	char disparity_filename[100] = "test_mapping/" ;

	if (disparity)
	{
		char a1[100];
		switch (alg)
		{
		case STEREO_BM:
			//sprintf(a1, "BM_max_dis=%d blocksize=%d picname=%st.jpg", numberOfDisparities, SADWindowSize, imgfilename);
			sprintf(a1, "BM_max_dis=%d_blocksize=%d.jpg", numberOfDisparities, SADWindowSize);//, numberOfDisparities, SADWindowSize, imgfilename);
			strcat(disparity_filename,a1);
			break;
		case STEREO_VAR:
			sprintf(a1, "VAR_max_dis=%d_blocksize=%djpg", numberOfDisparities, SADWindowSize);
			strcat(disparity_filename, a1);
			break;
		case STEREO_SGBM:
			sprintf(a1, "SGBM_max_dis_%d_blocksize=%d.jpg", numberOfDisparities, SADWindowSize);
			strcat(disparity_filename, a1);
			break;
		case STEREO_GC:
			sprintf(a1, "GC_max_dis_%d_blocksize=%d.jpg", numberOfDisparities, SADWindowSize);
			strcat(disparity_filename, a1);
			break;
		default:
			sprintf(a1, "HH_max_dis=%d_blocksize=%d.jpg", numberOfDisparities, SADWindowSize);
			strcat(disparity_filename, a1);
			break;
		}
		printf("视差图的名称=%s\n", disparity_filename);
		//vector<int> compression_params;
		//compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
		//compression_params.push_back(98);
		//waitKey();
		cv::imwrite(disparity_filename, disp8);
	}

	Mat nidaye;
	namedWindow("nidaye");

	//imwrite(a1, disp8);
	readYML2MAT("test13/3.yml",nidaye);
	//flip(nidaye,nidaye,-1);

	imshow("nidaye", nidaye);
	if (recons)
	{
		printf("storing the point cloud...\n");
		fflush(stdout);
	
		reprojectImageTo3D(nidaye, xyz, Q, true);
		printf("输出图像的分辨率：x=%d，y=%d,type=%d.\n", img1.rows, img1.cols, img1.type());
		printf("点云矩阵维数位：x=%d，y=%d,type=%d\n", xyz.rows, xyz.cols, xyz.type());
		//gl(argc, argv);// xyz, img1);
		
		//gl(argc, argv, xyz, img1, xyz.cols, xyz.rows);
		
		//saveXYZ(point_cloud_filename, xyz,	img1);
		printf("point cloud has been storyed!");
		printf("\n");
	}

	copyCloud( xyz, pCloudPCL,img1);
	if (saveCloud)
		cloudSave(pCloudPCL,saveCloudCount);
	vilizeCloud( pCloudPCL);

	while (waitKey(20) != 'q');
	return 0;
}
