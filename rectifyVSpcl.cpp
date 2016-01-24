#include "rectifyVSpcl.h"


static void saveXYZ(const char* filename, const Mat& mat, const Mat& img)
{
	const double max_z = 1.0e3;
	FILE* fp = fopen(filename, "wt");
	for (int y = 0; y < mat.rows; y++)
	{
		for (int x = 0; x < mat.cols; x++)
		{
			Vec3f point = mat.at<Vec3f>(y, x);
			const uchar* color = img.ptr<uchar>(y, x);
			if (fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
			fprintf(fp, "%f %f %f %c %c %c\n", point[0], point[1], point[2], color[0], color[1], color[2]);
		}
	}
	fclose(fp);
}

recvspcl::recvspcl() :
cloudCount(0),
cloudSave(false),
pCloudPCL(new pcl::PointCloud<pcl::PointXYZRGB>)
//viewerl(new pcl::visualization::PCLVisualizer)
{
	char name1[100] = "args/intrinsics1.yml";
	char name2[100] = "args/extrinsics1.yml";

	readarg(name1,name2);
}

recvspcl::~recvspcl()
{}

//给点云着色
int recvspcl::colorCloud(Mat& img, pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudPCL)
{

	CloudPCL->clear();
	if (xyz.depth() == CV_32FC1 || xyz.depth() == CV_64FC1)
	{
		pcl::PointXYZRGB point;
		for (int i = 0; i < xyz.rows; i++)
		{
			float *ptrSource = xyz.ptr<float>(i);
		
			for (int j = 0; j < xyz.cols; j++)
			{
				if (ptrSource[3 * j] < 1500 && ptrSource[3 * j] > -1500
					&& ptrSource[3 * j + 1] < 1500 && ptrSource[3 * j + 1] > -1500
					&& 1< ptrSource[3 * j + 2] && ptrSource[3 * j + 2] <= 8000.0)
				{
					point.x = -ptrSource[3 * j] / 500;
					point.y = ptrSource[3 * j + 1] / 500;
					point.z = ptrSource[3 * j + 2] / 500;

					point.b = (uint8_t)img.data[(i*img.cols + j) * 3];
					point.g = (uint8_t)img.data[(i*img.cols + j) * 3 + 1];
					point.r = (uint8_t)img.data[(i*img.cols + j) * 3 + 2];

					CloudPCL->points.push_back(point);
				}
			}
		}
		cout << xyz.rows << "=  " << img.rows << endl;
		cout << xyz.cols << "        =    " << img.cols << endl;
		CloudPCL->width = pCloudPCL->size();
		CloudPCL->height = 1;
	}
	else
	printf("The input Cloud in Mat format is something wrong!/n");
	printf("we gererated one pointCloud with  %d points!/n", pCloudPCL->size());
	return 1;
}

int recvspcl::vilizeCloud()
{
	viewerl->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pCloudPCL);

	viewerl->removeAllPointClouds();
	viewerl->addPointCloud<pcl::PointXYZRGB>(pCloudPCL, rgb, "sample cloud");
	viewerl->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewerl->initCameraParameters();

	viewerl->spinOnce(100);
	return 1;
}

int recvspcl::saveCloud()
{
	char path[100];
	cloudCount++;
	sprintf(path, "data/pcd_color%d.pcd", cloudCount);
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

int recvspcl::readarg(const char* id1, const char* id2)
{
	FileStorage fs(id1, CV_STORAGE_READ);
	if (!fs.isOpened())
	{
		printf("Failed to open file %s\n", id1);
		getchar();
		return -1;
	}

	fs["M1"] >> M1;
	fs["D1"] >> D1;
	fs["M2"] >> M2;
	fs["D2"] >> D2;

	fs.open(id2, CV_STORAGE_READ);
	if (!fs.isOpened())
	{
		printf("Failed to open file %s\n", id2);
		getchar();
		return -1;
	}

                              
	fs["R"] >> R;
	fs["T"] >> T;

	return 0;
}

int recvspcl::rectify(Mat &img1,Mat& img2)
{
	if (R.rows == 1)
	{
		Rodrigues(R, R11);
		R = R11;
	}
	Size img_size = img1.size();

	stereoRectify(M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, 0, -1, img_size, &roi1, &roi2);
	Mat map11, map12, map21, map22;
	initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);                                     //初始化两个相机矫正模型
	initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

	Mat img1r, img2r, temp1, temp2;
	remap(img1, img1r, map11, map12, INTER_LINEAR);                                                                               // 矫正两幅图像使其对极线平行
	remap(img2, img2r, map21, map22, INTER_LINEAR);

	img1 = img1r;
	img2 = img2r;
	return 0;
}

int recvspcl::project3D(Mat& dis)
{
	reprojectImageTo3D(dis, xyz, Q, true);
	return 0;
}
