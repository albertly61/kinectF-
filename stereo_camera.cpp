#include "stereo_camera.h"
#include "image.h"

stereo_camera::stereo_camera()
{
	cap.open(0);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
	if (!cap.isOpened()) // check if we succeeded
	{
		cout << "cant open the camera!" << endl;
		getchar();
		return ;
	}
	frame.create(480, 1280, CV_8UC3);
}

stereo_camera::~stereo_camera()
{
}

int stereo_camera::openCamera()
{
	cap.open(0);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
	if (!cap.isOpened()) // check if we succeeded
	{
		cout << "cant open the camera!" << endl;
		getchar();
		return -1;
	}
	return 0;
}

int stereo_camera::getFullPic(Mat & mat)
{
	cap >> mat;
	return 0;
}

int stereo_camera::getTwoPic(Mat & left,Mat& right)
{
	
	cap >> frame;
	flip(frame, frame, 1);
	left = frame.colRange(0, 640);
	right = frame.colRange(640, 1280);
	return 1;
}

int stereo_camera::prossNew(Mat & mat1, Mat& mat2, Mat &dis,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloudPCL)
{
	Mat tmp1, tmp2;
	tmp1.create(mat1.rows, mat1.cols, CV_8SC1);
	tmp2.create(mat1.rows, mat1.cols, CV_8SC1);

	cvtColor(mat1, tmp1, CV_BGR2GRAY);
	cvtColor(mat2, tmp2, CV_BGR2GRAY);
	/*mat2 = tmp2;
	mat1 = tmp1;*/
	image<uchar> *I1 = new image<uchar>(mat1.cols, mat1.rows);
	image<uchar> *I2 = new image<uchar>(mat2.cols, mat2.rows);
	loadMat(I1, tmp1, mat1.cols, mat1.rows);
	loadMat(I2, tmp2, mat2.cols, mat2.rows);

	cout << "width= " << I1->width() << ".  height= " << I2->height() << endl;
	if (I1->width() <= 0 ||
		I1->height() <= 0 ||
		I2->width() <= 0 ||
		I2->height() <= 0 ||
		I1->width() != I2->width() ||
		I1->height() != I2->height())
	{
		cout << "ERROR: Images must be of same size, but" << endl;
		cout << "       I1: " << I1->width() << " x " << I1->height() <<
			", I2: " << I2->width() << " x " << I2->height() << endl;
		delete I1;
		delete I2;
		return -1;
	}

	// get image width and height
	int32_t width = I1->width();
	int32_t height = I1->height();

	cout << "width= " << width << ".  height= " << height << endl;

	// allocate memory for disparity images
	const int32_t dims[3] = { width, height, width }; // bytes per line = width
	float* D1_data = (float*)malloc(width*height*sizeof(float));
	float* D2_data = (float*)malloc(width*height*sizeof(float));

	// process
	Elas::parameters param;
	param.postprocess_only_left = false;
	Elas elas(param);
	elas.process(I1->data, I2->data, D1_data, D2_data, dims);

	// find maximum disparity for scaling output disparity images to [0..255]
	float disp_max = 0;
	for (int32_t i = 0; i<width*height; i++)
	{
		if (D1_data[i]>disp_max)
			disp_max = D1_data[i];
	}

	// copy float to uchar
	image<uchar> *D1 = new image<uchar>(width, height);
	dis.create(height, width, CV_8UC1);
	for (int32_t i = 0; i<width*height; i++)
	{
		dis.data[i] = (uint8_t)max(255.0*D1_data[i] / disp_max, 0.0);
	}

	recpcl.project3D(dis);
	recpcl.colorCloud(mat1);
	pCloudPCL = recpcl.pCloudPCL;

	// free memory
	delete I1;
	delete I2;
	delete D1;
	free(D1_data);
	free(D2_data);
	return 0;
}
