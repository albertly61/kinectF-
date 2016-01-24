
#include <iostream>
#include "elas.h"
#include "image.h"
#include <opencv2/opencv.hpp>
#include "rectifyVSpcl.h"
#include <time.h>
 
//#include <algorithm>
using namespace std;
using namespace cv;
double max(double a1, double a2)
{
	return a1 > a2 ? a1 : a2;
}

void process(const char* file_1, const char* file_2, cv::Mat& mat1, cv::Mat& mat2, cv::Mat& dis)
{

	cout << "Processing: " << file_1 << ", " << file_2 << endl;


	// load images
	//image<uchar> *I1, *I2;
	mat1 = imread(file_1,1);
	mat2 = imread(file_2, 1);
	cout << "width= " << mat1.cols << ".  height= " << mat2.rows << endl;
	Mat tmp1, tmp2;
	tmp1.create(mat1.rows, mat1.cols, CV_8SC1); 
	tmp2.create(mat1.rows, mat1.cols, CV_8SC1);
	//mat1.convertTo(tmp1, CV_RGB2GRAY);
	//mat2.convertTo(tmp2,CV_RGB2GRAY);
	cvtColor(mat1, tmp1, CV_BGR2GRAY);
	cvtColor(mat2, tmp2, CV_BGR2GRAY);
	mat2 = tmp2;
	mat1 = tmp1;
	//namedWindow("123");
	//imshow("123",tmp1);
	cout << "width= " << mat1.cols << ".  height= " << mat2.rows << endl;
	image<uchar> *I1 = new image<uchar>(mat1.cols, mat1.rows);
	image<uchar> *I2 = new image<uchar>(mat2.cols, mat2.rows);
	loadMat(I1,mat1,mat1.cols,mat1.rows);
	loadMat(I2,mat2, mat2.cols, mat2.rows);
	cout << "width= " << I1->width() << ".  height= " << I2->height() << endl;
	/*I1 = im1;
	I2 = im2;*/
	// check for correct size
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
		return;
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
		if (D2_data[i]>disp_max)
			disp_max = D2_data[i];
	}

	// copy float to uchar
	image<uchar> *D1 = new image<uchar>(width, height);
	image<uchar> *D2 = new image<uchar>(width, height);
	for (int32_t i = 0; i<width*height; i++)
	{
		D1->data[i] = (uint8_t)max(255.0*D1_data[i] / disp_max, 0.0);
		D2->data[i] = (uint8_t)max(255.0*D2_data[i] / disp_max, 0.0);
	}
	dis.create(height,width,CV_8UC1);
	for (int32_t i = 0; i < width*height; i++)
	{
		dis.data[i] = D1->data[i];
		if (i % 10000 == 0)
			cout << D1->data[i];
	}
	// save disparity images
	//char output_1[1024];
	//char output_2[1024];
	//strncpy(output_1, file_1, strlen(file_1) - 4);
	//strncpy(output_2, file_2, strlen(file_2) - 4);
	//output_1[strlen(file_1) - 4] = '\0';
	//output_2[strlen(file_2) - 4] = '\0';
	//strcat(output_1, "_disp.pgm");
	//strcat(output_2, "_disp.pgm");
	//savePGM(D1, output_1);
	//savePGM(D2, output_2);

	// free memory
	delete I1;
	delete I2;
	delete D1;
	delete D2;
	free(D1_data);
	free(D2_data);
}

int prossNew(Mat & mat1,Mat& mat2,Mat &dis)
{
	Mat tmp1, tmp2;
	tmp1.create(mat1.rows, mat1.cols, CV_8SC1);
	tmp2.create(mat1.rows, mat1.cols, CV_8SC1);
	//mat1.convertTo(tmp1, CV_RGB2GRAY);
	//mat2.convertTo(tmp2,CV_RGB2GRAY);
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

	// save disparity images


	// free memory
	delete I1;
	delete I2;
	delete D1;
	free(D1_data);
	free(D2_data);
	return 0;
}

int savePic(Mat& left,Mat& right, Mat& dis,int& count)
{
	count++;
	char path1[100],path2[100],path3[100];
	sprintf(path1,"data/left%d.jpg",count);
	sprintf(path2, "data/right%d.jpg", count);
	sprintf(path3, "data/dis%d.jpg", count);
	imwrite(path1,left);
	imwrite(path2,right);
	imwrite(path3,dis);
	return 0;
}

int main(int argc, char** argv)
{
	cv::Mat left, right,dis,frame;
	bool yet = false;
	int count = 0;
	VideoCapture cap(0); // open the default camera
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
	if (!cap.isOpened()) // check if we succeeded
	{
		cout << "cant open the camera!" << endl;
		getchar();
		return -1;
	}
	class recvspcl recpcl;
	char a;
	namedWindow("left",WINDOW_AUTOSIZE);
	namedWindow("right", WINDOW_AUTOSIZE);
	namedWindow("dis", WINDOW_AUTOSIZE);
	while(1)
	{
		a = waitKey(100);
		if (a == 'a')
		{
			unsigned long time1= clock();
			cap >> frame;
			flip(frame, frame, 1);
			left = frame.colRange(0, 640);
			right = frame.colRange(640, 1280);
			unsigned long time2= clock();

			recpcl.rectify(left,right);
			unsigned long time3= clock();

			prossNew(left,right,dis);
			unsigned long time4= clock();

			recpcl.project3D(dis);
			recpcl.colorCloud(left);
			recpcl.vilizeCloud();
			imshow("left",left);
			imshow("right", right);
			imshow("dis", dis);

			unsigned long time5= clock();
			cout << "Get the pic using time: " << time2 - time1 << " ms." << endl;
			cout << "Rectify pic using time: " << time3 - time2 << " ms." << endl;
			cout << "Get the dis using time: " << time4 - time3 << " ms." << endl;
			cout << "Show the pic using time: " << time5- time4 << " ms." << endl;
			yet = true;
			waitKey(2);
		}
		if (a == 'b')
		{
			if (yet == true)
			{
				recpcl.saveCloud();
				savePic(left,right,dis,count);
				cout << "We gonna save this pic and point cloud!" << count << " times." << endl;
			}
		}
		if (a == 'c')
			break;

	}

	return 0;
}
