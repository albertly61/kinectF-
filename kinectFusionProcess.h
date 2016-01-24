#pragma once

#include "stdafx.h"

#include <NuiApi.h>
#include <NuiKinectFusionApi.h>

#include "Timer.h"

#include <opencv2/opencv.hpp>

#include <iostream>

#include "PCLViewAndProcess.h"
//#include "image.h"
#include "rectifyVSpcl.h"

using namespace std;
using namespace cv;

/// <summary>
/// Performs all Kinect Fusion processing for the KinectFusionExplorer.
/// All data capture and processing is done on a worker thread.
/// </summary>
class KinectFusionProcessor
{
	static const int            cResetOnTimeStampSkippedMillisecondsGPU = 2000;
	static const int            cResetOnTimeStampSkippedMillisecondsCPU = 6000;
	static const int            cResetOnNumberOfLostFrames = 100;
	static const int            cTimeDisplayInterval = 4;
	static const int            cRenderIntervalMilliseconds = 100; // Render every 100ms
	static const int            cMinTimestampDifferenceForFrameReSync = 17; // The minimum timestamp difference between depth and color (in ms) at which they are considered un-synchronized. 

	static enum kinds{
		noon, color, colorS, depth, depthS, colorVideo, depthVideo, colorAnddepth, colorSanddepthS,
		fusionReset, meshSave, pclBasicSave, pclColorSave, pclNormSave, pclBasicWithNormSave, 
		pclColorWithNormSave
	};	
	static enum state{ busying = 20, free };
		
	int flagChose = 0;                    //彩色还是黑白色
	bool flagSavingCVideo = false;
	bool flagSavingDVideo = false;
	 int flagWitch = noon;            // 哪项操作
	 int flagState = free;            //是否有空

	 int colorpiccount = 0, colormpegcount = 0, depthpiccount = 0, depthmpegcount = 0;
	
public:
	/// <summary>
	/// Constructor
	/// </summary>
	KinectFusionProcessor();

	/// <summary>
	/// Destructor
	/// </summary>
	~KinectFusionProcessor();

	/// <summary>
	/// Reset the reconstruction camera pose and clear the volume on the next frame.
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT                     ResetReconstruction();


	/// <summary>
	/// Is reconstruction volume initialized and running.
	/// </summary>
	bool                        IsVolumeInitialized();

	/// <summary>
	/// Is the camera pose finder initialized and running.
	/// </summary>
	bool                        IsCameraPoseFinderAvailable();

	//void run();
	/// <summary>
	/// Main processing function
	/// </summary>
	DWORD                       MainLoop();

	ViewerAndProcesser   pclProcess;

private:

	HWND                        m_hWnd;

	INuiSensor*                 m_pNuiSensor;

	HANDLE                      m_pDepthStreamHandle;
	HANDLE                      m_hNextDepthFrameEvent;

	HANDLE                      m_pColorStreamHandle;
	HANDLE                      m_hNextColorFrameEvent;

	LONGLONG                    m_cLastDepthFrameTimeStamp;
	LONGLONG                    m_cLastColorFrameTimeStamp;

	/// <summary>
	/// Shuts down the sensor.
	/// </summary>
	void                        ShutdownSensor();

	/// <summary>
	/// Create the first connected Kinect found.
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT                     CreateFirstConnected();

	/// <summary>
	/// Initialize Kinect Fusion volume and images for processing
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT                     InitializeKinectFusion();

	/// <summary>
	/// Create a Kinect Fusion image frame of the specified type.
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT                     CreateFrame(
		NUI_FUSION_IMAGE_TYPE frameType,
		unsigned int imageWidth,
		unsigned int imageHeight,
		NUI_FUSION_IMAGE_FRAME** ppImageFrame);

	/// <summary>
	/// Release and re-create a Kinect Fusion Reconstruction Volume.
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT                     RecreateVolume();

	/// <summary>
	/// Copy the extended depth data out of a Kinect image frame.
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT                     CopyExtendedDepth(NUI_IMAGE_FRAME &imageFrame);

	/// <summary>
	/// Copy the color data out of a Kinect image frame
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT                     CopyColor(NUI_IMAGE_FRAME &imageFrame,bool getColorFrame);

	/// <summary>
	/// Get the next frames from Kinect, re-synchronizing depth with color if required.
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT                     GetKinectFrames(bool &integrateColor);

	bool m_saveCloud;
	bool m_gotKinectRGB;
	/// <summary>
	/// Adjust color to the same space as depth
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT                     MapColorToDepth();

	/// <summary>
	/// Handle new depth data and perform Kinect Fusion Processing.
	/// </summary>
	void                        ProcessDepth();

	/// <summary>
	/// Perform camera tracking using AlignDepthFloatToReconstruction
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT                     TrackCameraAlignDepthFloatToReconstruction(Matrix4 &calculatedCameraPose, FLOAT &alignmentEnergy);

	/// <summary>
	/// Perform camera tracking using AlignPointClouds
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT                     TrackCameraAlignPointClouds(Matrix4 &calculatedCameraPose, FLOAT &alignmentEnergy);

	/// <summary>
	/// Perform camera pose finding when tracking is lost using AlignPointClouds.
	/// This is typically more successful than FindCameraPoseAlignDepthFloatToReconstruction.
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT                     FindCameraPoseAlignPointClouds();

	/// <summary>
	/// Perform camera pose finding when tracking is lost using AlignDepthFloatToReconstruction.
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT                     FindCameraPoseAlignDepthFloatToReconstruction();

	/// <summary>
	/// Calculate the residual alignment energy following AlignPointClouds.
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code</returns>
	//HRESULT                     CalculateAlignmentResidualEnergy(float &alignmentEnergy);

	void decide();

	void save();
	/// <summary>
	/// Performs raycasting for given pose and sets the tracking reference frame.
	/// </summary>
	/// <param name="worldToCamera">The reference camera pose.</param>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT                     SetReferenceFrame(const Matrix4 &worldToCamera);

	/// <summary>
	/// Set the tracking variables to tracking failed.
	/// </summary>
	void                        SetTrackingFailed();

	/// <summary>
	/// Set the tracking variables to tracking success.
	/// </summary>
	void                        SetTrackingSucceeded();

	/// <summary>
	/// Reset the tracking following a reset or re-create of the volume.
	/// </summary>
	void                        ResetTracking();

	/// <summary>
	/// Process the color image for the camera pose finder.
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT                     ProcessColorForCameraPoseFinder(bool &resampled);

	/// <summary>
	/// Update data for camera pose finding and store key frame poses.
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT                     UpdateCameraPoseFinder();

	//	int Color2Mat(NUI_IMAGE_FRAME &imageFrame));
	/// <summary>
	/// Reset the reconstruction camera pose and clear the volume.
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT                     InternalResetReconstruction();

	int                           writeCameraPose();

	bool                        m_bKinectFusionInitialized;
	bool                        m_bResetReconstruction;

	bool                        m_bIntegrationResumed;

	/// <summary>
	/// The Kinect Fusion Volume.
	/// </summary>
	INuiFusionColorReconstruction* m_pVolume;

	/// <summary>
	// The default Kinect Fusion World to Volume Transform.
	/// </summary>
	Matrix4                     m_defaultWorldToVolumeTransform;
	Matrix4                     m_worldToCameraTransform;
	/// <summary>
	/// Frames from the depth input.
	/// </summary>
	NUI_DEPTH_IMAGE_PIXEL*      m_pDepthImagePixelBuffer;
	int                         m_cPixelBufferLength;

	/// <summary>
	/// Frames generated from the depth input for AlignPointClouds
	/// </summary>
	NUI_FUSION_IMAGE_FRAME*     m_pDownsampledDepthFloatImage;
	NUI_FUSION_IMAGE_FRAME*     m_pDownsampledSmoothDepthFloatImage;
	NUI_FUSION_IMAGE_FRAME*     m_pDownsampledDepthPointCloud;

	/// <summary>
	/// For mapping color to depth
	/// </summary>
	NUI_FUSION_IMAGE_FRAME*     m_pColorImage;
	NUI_FUSION_IMAGE_FRAME*     m_pResampledColorImageDepthAligned;
	int                         m_cColorCoordinateBufferLength;
	NUI_COLOR_IMAGE_POINT*      m_pColorCoordinates;
	INuiCoordinateMapper*       m_pMapper;

	/// <summary>
	/// Frames generated from ray-casting the Reconstruction Volume.
	/// </summary>
	Matrix4                     m_worldToBGRTransform;
	NUI_FUSION_IMAGE_FRAME*     m_pRaycastPointCloud;
	NUI_FUSION_IMAGE_FRAME*     m_pRaycastDepthFloatImage;
	NUI_FUSION_IMAGE_FRAME*     m_pDownsampledRaycastPointCloud;

	/// <summary>
	/// Images for display.
	/// </summary>
	NUI_FUSION_IMAGE_FRAME*     m_pDepthFloatImage;
	NUI_FUSION_IMAGE_FRAME*     m_pShadedSurface;
	NUI_FUSION_IMAGE_FRAME*     m_pShadedSurfaceNormals;
	NUI_FUSION_IMAGE_FRAME*     m_pCapturedSurfaceColor;
	NUI_FUSION_IMAGE_FRAME*     m_pFloatDeltaFromReference;
	NUI_FUSION_IMAGE_FRAME*     m_pShadedDeltaFromReference;
	NUI_FUSION_IMAGE_FRAME*     m_pDownsampledShadedDeltaFromReference;

	/// <summary>
	/// Camera Tracking parameters.
	/// </summary>
	int                         m_cLostFrameCounter;
	bool                        m_bTrackingFailed;

	/// <summary>
	/// Camera Pose Finder.
	/// Note color will be re-sampled to the depth size if depth and color capture resolutions differ.
	/// </summary>
	INuiFusionCameraPoseFinder* m_pCameraPoseFinder;
	NUI_FUSION_IMAGE_FRAME*     m_pResampledColorImage;
	NUI_FUSION_IMAGE_FRAME*     m_pDepthPointCloud;
	NUI_FUSION_IMAGE_FRAME*     m_pSmoothDepthFloatImage;
	unsigned                    m_cSuccessfulFrameCounter;
	bool                        m_bTrackingHasFailedPreviously;
	bool                        m_bCalculateDeltaFrame;
	bool     m_bGetColorFrame;  //是是否获取Kinect彩色图像，仅用于保存
	/// <summary>
	/// Frame counter and timer.
	/// </summary>
	int                         m_cFrameCounter;
	Timing::Timer               m_timer;
	double                      m_fFrameCounterStartTime;
	double                      m_fMostRecentRaycastTime;

	int Display();


	/*IplImage* pFrame = NULL;*/
	int stereoCount = 0;
	/*CvCapture* pCapture = NULL;*/
	//VideoCapture pCapture;
	/*void cutAndSaveImg(Mat& src_img, char* path1, char* path2);*/
	//opencv Mat
	//Mat stereoImg;
	Mat cloudShade;
	//Mat depthData;
	Mat m_imgDepth;
	Mat m_imgColor;
	Mat cloudRGB;
	Mat kinectRGB;
	Mat dert;
	//Mat stereoView;

	//param
	bool                        m_bAutoFindCameraPoseWhenLost;
	NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE m_processorType;
	int                         m_deviceIndex; 
	unsigned int m_deviceMemory;
	NUI_FUSION_RECONSTRUCTION_PARAMETERS m_reconstructionParams;
	bool m_bTranslateResetPoseByMinDepthThreshold;
	bool m_bMirrorDepthFrame;
	bool  m_bCaptureColor;
	float                       m_fMinDepthThreshold;
	float                       m_fMaxDepthThreshold;
	bool           m_bAutoResetReconstructionOnTimeout;
	char     m_cSmoothingKernelWidth;
	float m_fSmoothingDistanceThreshold;
	float m_fMaxTranslationDelta;              // 0.15 - 0.3m per frame typical
	float m_fMaxRotationDelta;
	float	m_fMaxAlignToReconstructionEnergyForSuccess;
	char m_cColorIntegrationInterval;
	int m_cDeltaFromReferenceFrameCalculationInterval;
	bool                        m_bAutoResetReconstructionWhenLost;
	int m_cMinSuccessfulTrackingFramesForCameraPoseFinderAfterFailure;
	bool m_bPauseIntegration;
	unsigned short              m_cMaxIntegrationWeight;
	bool m_bDisplaySurfaceNormals ;
	int m_cMinSuccessfulTrackingFramesForCameraPoseFinder;
	int m_cCameraPoseFinderProcessFrameCalculationInterval;
	float                       m_fCameraPoseFinderDistanceThresholdReject;
	float m_fFramesPerSecond;
	float m_fMaxAlignPointCloudsEnergyForSuccess;
	unsigned int                m_cMaxCameraPoseFinderPoseTests;
	float m_fMinAlignPointCloudsEnergyForSuccess;
	float                       m_fMinAlignToReconstructionEnergyForSuccess;
	float                       m_fCameraPoseFinderDistanceThresholdAccept;

	int  gengeratePointsB(NUI_FUSION_IMAGE_FRAME*     m_pPointCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyzcnormals);
	int  gengeratePointsNB(NUI_FUSION_IMAGE_FRAME*     m_pPointCloud, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_xyzcnormals);
	int gengeratePointsNC(NUI_FUSION_IMAGE_FRAME*     m_pPointCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzcnormals);
	bool  VolumeChanged(const NUI_FUSION_RECONSTRUCTION_PARAMETERS& params);
	//gai
	HRESULT m_hrRecreateVolume;
	bool m_bNearMode;
	bool  m_bRecreateVolume;

	//vidoerWirter
	VideoWriter m_writeColor;
	VideoWriter m_writeDepth;
	//other
	bool m_savePCLNBC;
	bool m_savePCLNB;
	//cameraPose
	BOOL addedPose = FALSE;

	//文件相关
	//ofstream ocout;
	int ofsCount;
	FileStorage fs;

	//stereo
	//stereo_camera stereo;
	Mat stereoPic;
	Mat left, right, dis;
	Mat frame;

	float* D1_data = (float*)malloc(640*480*sizeof(float));
	float* D2_data = (float*)malloc(640*480*sizeof(float));
	//bool stereoCloudGenerate;
	int	getFullPic(Mat & mat);
	int    prossStereo(Mat & mat1, Mat& mat2, Mat &dis, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloudPCL);
	VideoCapture cap;
	recvspcl recpcl;
};