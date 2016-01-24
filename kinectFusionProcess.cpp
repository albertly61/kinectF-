#pragma warning(push)
#pragma warning(disable:6255)
#pragma warning(disable:6263)
#pragma warning(disable:4995)
#include "ppl.h"
#pragma warning(pop)

#include "kinectFusionProcess.h"
#include "image.h"
#include "stereo_camera.h"

void SetIdentityMatrix(Matrix4 &mat)
{
	mat.M11 = 1; mat.M12 = 0; mat.M13 = 0; mat.M14 = 0;
	mat.M21 = 0; mat.M22 = 1; mat.M23 = 0; mat.M24 = 0;
	mat.M31 = 0; mat.M32 = 0; mat.M33 = 1; mat.M34 = 0;
	mat.M41 = 0; mat.M42 = 0; mat.M43 = 0; mat.M44 = 1;
}

static inline bool IsValidResampleFactor(unsigned int factor)
{
	return (1 == factor || 2 == factor || 4 == factor || 8 == factor || 16 == factor);
}

HRESULT DownsampleFrameNearestNeighbor(NUI_FUSION_IMAGE_FRAME *src, NUI_FUSION_IMAGE_FRAME *dest, unsigned int factor)
{
	HRESULT hr = S_OK;

	if (nullptr == src || nullptr == dest)
	{
		return E_INVALIDARG;
	}

	if (!(src->imageType == NUI_FUSION_IMAGE_TYPE_COLOR || src->imageType == NUI_FUSION_IMAGE_TYPE_FLOAT || src->imageType == NUI_FUSION_IMAGE_TYPE_POINT_CLOUD)
		|| src->imageType != dest->imageType)
	{
		return E_INVALIDARG;
	}

	if (!IsValidResampleFactor(factor))
	{
		return E_INVALIDARG;
	}

	INuiFrameTexture *srcFrameTexture = src->pFrameTexture;
	INuiFrameTexture *downsampledFloatFrameTexture = dest->pFrameTexture;

	unsigned int downsampledWidth = src->width / factor;
	unsigned int downsampleHeight = src->height / factor;

	if (1 == factor && srcFrameTexture->BufferLen() != downsampledFloatFrameTexture->BufferLen())
	{
		return E_INVALIDARG;
	}
	else if (dest->width != downsampledWidth || dest->height != downsampleHeight)
	{
		return E_INVALIDARG;
	}

	NUI_LOCKED_RECT srcLockedRect;

	// Lock the frame data so the Kinect knows not to modify it while we're reading it
	hr = srcFrameTexture->LockRect(0, &srcLockedRect, nullptr, 0);

	// Make sure we've received valid data
	if (FAILED(hr) || srcLockedRect.Pitch == 0)
	{
		return E_NOINTERFACE;
	}

	NUI_LOCKED_RECT downsampledLockedRect;

	// Lock the frame data so the Kinect knows not to modify it while we're reading it
	hr = downsampledFloatFrameTexture->LockRect(0, &downsampledLockedRect, nullptr, 0);

	// Make sure we've received valid data
	if (FAILED(hr) || downsampledLockedRect.Pitch == 0)
	{
		srcFrameTexture->UnlockRect(0);
		return E_NOINTERFACE;
	}

	float *srcValues = (float *)srcLockedRect.pBits;
	float *downsampledDestValues = (float *)downsampledLockedRect.pBits;

	const unsigned int srcImageWidth = src->width;

	if (1 == factor)
	{
		errno_t err = memcpy_s(downsampledDestValues, downsampledFloatFrameTexture->BufferLen(), srcValues, srcFrameTexture->BufferLen());
		if (0 != err)
		{
			hr = E_FAIL;
		}
	}
	else
	{
		// Adjust for point cloud image size (6 floats per pixel)
		unsigned int step = (src->imageType == NUI_FUSION_IMAGE_TYPE_POINT_CLOUD) ? 6 : 1;
		unsigned int factorStep = factor * step;

		Concurrency::parallel_for(0u, downsampleHeight, [=, &downsampledDestValues, &srcValues](unsigned int y)
		{
			unsigned int index = downsampledWidth * y * step;
			unsigned int srcIndex = srcImageWidth * y * factorStep;

			for (unsigned int x = 0; x<downsampledWidth; ++x, srcIndex += factorStep)
			{
				for (unsigned int s = 0, localSourceIndex = srcIndex; s<step; ++s, ++index, ++localSourceIndex)
				{
					downsampledDestValues[index] = srcValues[localSourceIndex];
				}
			}
		});
	}

	// We're done with the textures so unlock them
	srcFrameTexture->UnlockRect(0);
	downsampledFloatFrameTexture->UnlockRect(0);

	return hr;
}

/// <summary>
/// Up sample color or depth float (32bits/pixel) frame with nearest neighbor - replicates pixels
/// </summary>
/// <param name="src">The source color image.</param>
/// <param name="dest">The destination up-sampled color image.</param>
/// <param name="factor">The up sample factor (1=just copy, 2=x*2,y*2, 4=x*4,y*4).</param>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT UpsampleFrameNearestNeighbor(NUI_FUSION_IMAGE_FRAME *src, NUI_FUSION_IMAGE_FRAME *dest, unsigned int factor)
{
	HRESULT hr = S_OK;

	if (nullptr == src || nullptr == dest)
	{
		return E_INVALIDARG;
	}

	if (src->imageType != dest->imageType || !(src->imageType == NUI_FUSION_IMAGE_TYPE_COLOR || src->imageType == NUI_FUSION_IMAGE_TYPE_FLOAT))
	{
		return E_INVALIDARG;
	}

	if (!IsValidResampleFactor(factor))
	{
		return E_INVALIDARG;
	}

	INuiFrameTexture *srcFrameTexture = src->pFrameTexture;
	INuiFrameTexture *upsampledDestFrameTexture = dest->pFrameTexture;

	unsigned int upsampledWidth = src->width * factor;
	unsigned int upsampleHeight = src->height * factor;

	if (1 == factor && srcFrameTexture->BufferLen() != upsampledDestFrameTexture->BufferLen())
	{
		return E_INVALIDARG;
	}
	else if (dest->width != upsampledWidth || dest->height != upsampleHeight)
	{
		return E_INVALIDARG;
	}

	NUI_LOCKED_RECT srcLockedRect;

	// Lock the frame data so the Kinect knows not to modify it while we're reading it
	hr = srcFrameTexture->LockRect(0, &srcLockedRect, nullptr, 0);

	// Make sure we've received valid data
	if (FAILED(hr) || srcLockedRect.Pitch == 0)
	{
		return E_NOINTERFACE;
	}

	NUI_LOCKED_RECT upsampledLockedRect;

	// Lock the frame data so the Kinect knows not to modify it while we're reading it
	hr = upsampledDestFrameTexture->LockRect(0, &upsampledLockedRect, nullptr, 0);

	// Make sure we've received valid data
	if (FAILED(hr) || upsampledLockedRect.Pitch == 0)
	{
		srcFrameTexture->UnlockRect(0);
		return E_NOINTERFACE;
	}

	unsigned int *srcValues = (unsigned int *)srcLockedRect.pBits;
	unsigned int *upsampledDestValues = (unsigned int *)upsampledLockedRect.pBits;

	const unsigned int srcImageWidth = src->width;
	const unsigned int srcImageHeight = src->height;

	if (1 == factor)
	{
		errno_t err = memcpy_s(upsampledDestValues, upsampledDestFrameTexture->BufferLen(), srcValues, srcFrameTexture->BufferLen());
		if (0 != err)
		{
			hr = E_FAIL;
		}
	}
	else
	{
		unsigned int upsampleRowMultiplier = upsampledWidth * factor;

		// Note we run this only for the source image height pixels to sparsely fill the destination with rows
		Concurrency::parallel_for(0u, srcImageHeight, [=, &upsampledDestValues, &srcValues](unsigned int y)
		{
			unsigned int index = upsampleRowMultiplier * y;
			unsigned int srcIndex = srcImageWidth * y;

			// Fill row
			for (unsigned int x = 0; x<srcImageWidth; ++x, ++srcIndex)
			{
				unsigned int color = srcValues[srcIndex];

				// Replicate pixels horizontally
				for (unsigned int s = 0; s<factor; ++s, ++index)
				{
					upsampledDestValues[index] = color;
				}
			}
		});

		unsigned int rowByteSize = upsampledWidth * sizeof(unsigned int);

		// Duplicate the remaining rows with memcpy
		for (unsigned int y = 0; y<srcImageHeight; ++y)   // iterate only for the smaller number of rows
		{
			unsigned int srcRowIndex = upsampleRowMultiplier * y;

			// Duplicate lines
			for (unsigned int r = 1; r<factor; ++r)
			{
				unsigned int index = upsampledWidth * ((y*factor) + r);

				errno_t err = memcpy_s(&(upsampledDestValues[index]), rowByteSize, &(upsampledDestValues[srcRowIndex]), rowByteSize);
				if (0 != err)
				{
					hr = E_FAIL;
				}
			}
		}
	}

	// We're done with the textures so unlock them
	srcFrameTexture->UnlockRect(0);
	upsampledDestFrameTexture->UnlockRect(0);

	return hr;
}

/// <summary>
/// Extract 3x3 rotation matrix from the Matrix4 4x4 transformation:
/// Then convert to Euler angles.
/// </summary>
/// <param name="transform">The transform matrix.</param>
/// <param name="rotation">Array of 3 floating point values for euler angles.</param>
void ExtractRot2Euler(const Matrix4 &transform, _Out_cap_c_(3) float *rotation)
{
	float phi = atan2f(transform.M23, transform.M33);
	float theta = asinf(-transform.M13);
	float psi = atan2f(transform.M12, transform.M11);

	rotation[0] = phi;	// This is rotation about x,y,z, or pitch, yaw, roll respectively
	rotation[1] = theta;
	rotation[2] = psi;
}

/// <summary>
/// Extract translation Vector3 from the Matrix4 4x4 transformation in M41,M42,M43
/// </summary>
/// <param name="transform">The transform matrix.</param>
/// <param name="translation">Array of 3 floating point values for translation.</param>
void ExtractVector3Translation(const Matrix4 &transform, _Out_cap_c_(3) float *translation)
{
	translation[0] = transform.M41;
	translation[1] = transform.M42;
	translation[2] = transform.M43;
}

/// <summary>
/// Test whether the camera moved too far between sequential frames by looking at starting and end transformation matrix.
/// We assume that if the camera moves or rotates beyond a reasonable threshold, that we have lost track.
/// Note that on lower end machines, if the processing frame rate decreases below 30Hz, this limit will potentially have
/// to be increased as frames will be dropped and hence there will be a greater motion between successive frames.
/// </summary>
/// <param name="T_initial">The transform matrix from the previous frame.</param>
/// <param name="T_final">The transform matrix from the current frame.</param>
/// <param name="maxTrans">The maximum translation in meters we expect per x,y,z component between frames under normal motion.</param>
/// <param name="maxRotDegrees">The maximum rotation in degrees we expect about the x,y,z axes between frames under normal motion.</param>
/// <returns>true if camera transformation is greater than the threshold, otherwise false</returns>
bool CameraTransformFailed(const Matrix4 &T_initial, const Matrix4 &T_final, float maxTrans, float maxRotDegrees)
{
	// Check if the transform is too far out to be reasonable 
	float deltaTrans = maxTrans;
	float angDeg = maxRotDegrees;
	float deltaRot = (angDeg * (float)3.1415926) / 180.0f;

	// Calculate the deltas
	float eulerInitial[3];
	float eulerFinal[3];

	ExtractRot2Euler(T_initial, eulerInitial);
	ExtractRot2Euler(T_final, eulerFinal);

	float transInitial[3];
	float transFinal[3];

	ExtractVector3Translation(T_initial, transInitial);
	ExtractVector3Translation(T_final, transFinal);

	bool failRot = false;
	bool failTrans = false;

	float rDeltas[3];
	float tDeltas[3];

	static const float pi = static_cast<float>(3.1415926);

	for (int i = 0; i < 3; i++)
	{
		// Handle when one angle is near PI, and the other is near -PI.
		if (eulerInitial[i] >= (pi - deltaRot) && eulerFinal[i] < (deltaRot - pi))
		{
			eulerInitial[i] -= pi * 2;
		}
		else if (eulerFinal[i] >= (pi - deltaRot) && eulerInitial[i] < (deltaRot - pi))
		{
			eulerFinal[i] -= pi * 2;
		}

		rDeltas[i] = eulerInitial[i] - eulerFinal[i];
		tDeltas[i] = transInitial[i] - transFinal[i];

		if (fabs(rDeltas[i]) > deltaRot)
		{
			failRot = true;
			break;
		}
		if (fabs(tDeltas[i]) > deltaTrans)
		{
			failTrans = true;
			break;
		}
	}

	return failRot || failTrans;
}

template <typename T>
inline T clamp(const T& x, const T& a, const T& b)
{
	if (x < a)
		return a;
	else if (x > b)
		return b;
	else
		return x;
}

/// <summary>
/// Color the residual/delta image from the AlignDepthFloatToReconstruction call
/// </summary>
/// <param name="pFloatDeltaFromReference">A pointer to the source FloatDeltaFromReference image.</param>
/// <param name="pShadedDeltaFromReference">A pointer to the destination color ShadedDeltaFromReference image.</param>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT ColorResiduals(const NUI_FUSION_IMAGE_FRAME *pFloatDeltaFromReference,
	const NUI_FUSION_IMAGE_FRAME *pShadedDeltaFromReference)
{
	HRESULT hr = S_OK;

	if (nullptr == pShadedDeltaFromReference ||
		nullptr == pFloatDeltaFromReference)
	{
		return E_FAIL;
	}

	if (nullptr == pShadedDeltaFromReference->pFrameTexture ||
		nullptr == pFloatDeltaFromReference->pFrameTexture)
	{
		return E_NOINTERFACE;
	}

	if (pFloatDeltaFromReference->imageType != NUI_FUSION_IMAGE_TYPE_FLOAT || pShadedDeltaFromReference->imageType != NUI_FUSION_IMAGE_TYPE_COLOR)
	{
		return E_INVALIDARG;
	}

	unsigned int width = pShadedDeltaFromReference->width;
	unsigned int height = pShadedDeltaFromReference->height;

	if (width != pFloatDeltaFromReference->width
		|| height != pFloatDeltaFromReference->height)
	{
		return E_INVALIDARG;
	}

	// 32bit ABGR color pixels for shaded image
	NUI_LOCKED_RECT shadedDeltasLockedRect;
	hr = pShadedDeltaFromReference->pFrameTexture->LockRect(0, &shadedDeltasLockedRect, nullptr, 0);
	if (FAILED(hr) || shadedDeltasLockedRect.Pitch == 0)
	{
		return hr;
	}

	// 32bit float per pixel signifies distance delta from the reconstructed surface model after AlignDepthFloatToReconstruction
	NUI_LOCKED_RECT floatDeltasLockedRect;
	hr = pFloatDeltaFromReference->pFrameTexture->LockRect(0, &floatDeltasLockedRect, nullptr, 0);
	if (FAILED(hr) || floatDeltasLockedRect.Pitch == 0)
	{
		return hr;
	}

	unsigned int *pColorBuffer = reinterpret_cast<unsigned int *>(shadedDeltasLockedRect.pBits);
	const float *pFloatBuffer = reinterpret_cast<float *>(floatDeltasLockedRect.pBits);

	Concurrency::parallel_for(0u, height, [&](unsigned int y)
	{
		unsigned int* pColorRow = reinterpret_cast<unsigned int*>(reinterpret_cast<unsigned char*>(pColorBuffer)+(y * shadedDeltasLockedRect.Pitch));
		const float* pFloatRow = reinterpret_cast<const float*>(reinterpret_cast<const unsigned char*>(pFloatBuffer)+(y * floatDeltasLockedRect.Pitch));

		for (unsigned int x = 0; x < width; ++x)
		{
			float residue = pFloatRow[x];
			unsigned int color = 0;

			if (residue <= 1.0f)   // Pixel byte ordering: ARGB
			{
				color |= (255 << 24);                                                                               // a
				color |= (static_cast<unsigned char>(255.0f * clamp(1.0f + residue, 0.0f, 1.0f)) << 16);            // r
				color |= (static_cast<unsigned char>(255.0f * clamp(1.0f - std::abs(residue), 0.0f, 1.0f)) << 8);   // g
				color |= (static_cast<unsigned char>(255.0f * clamp(1.0f - residue, 0.0f, 1.0f)));                  // b
			}

			pColorRow[x] = color;
		}
	});

	pShadedDeltaFromReference->pFrameTexture->UnlockRect(0);
	pFloatDeltaFromReference->pFrameTexture->UnlockRect(0);

	return hr;
}


/// <summary>
/// Constructor
/// </summary>
KinectFusionProcessor::KinectFusionProcessor() :
m_hWnd(nullptr),
m_pVolume(nullptr),
m_bPauseIntegration(false),
//m_hrRecreateVolume(S_OK),
m_pNuiSensor(nullptr),
m_hNextDepthFrameEvent(INVALID_HANDLE_VALUE),
m_pDepthStreamHandle(INVALID_HANDLE_VALUE),
m_hNextColorFrameEvent(INVALID_HANDLE_VALUE),
m_pColorStreamHandle(INVALID_HANDLE_VALUE),
m_cLostFrameCounter(0),
m_bTrackingFailed(false),
m_cFrameCounter(0),
m_fFrameCounterStartTime(0),
m_cLastDepthFrameTimeStamp(0),
m_cLastColorFrameTimeStamp(0),
m_fMostRecentRaycastTime(0),
m_pDepthImagePixelBuffer(nullptr),
m_pColorCoordinates(nullptr),
m_pMapper(nullptr),
m_cPixelBufferLength(0),
m_cColorCoordinateBufferLength(0),
m_pDepthFloatImage(nullptr),
m_pColorImage(nullptr),
m_pResampledColorImage(nullptr),
m_pResampledColorImageDepthAligned(nullptr),
m_pSmoothDepthFloatImage(nullptr),
m_pDepthPointCloud(nullptr),
m_pRaycastPointCloud(nullptr),
m_pRaycastDepthFloatImage(nullptr),
m_pShadedSurface(nullptr),
m_pShadedSurfaceNormals(nullptr),
m_pCapturedSurfaceColor(nullptr),
m_pFloatDeltaFromReference(nullptr),
m_pShadedDeltaFromReference(nullptr),
m_bKinectFusionInitialized(false),
m_bResetReconstruction(false),
m_bIntegrationResumed(false),
m_pCameraPoseFinder(nullptr),
m_bTrackingHasFailedPreviously(false),
m_pDownsampledDepthFloatImage(nullptr),
m_pDownsampledSmoothDepthFloatImage(nullptr),
m_pDownsampledDepthPointCloud(nullptr),
m_pDownsampledShadedDeltaFromReference(nullptr),
m_pDownsampledRaycastPointCloud(nullptr),
m_bCalculateDeltaFrame(false),
m_savePCLNBC(false),
m_savePCLNB(false),
m_saveCloud(false),
m_bAutoFindCameraPoseWhenLost(true),
m_bRecreateVolume(false),
m_bNearMode(false),
m_hrRecreateVolume(-1),
m_processorType(NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_AMP),
m_deviceIndex(-1),
m_deviceMemory(0),
m_bTranslateResetPoseByMinDepthThreshold(true),
m_bMirrorDepthFrame(false),
m_bCaptureColor(true),
m_fMinDepthThreshold(NUI_FUSION_DEFAULT_MINIMUM_DEPTH),
m_fMaxDepthThreshold(NUI_FUSION_DEFAULT_MAXIMUM_DEPTH),
m_bAutoResetReconstructionOnTimeout(false),
m_cSmoothingKernelWidth(1),                // 0=just copy, 1=3x3, 2=5x5, 3=7x7, here we create a 3x3 kernel
m_fSmoothingDistanceThreshold(0.04f),      // 4cm, could use up to around 0.1f
m_fMaxTranslationDelta(0.3f),               // 0.15 - 0.3m per frame typical
m_fMaxRotationDelta(20.0f),                 // 10-20 degrees per frame typical
m_fMaxAlignToReconstructionEnergyForSuccess(0.15f),
m_cColorIntegrationInterval(3),
m_cDeltaFromReferenceFrameCalculationInterval(2),
m_bAutoResetReconstructionWhenLost(false),
m_cMinSuccessfulTrackingFramesForCameraPoseFinderAfterFailure(200),
m_cMaxIntegrationWeight(NUI_FUSION_DEFAULT_INTEGRATION_WEIGHT),
m_bDisplaySurfaceNormals(false),
m_cMinSuccessfulTrackingFramesForCameraPoseFinder(45),
m_cCameraPoseFinderProcessFrameCalculationInterval(5),
m_fFramesPerSecond(0.0),
m_fCameraPoseFinderDistanceThresholdReject(1.0f),
m_fMaxAlignPointCloudsEnergyForSuccess(0.006f),
m_cMaxCameraPoseFinderPoseTests(5),
m_fMinAlignPointCloudsEnergyForSuccess(0.0f),
m_fMinAlignToReconstructionEnergyForSuccess(0.005f),
m_fCameraPoseFinderDistanceThresholdAccept(0.4f),
m_bGetColorFrame(false),
m_gotKinectRGB(false)
{
	m_hNextDepthFrameEvent = CreateEvent(
		nullptr,
		TRUE, /* bManualReset */
		FALSE, /* bInitialState */
		nullptr);
	m_hNextColorFrameEvent = CreateEvent(
		nullptr,
		TRUE, /* bManualReset */
		FALSE, /* bInitialState */
		nullptr);

	SetIdentityMatrix(m_worldToCameraTransform);
	SetIdentityMatrix(m_defaultWorldToVolumeTransform);

	// Define a cubic Kinect Fusion reconstruction volume, with the sensor at the center of the
	// front face and the volume directly in front of sensor.
	m_reconstructionParams.voxelsPerMeter = 256;    // 1000mm / 256vpm = ~3.9mm/voxel
	m_reconstructionParams.voxelCountX = 512;       // 512 / 256vpm = 2m wide reconstruction
	m_reconstructionParams.voxelCountY = 384;       // Memory = 512*384*512 * 4bytes per voxel
	m_reconstructionParams.voxelCountZ = 512;       // This will require a GPU with at least 512MB

	cloudShade.create(480, 640, CV_8UC4);
	m_imgDepth.create(480, 640, CV_32FC1);
	cloudRGB.create(480, 640, CV_8UC4);
	m_imgColor.create(480, 640, CV_8UC4);
	kinectRGB.create(480,640,CV_8UC4);
	dert.create(480, 640, CV_8UC4);
	dis.create(480, 640, CV_8UC3);
	/*stereoImg.create(480, 1280, CV_8UC3);*/
	stereoPic.create(480, 1280, CV_8UC3);
	frame.create(480, 1280, CV_8UC3);

	cap.open(0);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
	if (!cap.isOpened()) // check if we succeeded
	{
		cout << "cant open the camera!" << endl;
		getchar();
		return;
	}

	//stereo = new stereo_camera;
	//namedWindow("cloud", WINDOW_AUTOSIZE);
	namedWindow("depth", WINDOW_AUTOSIZE);
	namedWindow("dis", WINDOW_AUTOSIZE);
	namedWindow("color", WINDOW_AUTOSIZE);
	namedWindow("stereoview",WINDOW_AUTOSIZE);

}


/// <summary>
/// Destructor
/// </summary>
KinectFusionProcessor::~KinectFusionProcessor()
{
	// Clean up Kinect Fusion
	SafeRelease(m_pVolume);
	SafeRelease(m_pMapper);

	// Clean up Kinect Fusion Camera Pose Finder
	SafeRelease(m_pCameraPoseFinder);

	SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pDepthFloatImage);
	SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pColorImage);
	SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pResampledColorImage);
	SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pResampledColorImageDepthAligned);
	SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pRaycastPointCloud);
	SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pRaycastDepthFloatImage);
	SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pShadedSurface);
	SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pShadedSurfaceNormals);
	SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pCapturedSurfaceColor);
	SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pFloatDeltaFromReference);
	SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pShadedDeltaFromReference);
	SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pSmoothDepthFloatImage);
	SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pDepthPointCloud);
	SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pDownsampledDepthFloatImage);
	SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pDownsampledSmoothDepthFloatImage);
	SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pDownsampledDepthPointCloud);
	SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pDownsampledShadedDeltaFromReference);

	// Clean up the depth pixel array
	SAFE_DELETE_ARRAY(m_pDepthImagePixelBuffer);

	// Clean up the color coordinate array
	SAFE_DELETE_ARRAY(m_pColorCoordinates);

	// Clean up synchronization objects
	if (m_hNextDepthFrameEvent != INVALID_HANDLE_VALUE)
	{
		CloseHandle(m_hNextDepthFrameEvent);
	}
	if (m_hNextColorFrameEvent != INVALID_HANDLE_VALUE)
	{
		CloseHandle(m_hNextColorFrameEvent);
	}
	//free(D1_data);
	//free(D2_data);

	destroyAllWindows();
}

int KinectFusionProcessor::Display()
{
	//char key;
	//imshow("cloud", cloudRGB);
	//imshow("cloud", cloudShade);
	imshow("depth", m_imgDepth);
	//imshow("dert", dert);
	imshow("dis", dis);
	imshow("color", m_imgColor);
	imshow("stereoview", stereoPic);
	return 0;
}

/// <summary>
/// Shuts down the sensor
/// </summary>
void KinectFusionProcessor::ShutdownSensor()
{
	// Clean up Kinect
	if (m_pNuiSensor != nullptr)
	{
		m_pNuiSensor->NuiShutdown();
		SafeRelease(m_pNuiSensor);
	}
}

/// <summary>
/// Is reconstruction volume initialized
/// </summary>
bool KinectFusionProcessor::IsVolumeInitialized()
{
	return nullptr != m_pVolume;
}

/// <summary>
/// Is the camera pose finder initialized and running.
/// </summary>
bool KinectFusionProcessor::IsCameraPoseFinderAvailable()
{
	return m_bAutoFindCameraPoseWhenLost
		&& nullptr != m_pCameraPoseFinder
		&& m_pCameraPoseFinder->GetStoredPoseCount() > 0;
}
//
//void KinectFusionProcessor::cutAndSaveImg(Mat& src_img, char* path1, char* path2)
//{
//	Mat left, right;
//	flip(src_img, src_img, 1);
//	left = src_img.colRange(0, 640);
//	right = src_img.colRange(640, 1280);
//	imwrite(path1, left);
//	imwrite(path2, right);
//}

/// <summary>
/// Main processing function
/// </summary>
DWORD KinectFusionProcessor::MainLoop()
{
	HRESULT hr = S_OK;

	hr = CreateFirstConnected();
	if (FAILED(hr))
	{
		cout << ("Failed to Create first connection.")<<endl;
		getchar();
		return hr;
	}

	hr = InitializeKinectFusion();
	if (FAILED(hr))
	{
		cout << ("Failed to initalize KinectFusion.")<<endl;
		getchar();
		return hr;
	}
//	stereo.openCamera();
	/*stereo.getFullPic(stereoPic);*/
	bool bStopProcessing = false;
	pclProcess.runRGBManual();

	// Main loop
	while (!bStopProcessing)
	{
		DWORD waitResult = WaitForSingleObject(m_hNextDepthFrameEvent, INFINITE);

		switch (waitResult)
		{
		case 0xFFFFFFFF: // m_hStopProcessingEvent
			bStopProcessing = true;
			break;

		case 0: // m_hNextDepthFrameEvent
		{
					if (m_bKinectFusionInitialized)
					{
						if (!m_bRecreateVolume)
						{
							m_hrRecreateVolume = RecreateVolume();
						}

						else if (m_bResetReconstruction)
						{
							HRESULT hr = InternalResetReconstruction();

							if (SUCCEEDED(hr))
							{
								cout << ("Reconstruction has been reset.") << endl;
							}
							else
							{
								cout << "Failed to reset reconstruction." << endl;
							}
							m_bResetReconstruction = false;
						}

						ProcessDepth();
						//gengeratePointsNB(m_pRaycastPointCloud, pclProcess.basicWithNorm_cloud_ptr);

						if (addedPose==TRUE)
						{
							gengeratePointsNC(m_pRaycastPointCloud, pclProcess.point_cloud_ptr);
						/*	stereo.getTwoPic(left, right);*/
							/*stereo.prossNew(left, right, dis, pclProcess.point_cloud_stereo_ptr);*/
							prossStereo(left,right,dis,pclProcess.point_cloud_stereo_ptr);
							pclProcess.blashRGBManual();	
						}
						else
						{
							getFullPic(stereoPic);
							/*stereo.getFullPic(stereoPic);*/
						}

						decide();
						save();

						Display();
					
					}
					break;
			}
			break;

		default:
			bStopProcessing = true;
		}
	}

	ShutdownSensor();
	return 0;
}

//stereo get 480*1280 pic
int KinectFusionProcessor::getFullPic(Mat & mat)
{
	cap >> mat;
	return 0;
}

int KinectFusionProcessor::prossStereo(Mat & mat1,
	Mat& mat2,
	Mat &dis,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloudPCL)
{
	cap >> frame;
	flip(frame, frame, 1);
	left = frame.colRange(0, 640);
	right = frame.colRange(640, 1280);

	Mat tmp1, tmp2;
	tmp1.create(mat1.rows, mat1.cols, CV_8SC1);
	tmp2.create(mat1.rows, mat1.cols, CV_8SC1);

	cvtColor(mat1, tmp1, CV_BGR2GRAY);
	cvtColor(mat2, tmp2, CV_BGR2GRAY);
	recpcl.rectify(tmp1, tmp2);

	image<uchar> *I1 = new image<uchar>(mat1.cols, mat1.rows);
	image<uchar> *I2 = new image<uchar>(mat2.cols, mat2.rows);
	loadMat(I1, tmp1, mat1.cols, mat1.rows);
	loadMat(I2, tmp2, mat2.cols, mat2.rows);

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
	recpcl.colorCloud(mat1, pCloudPCL);
	//pCloudPCL = recpcl.pCloudPCL;

	// free memory
	delete I1;
	delete I2;
	delete D1;
	return 1;
}

/// <summary>
/// Main processing function
/// </summary>
void KinectFusionProcessor::decide()
{
	char key;
	key = cv::waitKey(1);
	switch (key)
	{
	case(97) :     // A
		if (flagState == free &&flagWitch == noon)  // 单拍彩色
		{
			flagState = busying;
			flagWitch = color;
			std::cout << "we going save this color frame!....................................." << endl;
		}
		break;

	case (115) :       //S
		if (flagState == free&&flagWitch == noon)  // 开始连拍彩色
		{
			flagState = busying;
			flagWitch = colorS;
			std::cout << "we going save this color seres!................" << endl;
		}
		else if (flagState == busying&&flagWitch == colorS)  // 停止连拍彩色
		{
			flagState = free;
			flagWitch = noon;
			std::cout << "we going stop save this color seres!.........." << endl;
		}
		break;

	case(100) :           //D
		if (flagState == free&&flagWitch == noon) // 单拍深度
		{
			flagState = busying;
			flagWitch = depth;
			std::cout << "we going save this depth frame!。。。。。。。。" << endl;
		}
		break;

	case(102) :
		if (flagState == free&&flagWitch == noon) // F   开始连拍深度
		{
			flagState = busying;
			flagWitch = depthS;
			std::cout << "we going save this depth serial!。。。。。。。" << endl;
		}
		else if (flagState == busying && flagWitch == depthS)// 停止连拍深度
		{
			flagState = free;
			flagWitch = noon;
			std::cout << "we going stop save this depth serise!。。。。。。。" << endl;
		}
		break;

	case(113) :       //Q
		if (flagState == free&&flagWitch == noon)  // 保存彩色视频
		{
			flagState = busying;
			flagWitch = colorVideo;
			flagSavingCVideo = true;
			std::cout << "we going save this color video!。。。。。。。" << endl;
		}
		else if (flagState == busying&&flagWitch == colorVideo)  // 停止保存彩色视频
		{
			flagState = busying;
			flagWitch = noon;
			std::cout << "we going stop save this color video!。。。。。。。" << endl;
		}
		break;

	case(119) :            //W
		if (flagState == free&&flagWitch == noon)  // 保存深度视频
		{
			flagState = busying;
			flagWitch = depthVideo;
			flagSavingDVideo = true;
			std::cout << "we going save this depth video!。。。。。。。。。。。。" << endl;
		}
		else if (flagState == busying&&flagWitch == depthVideo)  // 停止保存深度视频
		{
			flagState = busying;
			flagWitch = noon;
			std::cout << "we going stop save this depth video!。。。。。。。。。。。。" << endl;
		}
		break;

	case(101) :         //E
		if (flagState == free&&flagWitch == noon)  //深度彩色同时连拍  拍一帧
		{
			flagState = busying;
			flagWitch = colorAnddepth;
			std::cout << "we going save this color and depth frame!。。。。。。。。。" << endl;
		}
		break;

	case(114) :         //R
		if (flagState == free&&flagWitch == noon)                                        //深度彩色同时连拍多帧   
		{
			flagState = busying;
			flagWitch = colorSanddepthS;
			std::cout << "we going save this color and depth seres frame!。。。。。。。。。" << endl;
		}
		else if (flagState == busying&&flagWitch == colorSanddepthS)  // 停止深度彩色同时连拍多帧
		{
			flagState = free;
			flagWitch = noon;
			std::cout << "we going stop save this color and depth seres frame!。。。。。。。。。" << endl;
		}
		break;

	case(122) :           //Z
		flagState = busying;                 //	Reset reconstruction
		flagWitch = fusionReset;
		std::cout << "we going reset this reconstruction!。。。。。。。。。。。。。。。。。" << endl;
		break;

	case(120) :     //X              
		flagState = busying;                            //kinect mesh 保存
		flagWitch = meshSave;
		m_saveCloud = true;
		std::cout << "we going save this point cloud by kinect in mesh !。。。。。。。。。。。。" << endl;
		break;

	case(99) :                //C
		flagState = busying;                                      //彩色点云保存
		flagWitch = pclBasicSave;
		m_bGetColorFrame = true;
		std::cout << "we going save this ***** basic ******point cloud by PCL in PCD !。。。。。。。。" << endl;
		//m_savePCDcolor = true;
		break;

	case(118) :                //V
		flagState = busying;                                    // XYZ点云保存
		flagWitch = pclColorSave;    //

		std::cout << "we going save this ***** Color ***** point cloud by PCL in PCD !。。。。。。。。。。" << endl;
		//m_savePCDbasic = true;
		break;
	case(98) :                        //B
		flagState = busying;                                      //法线点云保存
		flagWitch = pclNormSave;                         
		std::cout << "we going save this ***** norm ****** point cloud by PCL in PCD !。。。。。。。。。。" << endl;
		break;

	case(103) :   //G
		flagState = busying;
		flagWitch = pclBasicWithNormSave;            //法线和XYZ点云保存           
		m_savePCLNB = true;
		std::cout << "we going save this ***** basicAndNorm ****** point cloud by PCL in PCD !。。。。。。。。。。" << endl;
		break;

	case(121) :   //Y
		flagState = busying;
		flagWitch = pclColorWithNormSave;                // 彩色法线点云保存
		m_bGetColorFrame = true;
		m_savePCLNBC = true;
		std::cout << "we going save this ***** ColorAndNorm ****** point cloud by PCL in PCD !。。。。。。。。。。" << endl;
		break;

	//case(107) :   //K
	//	flagState = busying;
	//	flagWitch = stereoSave;                // 彩色法线点云保存
	//	m_savePCLNBC = true;
	//	std::cout << "we going save this ***** ColorAndNorm ****** point cloud by PCL in PCD !。。。。。。。。。。" << endl;
	//	break;
	}

}

/// <summary>
/// Main processing function
/// </summary>
void KinectFusionProcessor::save()
{
	if (flagState == busying&&flagWitch == color)   // A 彩色单拍
	{
		char path[100];
		sprintf_s(path, 100, "data/color%d.jpg", colorpiccount);
		colorpiccount++;
		imwrite(path, m_imgColor);
		flagState = free;
		flagWitch = noon;
	}

	else if (flagState == busying&&flagWitch == colorS)    //S 彩色连拍
	{
		char path[100];
		sprintf_s(path, 100, "data/color%d.jpg", colorpiccount);
		colorpiccount++;
		imwrite(path, m_imgColor);
	}

	else if (flagState == busying&&flagWitch == depth)     //D   深度单拍
	{
		char path[100];
		sprintf_s(path, 100, "data/depth%d.jpg", depthpiccount);
		depthpiccount++;
		imwrite(path, m_imgDepth);
		flagState = free;
		flagWitch = noon;
	}

	else if (flagState == busying&&flagWitch == depthS)    //F 深度连拍
	{
		char path[100];
		sprintf_s(path, 100, "data/depth%d.jpg", depthpiccount);
		depthpiccount++;
		imwrite(path, m_imgDepth);
	}

	else if (flagState == busying&&flagWitch == colorVideo)    //Q 彩色视频
	{
		if (flagSavingCVideo)
		{
			flagSavingCVideo = false;
			char path[100];
			sprintf_s(path, 100, "data/Color%d.mpeg", colormpegcount);
			Size_<int> imgsize(640, 480);
			m_writeColor.open(path, ('P', 'I', 'M', '1'), 30, imgsize, 1);
		}
		else
			m_writeColor << m_imgColor;
	}
	else if (flagState == busying&&flagWitch == noon)    
	{
		flagState = free;
		flagWitch = noon;
		colormpegcount++;
	}

	else if (flagState == busying&&flagWitch == depthVideo)    //W 深度视频
	{
		if (flagSavingDVideo)
		{
			flagSavingDVideo = false;
			char path[100];
			sprintf_s(path, 100, "data/Depth%d.mpeg", depthmpegcount);
			Size_<int> imgsize(640, 480);
			//m_writeDepth.open(path, cv::VideoWriter::fourcc('P', 'I', 'M', '1'), 30, imgsize, 0);
			m_writeDepth.open(path, ('P', 'I', 'M', '1'), 25, imgsize, 0);

		}
		else
			m_writeDepth << m_imgDepth;
	}
	else if (flagState == busying&&flagWitch == noon)
	{
		flagState = free;
		flagWitch = noon;
		depthmpegcount++;
	}

	else if (flagState == busying&&flagWitch == colorAnddepth)   //E 深度彩色同时拍一次
	{
		if (flagChose % 2 == 0)
		{
			char path1[100];
			sprintf_s(path1, 100, "data/color%d.jpg", colorpiccount);
			colorpiccount++;
			imwrite(path1, m_imgColor);
		}
		else
		{
			char path2[100];
			sprintf_s(path2, 100, "data/Depth%d.jpg", depthpiccount);
			depthpiccount++;
			imwrite(path2, m_imgDepth);
		}

		flagChose++;

		if (flagChose == 2)
		{
			flagState = free;
			flagWitch = noon;
			flagChose = 0;
		}
	}

	else if (flagState == busying&&flagWitch == colorSanddepthS)    //R    彩色深度同时连拍
	{
		if (flagChose == 0)
		{
			char path1[100];
			sprintf_s(path1, 100, "data/color%d.jpg", colorpiccount);
			colorpiccount++;
			imwrite(path1, m_imgColor);
			flagChose = 1;
		}
		else
		{
			char path2[100];
			sprintf_s(path2, 100, "data/Depth%d.jpg", depthmpegcount);
			depthmpegcount++;
			imwrite(path2, m_imgDepth);
			flagChose = 0;
		}
	}

	else if (flagState == busying&&flagWitch == fusionReset)     //Z reset Kinect Fusion
	{
		ResetReconstruction();
		pclProcess.viewerl->removeAllPointClouds();
		flagState = free;
		flagWitch = noon;
	}

	else if (flagState == busying&&flagWitch == meshSave)     //X 　kinect Mesh save
	{

		stereoCount++;
		char path1[100], path2[100];

		sprintf(path1, "data/right%d.jpg", stereoCount);
		sprintf(path2, "data/left%d.jpg", stereoCount);

	/*	cutAndSaveImg(stereoImg, path1, path2);*/

		flagState = free;
		flagWitch = noon;
	}

	else if (flagState == busying&&flagWitch == pclBasicSave)     //C　kinect Basic pcd data Save
	{
		//pclProcess.saveBasicCloud();
		if (m_gotKinectRGB == true)
		{
			gengeratePointsB(m_pRaycastPointCloud, pclProcess.basic_cloud_ptr);
			pclProcess.saveBasicCloud();

			stereoCount++;
			char path1[100], path2[100],path3[100];

			sprintf(path1, "data/right%d.jpg", stereoCount);
			sprintf(path2, "data/left%d.jpg", stereoCount);
			sprintf(path3, "data/kinect%d.jpg", stereoCount);

			/*cutAndSaveImg(stereoImg, path1, path2);*/
			imwrite(path3,kinectRGB);
			//Cut_img(stereoImg,);
			flagState = free;
			flagWitch = noon;
			m_bGetColorFrame = false;
			m_gotKinectRGB = false;
		}

	}

	else if (flagState == busying&&flagWitch == pclColorSave)    //Ｖ　kinect color pcd data save 
	{
		gengeratePointsNC(m_pRaycastPointCloud, pclProcess.point_cloud_ptr);
		pclProcess.saveColorCloud();

		stereoCount++;
		char path1[100], path2[100];

		sprintf(path1, "data/right%d.jpg", stereoCount);
		sprintf(path2, "data/left%d.jpg", stereoCount);

		/*cutAndSaveImg(stereoImg, path1, path2);*/

		flagState = free;
		flagWitch = noon;
	}

	else if (flagState == busying&&flagWitch == pclNormSave)   //Ｂ　cloud only norm save
	{
		pclProcess.saveNormCloud();

		stereoCount++;
		char path1[100], path2[100];

		sprintf(path1, "data/right%d.jpg", stereoCount);
		sprintf(path2, "data/left%d.jpg", stereoCount);

	/*	cutAndSaveImg(stereoImg, path1, path2);*/

		flagState = free;
		flagWitch = noon;
	}

	else if (flagState == busying&&flagWitch == pclBasicWithNormSave)   //Ｇ　cloud with norm save    
	{
		pclProcess.saveBasicWithNormCloud();

		stereoCount++;
		char path1[100], path2[100];

		sprintf(path1, "data/right%d.jpg", stereoCount);
		sprintf(path2, "data/left%d.jpg", stereoCount);

		/*cutAndSaveImg(stereoImg, path1, path2);*/

		writeCameraPose();
		flagState = free;
		flagWitch = noon;
	}
	else if (flagState == busying&&flagWitch == pclColorWithNormSave)   // Y  colud with norm and color save
	{
		if (m_gotKinectRGB == true)
		{
		/*	pclProcess.saveColorWithNormCloud();*/
			pclProcess.saveColorCloud();
			stereoCount++;
			char path1[100], path2[100],path3[100];

			sprintf(path1, "data/right%d.jpg", stereoCount);
			sprintf(path2, "data/left%d.jpg", stereoCount);
			sprintf(path3, "data/kinect%d.jpg", stereoCount);

		/*	cutAndSaveImg(stereoImg,path1, path2);*/
			imwrite(path3,kinectRGB);

			flagState = free;
			flagWitch = noon;
			m_gotKinectRGB = false;
			m_bGetColorFrame = false;
		}
	}
}

/// <summary>
/// Main processing function
/// </summary>
HRESULT KinectFusionProcessor::CreateFirstConnected()
{
	INuiSensor * pNuiSensor;
	HRESULT hr;

	int iSensorCount = 0;
	hr = NuiGetSensorCount(&iSensorCount);
	if (FAILED(hr))
	{
		cout << ("No ready Kinect found!")<<endl;
		return hr;
	}

	// Look at each Kinect sensor
	for (int i = 0; i < iSensorCount; ++i)
	{
		// Create the sensor so we can check status, if we can't create it, move on to the next
		hr = NuiCreateSensorByIndex(i, &pNuiSensor);
		if (FAILED(hr))
		{
			continue;
		}

		// Get the status of the sensor, and if connected, then we can initialize it
		hr = pNuiSensor->NuiStatus();
		if (S_OK == hr)
		{
			m_pNuiSensor = pNuiSensor;
			break;
		}

		// This sensor wasn't OK, so release it since we're not using it
		pNuiSensor->Release();
	}

	if (nullptr != m_pNuiSensor)
	{
		// Initialize the Kinect and specify that we'll be using depth
		hr = m_pNuiSensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH | NUI_INITIALIZE_FLAG_USES_COLOR);
		if (SUCCEEDED(hr))
		{
			// Open a depth image stream to receive depth frames
			hr = m_pNuiSensor->NuiImageStreamOpen(
				NUI_IMAGE_TYPE_DEPTH,
				NUI_IMAGE_RESOLUTION_640x480,
				0,
				2,
				m_hNextDepthFrameEvent,
				&m_pDepthStreamHandle);

			if (SUCCEEDED(hr))
			{
				// Open a color image stream to receive color frames
				hr = m_pNuiSensor->NuiImageStreamOpen(
					NUI_IMAGE_TYPE_COLOR,
					NUI_IMAGE_RESOLUTION_640x480,
					0,
					2,
					m_hNextColorFrameEvent,
					&m_pColorStreamHandle);
			}

			if (SUCCEEDED(hr))
			{
				// Create the coordinate mapper for converting color to depth space
				hr = m_pNuiSensor->NuiGetCoordinateMapper(&m_pMapper);
			}

			if (SUCCEEDED(hr) && m_bNearMode)
			{
				HRESULT hr = m_pNuiSensor->NuiImageStreamSetImageFrameFlags(
					m_pDepthStreamHandle,
					NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE);
			}
		}
		else
		{
			// Reset the event to non-signaled state
			ResetEvent(m_hNextDepthFrameEvent);
			ResetEvent(m_hNextColorFrameEvent);
		}
	}

	if (nullptr == m_pNuiSensor || FAILED(hr))
	{
		cout << ("No ready Kinect found!")<<endl;
		return E_FAIL;
	}

	return hr;
}

/// <summary>
/// Indicates whether the current reconstruction volume is different than the one in the params.
/// </summary>
bool  KinectFusionProcessor::VolumeChanged(const NUI_FUSION_RECONSTRUCTION_PARAMETERS& params)
{
	return
		m_reconstructionParams.voxelCountX != params.voxelCountX ||
		m_reconstructionParams.voxelCountY != params.voxelCountY ||
		m_reconstructionParams.voxelCountZ != params.voxelCountZ ||
		m_reconstructionParams.voxelsPerMeter != params.voxelsPerMeter;
}

/// <summary>
/// Initialize Kinect Fusion volume and images for processing
/// </summary>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT KinectFusionProcessor::InitializeKinectFusion()
{
	HRESULT hr = S_OK;

	// Check to ensure suitable DirectX11 compatible hardware exists before initializing Kinect Fusion
	WCHAR description[MAX_PATH];
	WCHAR instancePath[MAX_PATH];

	if (FAILED(hr = NuiFusionGetDeviceInfo(
		m_processorType,
		m_deviceIndex,
		&description[0],
		ARRAYSIZE(description),
		&instancePath[0],
		ARRAYSIZE(instancePath),
		&m_deviceMemory)))
	{
		if (hr == E_NUI_BADINDEX)
		{
			// This error code is returned either when the device index is out of range for the processor 
			// type or there is no DirectX11 capable device installed. As we set -1 (auto-select default) 
			// for the device index in the parameters, this indicates that there is no DirectX11 capable 
			// device. The options for users in this case are to either install a DirectX11 capable device
			// (see documentation for recommended GPUs) or to switch to non-real-time CPU based 
			// reconstruction by changing the processor type to NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_CPU.
			cout << ("No DirectX11 device detected, or invalid device index - Kinect Fusion requires a DirectX11 device for GPU-based reconstruction.")<<endl;
		}
		else
		{
			cout << ("Failed in call to NuiFusionGetDeviceInfo.")<<endl;
		}
		return hr;
	}

	unsigned int width =640;
	unsigned int height = 480;

	unsigned int colorWidth = 640;
	unsigned int colorHeight = 480;

	// Calculate the down sampled image sizes, which are used for the AlignPointClouds calculation frames
	unsigned int downsampledWidth = width / 2;
	unsigned int downsampledHeight = height / 2;

	// Frame generated from the depth input
	if (FAILED(hr = CreateFrame(NUI_FUSION_IMAGE_TYPE_FLOAT, width, height, &m_pDepthFloatImage)))
	{
		return hr;
	}

	// Frames generated from the depth input
	if (FAILED(hr = CreateFrame(NUI_FUSION_IMAGE_TYPE_FLOAT, downsampledWidth, downsampledHeight, &m_pDownsampledDepthFloatImage)))
	{
		return hr;
	}

	// Frame generated from the raw color input of Kinect
	if (FAILED(hr = CreateFrame(NUI_FUSION_IMAGE_TYPE_COLOR, colorWidth, colorHeight, &m_pColorImage)))
	{
		return hr;
	}

	// Frame generated from the raw color input of Kinect for use in the camera pose finder.
	// Note color will be down-sampled to the depth size if depth and color capture resolutions differ.
	if (FAILED(hr = CreateFrame(NUI_FUSION_IMAGE_TYPE_COLOR, width, height, &m_pResampledColorImage)))
	{
		return hr;
	}

	// Frame re-sampled from the color input of Kinect, aligned to depth - this will be the same size as the depth.
	if (FAILED(hr = CreateFrame(NUI_FUSION_IMAGE_TYPE_COLOR, width, height, &m_pResampledColorImageDepthAligned)))
	{
		return hr;
	}

	// Point Cloud generated from ray-casting the volume
	if (FAILED(hr = CreateFrame(NUI_FUSION_IMAGE_TYPE_POINT_CLOUD, width, height, &m_pRaycastPointCloud)))
	{
		return hr;
	}

	// Point Cloud generated from ray-casting the volume
	if (FAILED(hr = CreateFrame(NUI_FUSION_IMAGE_TYPE_POINT_CLOUD, downsampledWidth, downsampledHeight, &m_pDownsampledRaycastPointCloud)))
	{
		return hr;
	}

	// Depth frame generated from ray-casting the volume
	if (FAILED(hr = CreateFrame(NUI_FUSION_IMAGE_TYPE_FLOAT, width, height, &m_pRaycastDepthFloatImage)))
	{
		return hr;
	}

	// Image of the raycast Volume to display
	if (FAILED(hr = CreateFrame(NUI_FUSION_IMAGE_TYPE_COLOR, width, height, &m_pShadedSurface)))
	{
		return hr;
	}

	// Image of the raycast Volume with surface normals to display
	if (FAILED(hr = CreateFrame(NUI_FUSION_IMAGE_TYPE_COLOR, width, height, &m_pShadedSurfaceNormals)))
	{
		return hr;
	}

	// Image of the raycast Volume with the captured color to display
	if (FAILED(hr = CreateFrame(NUI_FUSION_IMAGE_TYPE_COLOR, width, height, &m_pCapturedSurfaceColor)))
	{
		return hr;
	}

	// Image of the camera tracking deltas to display
	if (FAILED(hr = CreateFrame(NUI_FUSION_IMAGE_TYPE_FLOAT, width, height, &m_pFloatDeltaFromReference)))
	{
		return hr;
	}

	// Image of the camera tracking deltas to display
	if (FAILED(hr = CreateFrame(NUI_FUSION_IMAGE_TYPE_COLOR, width, height, &m_pShadedDeltaFromReference)))
	{
		return hr;
	}

	// Image of the camera tracking deltas to display
	if (FAILED(hr = CreateFrame(NUI_FUSION_IMAGE_TYPE_COLOR, downsampledWidth, downsampledHeight, &m_pDownsampledShadedDeltaFromReference)))
	{
		return hr;
	}

	// Image from input depth for use with AlignPointClouds call
	if (FAILED(hr = CreateFrame(NUI_FUSION_IMAGE_TYPE_FLOAT, width, height, &m_pSmoothDepthFloatImage)))
	{
		return hr;
	}

	// Frames generated from smoothing the depth input
	if (FAILED(hr = CreateFrame(NUI_FUSION_IMAGE_TYPE_FLOAT, downsampledWidth, downsampledHeight, &m_pDownsampledSmoothDepthFloatImage)))
	{
		return hr;
	}

	// Image used in post pose finding success check AlignPointClouds call
	if (FAILED(hr = CreateFrame(NUI_FUSION_IMAGE_TYPE_POINT_CLOUD, width, height, &m_pDepthPointCloud)))
	{
		return hr;
	}

	// Point Cloud generated from depth input, in local camera coordinate system
	if (FAILED(hr = CreateFrame(NUI_FUSION_IMAGE_TYPE_POINT_CLOUD, downsampledWidth, downsampledHeight, &m_pDownsampledDepthPointCloud)))
	{
		return hr;
	}

	if (nullptr != m_pDepthImagePixelBuffer)
	{
		// If buffer length has changed, delete the old one.
		if (480*640 != m_cPixelBufferLength)
		{
			SAFE_DELETE_ARRAY(m_pDepthImagePixelBuffer);
		}
	}

	if (nullptr == m_pDepthImagePixelBuffer)
	{
		// Depth pixel array to capture data from Kinect sensor
		m_pDepthImagePixelBuffer =
			new(std::nothrow) NUI_DEPTH_IMAGE_PIXEL[640*480];

		if (nullptr == m_pDepthImagePixelBuffer)
		{
			cout << ("Failed to initialize Kinect Fusion depth image pixel buffer.")<<endl;
			return hr;
		}

		m_cPixelBufferLength = 640*480;
	}

	if (nullptr != m_pColorCoordinates)
	{
		// If buffer length has changed, delete the old one.
		if (640*480 != m_cColorCoordinateBufferLength)
		{
			SAFE_DELETE_ARRAY(m_pColorCoordinates);
		}
	}

	if (nullptr == m_pColorCoordinates)
	{
		// Color coordinate array to capture data from Kinect sensor and for color to depth mapping
		// Note: this must be the same size as the depth
		m_pColorCoordinates =
			new(std::nothrow) NUI_COLOR_IMAGE_POINT[640*480];

		if (nullptr == m_pColorCoordinates)
		{
			cout << ("Failed to initialize Kinect Fusion color image coordinate buffers.") << endl;
			return hr;
		}

		m_cColorCoordinateBufferLength = 640*480;
	}

	if (nullptr != m_pCameraPoseFinder)
	{
		SafeRelease(m_pCameraPoseFinder);
	}

	// Create the camera pose finder if necessary
	if (nullptr == m_pCameraPoseFinder)
	{
		NUI_FUSION_CAMERA_POSE_FINDER_PARAMETERS cameraPoseFinderParameters;

		cameraPoseFinderParameters.featureSampleLocationsPerFrameCount = NUI_FUSION_CAMERA_POSE_FINDER_DEFAULT_FEATURE_LOCATIONS_PER_FRAME_COUNT;
		cameraPoseFinderParameters.maxPoseHistoryCount = NUI_FUSION_CAMERA_POSE_FINDER_DEFAULT_POSE_HISTORY_COUNT;
		cameraPoseFinderParameters.maxDepthThreshold = NUI_FUSION_CAMERA_POSE_FINDER_DEFAULT_MAX_DEPTH_THRESHOLD;

		if (FAILED(hr = NuiFusionCreateCameraPoseFinder(
			&cameraPoseFinderParameters,
			nullptr,
			&m_pCameraPoseFinder)))
		{
			return hr;
		}
	}

	m_bKinectFusionInitialized = true;
	return hr;
}

HRESULT KinectFusionProcessor::CreateFrame(
	NUI_FUSION_IMAGE_TYPE frameType,
	unsigned int imageWidth,
	unsigned int imageHeight,
	NUI_FUSION_IMAGE_FRAME** ppImageFrame)
{
	HRESULT hr = S_OK;

	if (nullptr != *ppImageFrame)
	{
		// If image size or type has changed, release the old one.
		if ((*ppImageFrame)->width != imageWidth ||
			(*ppImageFrame)->height != imageHeight ||
			(*ppImageFrame)->imageType != frameType)
		{
			static_cast<void>(NuiFusionReleaseImageFrame(*ppImageFrame));
			*ppImageFrame = nullptr;
		}
	}

	// Create a new frame as needed.
	if (nullptr == *ppImageFrame)
	{
		hr = NuiFusionCreateImageFrame(
			frameType,
			imageWidth,
			imageHeight,
			nullptr,
			ppImageFrame);

		if (FAILED(hr))
		{
			cout << ("Failed to initialize Kinect Fusion image.")<<endl;
		}
	}

	return hr;
}


/// <summary>
/// Release and re-create a Kinect Fusion Reconstruction Volume
/// </summary>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT KinectFusionProcessor::RecreateVolume()
{
	HRESULT hr = S_OK;

	// Clean up Kinect Fusion
	SafeRelease(m_pVolume);

	SetIdentityMatrix(m_worldToCameraTransform);

	// Create the Kinect Fusion Reconstruction Volume
	// Here we create a color volume, enabling optional color processing in the Integrate, ProcessFrame and CalculatePointCloud calls
	hr = NuiFusionCreateColorReconstruction(
		&m_reconstructionParams,
		m_processorType,
		m_deviceIndex,
		&m_worldToCameraTransform,
		&m_pVolume);

	if (FAILED(hr))
	{
		if (E_NUI_GPU_FAIL == hr)
		{
			char buf[MAX_PATH];
			sprintf_s(buf, "Device %d not able to run Kinect Fusion, or error initializing.", m_deviceIndex);
			cout << (buf)<<endl;
		}
		else if (E_NUI_GPU_OUTOFMEMORY == hr)
		{
			char buf[MAX_PATH];
			sprintf_s(buf, "Device %d out of memory error initializing reconstruction - try a smaller reconstruction volume.", m_deviceIndex);
			cout << (buf)<<endl;
		}
		else if (NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_CPU != m_processorType)
		{
			char buf[MAX_PATH];
			sprintf_s(buf, "Failed to initialize Kinect Fusion reconstruction volume on device %d.", m_deviceIndex);
			cout << (buf)<<endl;
		}
		else
		{
			char buf[MAX_PATH];
			sprintf_s(buf, "Failed to initialize Kinect Fusion reconstruction volume on CPU %d.", m_deviceIndex);
			cout << (buf)<<endl;
		}

		return hr;
	}
	else
	{
		// Save the default world to volume transformation to be optionally used in ResetReconstruction
		hr = m_pVolume->GetCurrentWorldToVolumeTransform(&m_defaultWorldToVolumeTransform);
		if (FAILED(hr))
		{
			cout << ("Failed in call to GetCurrentWorldToVolumeTransform.")<<endl;
			return hr;
		}

		if (m_bTranslateResetPoseByMinDepthThreshold)
		{
			// This call will set the world-volume transformation
			hr = InternalResetReconstruction();
			if (FAILED(hr))
			{
				return hr;
			}
		}
		else
		{
			// Reset pause and signal that the integration resumed
			ResetTracking();
		}

		// Map X axis to blue channel, Y axis to green channel and Z axis to red channel,
		// normalizing each to the range [0, 1].
		SetIdentityMatrix(m_worldToBGRTransform);
		m_worldToBGRTransform.M11 = m_reconstructionParams.voxelsPerMeter / m_reconstructionParams.voxelCountX;
		m_worldToBGRTransform.M22 = m_reconstructionParams.voxelsPerMeter / m_reconstructionParams.voxelCountY;
		m_worldToBGRTransform.M33 = m_reconstructionParams.voxelsPerMeter / m_reconstructionParams.voxelCountZ;
		m_worldToBGRTransform.M41 = 0.5f;
		m_worldToBGRTransform.M42 = 0.5f;
		m_worldToBGRTransform.M44 = 1.0f;

		cout << ("Reconstruction has been reset.") << endl;
	}
	m_bRecreateVolume = true;
	return hr;
}

/// <summary>
/// Get Extended depth data
/// </summary>
/// <param name="imageFrame">The extended depth image frame to copy.</param>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT KinectFusionProcessor::CopyExtendedDepth(NUI_IMAGE_FRAME &imageFrame)
{
	HRESULT hr = S_OK;

	if (nullptr == m_pDepthImagePixelBuffer)
	{
		cout << ("Error depth image pixel buffer is nullptr.") << endl;
		return E_FAIL;
	}

	INuiFrameTexture *extendedDepthTex = nullptr;

	// Extract the extended depth in NUI_DEPTH_IMAGE_PIXEL format from the frame
	BOOL nearModeOperational = FALSE;
	hr = m_pNuiSensor->NuiImageFrameGetDepthImagePixelFrameTexture(
		m_pDepthStreamHandle,
		&imageFrame,
		&nearModeOperational,
		&extendedDepthTex);

	if (FAILED(hr))
	{
		cout << ("Error getting extended depth texture.")<<endl;
		return hr;
	}

	NUI_LOCKED_RECT extendedDepthLockedRect;

	// Lock the frame data to access the un-clamped NUI_DEPTH_IMAGE_PIXELs
	hr = extendedDepthTex->LockRect(0, &extendedDepthLockedRect, nullptr, 0);

	if (FAILED(hr) || extendedDepthLockedRect.Pitch == 0)
	{
		cout << ("Error getting extended depth texture pixels.")<<endl;
		return hr;
	}

	// Copy the depth pixels so we can return the image frame
	errno_t err = memcpy_s(
		m_pDepthImagePixelBuffer,
		640*480 * sizeof(NUI_DEPTH_IMAGE_PIXEL),
		extendedDepthLockedRect.pBits,
		extendedDepthTex->BufferLen());

	extendedDepthTex->UnlockRect(0);

	if (0 != err)
	{
		cout << ("Error copying extended depth texture pixels.") << endl;
		return hr;
	}

	return hr;
}

/// <summary>
/// Get Color data
/// </summary>
/// <param name="imageFrame">The color image frame to copy.</param>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT KinectFusionProcessor::CopyColor(NUI_IMAGE_FRAME &imageFrame,bool getColorFrame)
{
	HRESULT hr = S_OK;

	if (nullptr == m_pColorImage)
	{
		cout << ("Error copying color texture pixels.")<<endl;
		return E_FAIL;
	}

	INuiFrameTexture *srcColorTex = imageFrame.pFrameTexture;
	INuiFrameTexture *destColorTex = m_pColorImage->pFrameTexture;

	if (nullptr == srcColorTex || nullptr == destColorTex)
	{
		return E_NOINTERFACE;
	}

	// Lock the frame data to access the color pixels
	NUI_LOCKED_RECT srcLockedRect;

	hr = srcColorTex->LockRect(0, &srcLockedRect, nullptr, 0);

	if (FAILED(hr) || srcLockedRect.Pitch == 0)
	{
		cout << ("Error getting color texture pixels.")<<endl;
		return E_NOINTERFACE;
	}

	// Lock the frame data to access the color pixels
	NUI_LOCKED_RECT destLockedRect;

	hr = destColorTex->LockRect(0, &destLockedRect, nullptr, 0);

	if (FAILED(hr) || destLockedRect.Pitch == 0)
	{
		srcColorTex->UnlockRect(0);
		cout << ("Error copying color texture pixels.")<<endl;
		return E_NOINTERFACE;
	}

	// Copy the color pixels so we can return the image frame
	errno_t err = memcpy_s(
		destLockedRect.pBits,
		640*480* 4,
		srcLockedRect.pBits,
		srcLockedRect.size);
	if (getColorFrame)
	{
		uchar * dist = kinectRGB.data;
		errno_t err1 = memcpy_s(dist,
			640 * 480 * 4,
			srcLockedRect.pBits,
			srcLockedRect.size);
		if (err1 == 0)
			m_gotKinectRGB = true;
	}

	srcColorTex->UnlockRect(0);
	destColorTex->UnlockRect(0);

	if (0 != err)
	{
		cout << ("Error copying color texture pixels.")<<endl;
		hr = E_FAIL;
	}

	return hr;
}

/// <summary>
/// Adjust color to the same space as depth
/// </summary>
/// <returns>S_OK for success, or failure code</returns>
HRESULT KinectFusionProcessor::MapColorToDepth()
{
	HRESULT hr;

	if (nullptr == m_pColorImage || nullptr == m_pResampledColorImageDepthAligned
		|| nullptr == m_pDepthImagePixelBuffer || nullptr == m_pColorCoordinates)
	{
		return E_FAIL;
	}

	INuiFrameTexture *srcColorTex = m_pColorImage->pFrameTexture;
	INuiFrameTexture *destColorTex = m_pResampledColorImageDepthAligned->pFrameTexture;

	if (nullptr == srcColorTex || nullptr == destColorTex)
	{
		cout << ("Error accessing color textures.")<<endl;
		return E_NOINTERFACE;
	}

	// Lock the source color frame
	NUI_LOCKED_RECT srcLockedRect;

	// Lock the frame data to access the color pixels
	hr = srcColorTex->LockRect(0, &srcLockedRect, nullptr, 0);

	if (FAILED(hr) || srcLockedRect.Pitch == 0)
	{
		cout << ("Error accessing color texture pixels.")<<endl;
		return  E_FAIL;
	}

	// Lock the destination color frame
	NUI_LOCKED_RECT destLockedRect;

	// Lock the frame data to access the color pixels
	hr = destColorTex->LockRect(0, &destLockedRect, nullptr, 0);

	if (FAILED(hr) || destLockedRect.Pitch == 0)
	{
		srcColorTex->UnlockRect(0);
		cout << ("Error accessing color texture pixels.")<<endl;
		return  E_FAIL;
	}

	int *rawColorData = reinterpret_cast<int*>(srcLockedRect.pBits);
	int *colorDataInDepthFrame = reinterpret_cast<int*>(destLockedRect.pBits);

	// Get the coordinates to convert color to depth space
	hr = m_pMapper->MapDepthFrameToColorFrame(
		NUI_IMAGE_RESOLUTION_640x480,
		640*480,
		m_pDepthImagePixelBuffer,
		NUI_IMAGE_TYPE_COLOR,
		NUI_IMAGE_RESOLUTION_640x480,
		640*480,   // the color coordinates that get set are the same array size as the depth image
		m_pColorCoordinates);

	if (FAILED(hr))
	{
		srcColorTex->UnlockRect(0);
		destColorTex->UnlockRect(0);
		return hr;
	}

	// Loop over each row and column of the destination color image and copy from the source image
	// Note that we could also do this the other way, and convert the depth pixels into the color space, 
	// avoiding black areas in the converted color image and repeated color images in the background
	// However, then the depth would have radial and tangential distortion like the color camera image,
	// which is not ideal for Kinect Fusion reconstruction.
	if (m_bMirrorDepthFrame)
	{
		Concurrency::parallel_for(0, static_cast<int>(480), [&](int y)
		{
			unsigned int destIndex = y * 640;

			for (int x = 0; x < 640; ++x, ++destIndex)
			{
				// calculate index into depth array
				int colorInDepthX = m_pColorCoordinates[destIndex].x;
				int colorInDepthY = m_pColorCoordinates[destIndex].y;

				// make sure the depth pixel maps to a valid point in color space
				if (colorInDepthX >= 0 && colorInDepthX < 640
					&& colorInDepthY >= 0 && colorInDepthY < 480
					&& m_pDepthImagePixelBuffer[destIndex].depth != 0)
				{
					// Calculate index into color array
					unsigned int sourceColorIndex = colorInDepthX + (colorInDepthY * 640);

					// Copy color pixel
					colorDataInDepthFrame[destIndex] = rawColorData[sourceColorIndex];
				}
				else
				{
					colorDataInDepthFrame[destIndex] = 0;
				}
			}
		});
	}
	else
	{
		Concurrency::parallel_for(0, static_cast<int>(480), [&](int y)
		{
			unsigned int destIndex = y * 640;

			// Horizontal flip the color image as the standard depth image is flipped internally in Kinect Fusion
			// to give a viewpoint as though from behind the Kinect looking forward by default.
			unsigned int flippedDestIndex = destIndex + (640 - 1);

			for (int x = 0; x < 640; ++x, ++destIndex, --flippedDestIndex)
			{
				// calculate index into depth array
				int colorInDepthX = m_pColorCoordinates[destIndex].x;
				int colorInDepthY = m_pColorCoordinates[destIndex].y;

				// make sure the depth pixel maps to a valid point in color space
				if (colorInDepthX >= 0 && colorInDepthX < 640
					&& colorInDepthY >= 0 && colorInDepthY < 480
					&& m_pDepthImagePixelBuffer[destIndex].depth != 0)
				{
					// Calculate index into color array- this will perform a horizontal flip as well
					unsigned int sourceColorIndex = colorInDepthX + (colorInDepthY * 640);

					// Copy color pixel
					colorDataInDepthFrame[flippedDestIndex] = rawColorData[sourceColorIndex];
				}
				else
				{
					colorDataInDepthFrame[flippedDestIndex] = 0;
				}
			}
		});
	}

	srcColorTex->UnlockRect(0);
	destColorTex->UnlockRect(0);

	return hr;
}

/// <summary>
/// Get the next frames from Kinect, re-synchronizing depth with color if required.
/// </summary>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT KinectFusionProcessor::GetKinectFrames(bool &colorSynchronized)
{
	NUI_IMAGE_FRAME imageFrame;
	LONGLONG currentDepthFrameTime = 0;
	LONGLONG currentColorFrameTime = 0;
	colorSynchronized = true;   // assume we are synchronized to start with

	////////////////////////////////////////////////////////
	// Get an extended depth frame from Kinect

	HRESULT hr = m_pNuiSensor->NuiImageStreamGetNextFrame(m_pDepthStreamHandle, 0, &imageFrame);
	if (FAILED(hr))
	{
		cout << ("Kinect depth stream NuiImageStreamGetNextFrame call failed.")<<endl;
		return hr;
	}

	hr = CopyExtendedDepth(imageFrame);

	currentDepthFrameTime = imageFrame.liTimeStamp.QuadPart;

	// Release the Kinect camera frame
	m_pNuiSensor->NuiImageStreamReleaseFrame(m_pDepthStreamHandle, &imageFrame);

	if (FAILED(hr))
	{
		cout << ("Kinect depth stream NuiImageStreamReleaseFrame call failed.")<<endl;
		return hr;
	}

	////////////////////////////////////////////////////////
	// Get a color frame from Kinect

	currentColorFrameTime = m_cLastColorFrameTimeStamp;

	hr = m_pNuiSensor->NuiImageStreamGetNextFrame(m_pColorStreamHandle, 0, &imageFrame);
	if (FAILED(hr))
	{
		// Here we just do not integrate color rather than reporting an error
		colorSynchronized = false;
	}
	else
	{
		hr = CopyColor(imageFrame,m_bGetColorFrame);

		currentColorFrameTime = imageFrame.liTimeStamp.QuadPart;

		// Release the Kinect camera frame
		m_pNuiSensor->NuiImageStreamReleaseFrame(m_pColorStreamHandle, &imageFrame);

		if (FAILED(hr))
		{
			cout << ("Kinect color stream NuiImageStreamReleaseFrame call failed.")<<endl;
		}
	}

	// Check color and depth frame timestamps to ensure they were captured at the same time
	// If not, we attempt to re-synchronize by getting a new frame from the stream that is behind.
	int timestampDiff = static_cast<int>(abs(currentColorFrameTime - currentDepthFrameTime));

	if (timestampDiff >= cMinTimestampDifferenceForFrameReSync && m_cSuccessfulFrameCounter > 0 && (m_bAutoFindCameraPoseWhenLost || m_bCaptureColor))
	{
		// Get another frame to try and re-sync
		if (currentColorFrameTime - currentDepthFrameTime >= cMinTimestampDifferenceForFrameReSync)
		{
			// Perform camera tracking only from this current depth frame
			if (nullptr != m_pVolume)
			{
				// Convert the pixels describing extended depth as unsigned short type in millimeters to depth
				// as floating point type in meters.
				hr = m_pVolume->DepthToDepthFloatFrame(
					m_pDepthImagePixelBuffer,
					640*480 * sizeof(NUI_DEPTH_IMAGE_PIXEL),
					m_pDepthFloatImage,
					m_fMinDepthThreshold,
					m_fMaxDepthThreshold,
					m_bMirrorDepthFrame);

				if (FAILED(hr))
				{
					cout << ("Kinect Fusion NuiFusionDepthToDepthFloatFrame call failed.")<<endl;
					return hr;
				}

				Matrix4 calculatedCameraPose = m_worldToCameraTransform;
				FLOAT alignmentEnergy = 1.0f;

				hr = TrackCameraAlignPointClouds(calculatedCameraPose, alignmentEnergy);

				if (SUCCEEDED(hr))
				{
					m_worldToCameraTransform = calculatedCameraPose;

					// Raycast and set reference frame for tracking with AlignDepthFloatToReconstruction
					hr = SetReferenceFrame(m_worldToCameraTransform);

					SetTrackingSucceeded();
				}
				else
				{
					SetTrackingFailed();
				}
			}

			// Get another depth frame to try and re-sync as color ahead of depth
			hr = m_pNuiSensor->NuiImageStreamGetNextFrame(m_pDepthStreamHandle, timestampDiff, &imageFrame);
			if (FAILED(hr))
			{
				// Return silently, having performed camera tracking on the current depth frame
				return hr;
			}

			hr = CopyExtendedDepth(imageFrame);

			currentDepthFrameTime = imageFrame.liTimeStamp.QuadPart;

			// Release the Kinect camera frame
			m_pNuiSensor->NuiImageStreamReleaseFrame(m_pDepthStreamHandle, &imageFrame);

			if (FAILED(hr))
			{
				cout << ("Kinect depth stream NuiImageStreamReleaseFrame call failed.")<<endl;
				return hr;
			}
		}
		else if (currentDepthFrameTime - currentColorFrameTime >= cMinTimestampDifferenceForFrameReSync && WaitForSingleObject(m_hNextColorFrameEvent, 0) != WAIT_TIMEOUT)
		{
			// Get another color frame to try and re-sync as depth ahead of color and there is another color frame waiting
			hr = m_pNuiSensor->NuiImageStreamGetNextFrame(m_pColorStreamHandle, 0, &imageFrame);
			if (FAILED(hr))
			{
				// Here we just do not integrate color rather than reporting an error
				colorSynchronized = false;
			}
			else
			{
				hr = CopyColor(imageFrame,m_bGetColorFrame);

				currentColorFrameTime = imageFrame.liTimeStamp.QuadPart;

				// Release the Kinect camera frame
				m_pNuiSensor->NuiImageStreamReleaseFrame(m_pColorStreamHandle, &imageFrame);

				if (FAILED(hr))
				{
					cout << ("Kinect color stream NuiImageStreamReleaseFrame call failed.")<<endl;
					return hr;
				}
			}
		}

		timestampDiff = static_cast<int>(abs(currentColorFrameTime - currentDepthFrameTime));

		// If the difference is still too large, we do not want to integrate color
		if (timestampDiff > cMinTimestampDifferenceForFrameReSync || FAILED(hr))
		{
			colorSynchronized = false;
		}
		else
		{
			colorSynchronized = true;
		}
	}

	////////////////////////////////////////////////////////
	// To enable playback of a .xed file through Kinect Studio and reset of the reconstruction
	// if the .xed loops, we test for when the frame timestamp has skipped a large number. 
	// Note: this will potentially continually reset live reconstructions on slow machines which
	// cannot process a live frame in less time than the reset threshold. Increase the number of
	// milliseconds in cResetOnTimeStampSkippedMilliseconds if this is a problem.

	int cResetOnTimeStampSkippedMilliseconds = cResetOnTimeStampSkippedMillisecondsGPU;

	if (m_processorType == NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_CPU)
	{
		cResetOnTimeStampSkippedMilliseconds = cResetOnTimeStampSkippedMillisecondsCPU;
	}

	if (m_bAutoResetReconstructionOnTimeout && m_cFrameCounter != 0 && nullptr != m_pVolume
		&& abs(currentDepthFrameTime - m_cLastDepthFrameTimeStamp) > cResetOnTimeStampSkippedMilliseconds)
	{
		hr = InternalResetReconstruction();

		if (SUCCEEDED(hr))
		{
			cout << ("Reconstruction has been reset.") << endl;
		}
		else
		{
			cout << ("Failed to reset reconstruction.")<<endl;
		}
	}

	m_cLastDepthFrameTimeStamp = currentDepthFrameTime;
	m_cLastColorFrameTimeStamp = currentColorFrameTime;

	return hr;
}

/// <summary>
/// Perform camera tracking using AlignPointClouds
/// </summary>
HRESULT KinectFusionProcessor::TrackCameraAlignPointClouds(Matrix4 &calculatedCameraPose, FLOAT &alignmentEnergy)
{
	////////////////////////////////////////////////////////
	// Down sample the depth image

	HRESULT hr = DownsampleFrameNearestNeighbor(
		m_pDepthFloatImage,
		m_pDownsampledDepthFloatImage,
		2);

	if (FAILED(hr))
	{
		cout << ("Kinect Fusion DownsampleFrameNearestNeighbor call failed.")<<endl;
		return hr;
	}

	////////////////////////////////////////////////////////
	// Smooth depth image

	hr = m_pVolume->SmoothDepthFloatFrame(
		m_pDownsampledDepthFloatImage,
		m_pDownsampledSmoothDepthFloatImage,
		m_cSmoothingKernelWidth,
		m_fSmoothingDistanceThreshold);

	if (FAILED(hr))
	{
		cout << ("Kinect Fusion SmoothDepth call failed.")<<endl;
		return hr;
	}

	////////////////////////////////////////////////////////
	// Calculate Point Cloud from smoothed input Depth Image

	hr = NuiFusionDepthFloatFrameToPointCloud(
		m_pDownsampledSmoothDepthFloatImage,
		m_pDownsampledDepthPointCloud);

	if (FAILED(hr))
	{
		cout << ("Kinect Fusion NuiFusionDepthFloatFrameToPointCloud call failed.")<<endl;
		return hr;
	}

	////////////////////////////////////////////////////////
	// CalculatePointCloud

	// Raycast even if camera tracking failed, to enable us to visualize what is 
	// happening with the system
	hr = m_pVolume->CalculatePointCloud(
		m_pDownsampledRaycastPointCloud,
		nullptr,
		&calculatedCameraPose);

	if (FAILED(hr))
	{
		cout << ("Kinect Fusion CalculatePointCloud call failed.")<<endl;
		return hr;
	}

	////////////////////////////////////////////////////////
	// Call AlignPointClouds

	HRESULT tracking = S_OK;

	// Only calculate the residual delta from reference frame every m_cDeltaFromReferenceFrameCalculationInterval
	// frames to reduce computation time
	if (m_bCalculateDeltaFrame)
	{
		tracking = NuiFusionAlignPointClouds(
			m_pDownsampledRaycastPointCloud,
			m_pDownsampledDepthPointCloud,
			NUI_FUSION_DEFAULT_ALIGN_ITERATION_COUNT,
			m_pDownsampledShadedDeltaFromReference,
			&calculatedCameraPose);

		// Up sample the delta from reference image to display as the original resolution
		hr = UpsampleFrameNearestNeighbor(
			m_pDownsampledShadedDeltaFromReference,
			m_pShadedDeltaFromReference,
			2);

		if (FAILED(hr))
		{
			cout << ("Kinect Fusion UpsampleFrameNearestNeighbor call failed.")<<endl;
			return hr;
		}
	}
	else
	{
		tracking = NuiFusionAlignPointClouds(
			m_pDownsampledRaycastPointCloud,
			m_pDownsampledDepthPointCloud,
			NUI_FUSION_DEFAULT_ALIGN_ITERATION_COUNT,
			nullptr,
			&calculatedCameraPose);
	}
	/*
	cout << "align point cloud we have world to camera trans: " << 
		calculatedCameraPose.M11 << "      " << calculatedCameraPose.M12 << "      " << calculatedCameraPose.M13 << "      " << calculatedCameraPose.M14 << "      " <<
		calculatedCameraPose.M21 << "      " << calculatedCameraPose.M22 << "      " << calculatedCameraPose.M23 << "      " << calculatedCameraPose.M24 << "      " <<
		calculatedCameraPose.M31 << "      " << calculatedCameraPose.M32 << "      " << calculatedCameraPose.M33 << "      " << calculatedCameraPose.M34 << "      " <<
		calculatedCameraPose.M41 << "      " << calculatedCameraPose.M42 << "      " << calculatedCameraPose.M43 << "      " << calculatedCameraPose.M44 << "      " <<
		endl;
		*/
	if (!FAILED(tracking))
	{
		// Perform additional transform magnitude check
		// Camera Tracking has converged but did we get a sensible pose estimate?
		// see if relative rotation and translation exceed thresholds 
		if (CameraTransformFailed(
			m_worldToCameraTransform,
			calculatedCameraPose,
			m_fMaxTranslationDelta,
			m_fMaxRotationDelta))
		{
			// We calculated too large a move for this to be a sensible estimate,
			// quite possibly the camera tracking drifted. Force camera pose finding.
			hr = E_NUI_FUSION_TRACKING_ERROR;

			cout << (
				"Kinect Fusion AlignPointClouds camera tracking failed "
				"in transform magnitude check!")<<endl;
		}
	}

	else
	{
		hr = tracking;
	}
	/*
	cout << "align point cloud have a unkonw procees so we have world to camera trans: " <<
		calculatedCameraPose.M11 << "      " << calculatedCameraPose.M12 << "      " << calculatedCameraPose.M13 << "      " << calculatedCameraPose.M14 << "      " <<
		calculatedCameraPose.M21 << "      " << calculatedCameraPose.M22 << "      " << calculatedCameraPose.M23 << "      " << calculatedCameraPose.M24 << "      " <<
		calculatedCameraPose.M31 << "      " << calculatedCameraPose.M32 << "      " << calculatedCameraPose.M33 << "      " << calculatedCameraPose.M34 << "      " <<
		calculatedCameraPose.M41 << "      " << calculatedCameraPose.M42 << "      " << calculatedCameraPose.M43 << "      " << calculatedCameraPose.M44 << "      " <<
		endl;
		*/
	return hr;
}

/// <summary>
/// Perform camera tracking using AlignDepthFloatToReconstruction
/// </summary>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT KinectFusionProcessor::TrackCameraAlignDepthFloatToReconstruction(Matrix4 &calculatedCameraPose, FLOAT &alignmentEnergy)
{
	HRESULT hr = S_OK;

	// Only calculate the residual delta from reference frame every m_cDeltaFromReferenceFrameCalculationInterval
	// frames to reduce computation time
	HRESULT tracking = S_OK;

	if (m_bCalculateDeltaFrame)
	{
		tracking = m_pVolume->AlignDepthFloatToReconstruction(
			m_pDepthFloatImage,
			NUI_FUSION_DEFAULT_ALIGN_ITERATION_COUNT,
			m_pFloatDeltaFromReference,
			&alignmentEnergy,
			&calculatedCameraPose);
	}
	else
	{
		tracking = m_pVolume->AlignDepthFloatToReconstruction(
			m_pDepthFloatImage,
			NUI_FUSION_DEFAULT_ALIGN_ITERATION_COUNT,
			nullptr,
			&alignmentEnergy,
			&calculatedCameraPose);
	}
	/*
	cout << "align depth float so we have world to camera trans: " <<
		calculatedCameraPose.M11 << "      " << calculatedCameraPose.M12 << "      " << calculatedCameraPose.M13 << "      " << calculatedCameraPose.M14 << "      " <<
		calculatedCameraPose.M21 << "      " << calculatedCameraPose.M22 << "      " << calculatedCameraPose.M23 << "      " << calculatedCameraPose.M24 << "      " <<
		calculatedCameraPose.M31 << "      " << calculatedCameraPose.M32 << "      " << calculatedCameraPose.M33 << "      " << calculatedCameraPose.M34 << "      " <<
		calculatedCameraPose.M41 << "      " << calculatedCameraPose.M42 << "      " << calculatedCameraPose.M43 << "      " << calculatedCameraPose.M44 << "      " <<
		endl;

		*/
	bool trackingSuccess = !(FAILED(tracking) || alignmentEnergy > m_fMaxAlignToReconstructionEnergyForSuccess || (alignmentEnergy == 0.0f && m_cSuccessfulFrameCounter > 1));

	if (trackingSuccess)
	{
		// Get the camera pose
		m_pVolume->GetCurrentWorldToCameraTransform(&calculatedCameraPose);
	}
	else
	{
		if (FAILED(tracking))
		{
			hr = tracking;
		}
		else
		{
			// We failed in the energy check
			hr = E_NUI_FUSION_TRACKING_ERROR;
		}
	}

	/*
	cout << "align depth float then they cal it so we have world to camera trans: " <<
		calculatedCameraPose.M11 << "      " << calculatedCameraPose.M12 << "      " << calculatedCameraPose.M13 << "      " << calculatedCameraPose.M14 << "      " <<
		calculatedCameraPose.M21 << "      " << calculatedCameraPose.M22 << "      " << calculatedCameraPose.M23 << "      " << calculatedCameraPose.M24 << "      " <<
		calculatedCameraPose.M31 << "      " << calculatedCameraPose.M32 << "      " << calculatedCameraPose.M33 << "      " << calculatedCameraPose.M34 << "      " <<
		calculatedCameraPose.M41 << "      " << calculatedCameraPose.M42 << "      " << calculatedCameraPose.M43 << "      " << calculatedCameraPose.M44 << "      " <<
		endl;
		*/
	return hr;
}


///<summary>
/// Handle new depth data and perform Kinect Fusion processing
/// </summary>
void KinectFusionProcessor::ProcessDepth()
{
	HRESULT hr = S_OK;
	bool depthAvailable = false;
	bool raycastFrame = false;
	bool cameraPoseFinderAvailable = IsCameraPoseFinderAvailable();
	bool integrateColor =m_bCaptureColor && m_cFrameCounter % m_cColorIntegrationInterval == 0;  //每三次显示一次颜色
	bool colorSynchronized = false;
	FLOAT alignmentEnergy = 1.0f;
	Matrix4 calculatedCameraPose = m_worldToCameraTransform;
	m_bCalculateDeltaFrame = (m_cFrameCounter % m_cDeltaFromReferenceFrameCalculationInterval == 0)
		|| (m_bTrackingHasFailedPreviously && m_cSuccessfulFrameCounter <= 2);

	// Get the next frames from Kinect
	hr = GetKinectFrames(colorSynchronized);

	if (FAILED(hr))
	{
		goto FinishFrame;
	}

	// Only integrate when color is synchronized with depth
	integrateColor = integrateColor && colorSynchronized;

	////////////////////////////////////////////////////////
	// Depth to Depth Float

	// Convert the pixels describing extended depth as unsigned short type in millimeters to depth
	// as floating point type in meters.
	if (nullptr == m_pVolume)
	{
		hr = NuiFusionDepthToDepthFloatFrame(
			m_pDepthImagePixelBuffer,
			640,
			480,
			m_pDepthFloatImage,
			m_fMinDepthThreshold,
			m_fMaxDepthThreshold,
			m_bMirrorDepthFrame);
	}
	else
	{
		hr = m_pVolume->DepthToDepthFloatFrame(
			m_pDepthImagePixelBuffer,
			640*480 * sizeof(NUI_DEPTH_IMAGE_PIXEL),
			m_pDepthFloatImage,
			m_fMinDepthThreshold,
			m_fMaxDepthThreshold,
			m_bMirrorDepthFrame);
	}

	if (FAILED(hr))
	{
		cout << ("Kinect Fusion NuiFusionDepthToDepthFloatFrame call failed.")<<endl;
		goto FinishFrame;
	}

	depthAvailable = true;

	// Return if the volume is not initialized, just drawing the depth image
	if (nullptr == m_pVolume)
	{
		cout << (
			"Kinect Fusion reconstruction volume not initialized. "
			"Please try reducing volume size or restarting.")<<endl;
		goto FinishFrame;
	}

	////////////////////////////////////////////////////////
	// Perform Camera Tracking

	HRESULT tracking = E_NUI_FUSION_TRACKING_ERROR;

	if (!m_bTrackingFailed && 0 != m_cFrameCounter)
	{
		// Here we can either call or TrackCameraAlignDepthFloatToReconstruction or TrackCameraAlignPointClouds
		// The TrackCameraAlignPointClouds function typically has higher performance with the camera pose finder 
		// due to its wider basin of convergence, enabling it to more robustly regain tracking from nearby poses
		// suggested by the camera pose finder after tracking is lost.
		if (m_bAutoFindCameraPoseWhenLost)
		{
			tracking = TrackCameraAlignPointClouds(calculatedCameraPose, alignmentEnergy);     //  配准点云 
		}
		else
		{
			// If the camera pose finder is not turned on, we use AlignDepthFloatToReconstruction
			tracking = TrackCameraAlignDepthFloatToReconstruction(calculatedCameraPose, alignmentEnergy);    //yeah 
		}
	}

	if (FAILED(tracking) && 0 != m_cFrameCounter)   // frame 0 always succeeds
	{
		SetTrackingFailed();

		if (!cameraPoseFinderAvailable)
		{
			if (tracking == E_NUI_FUSION_TRACKING_ERROR)
			{
				char str[MAX_PATH];
				sprintf_s(str, "Kinect Fusion camera tracking FAILED! Align the camera to the last tracked position.");
				cout << (str)<<endl;
			}
			else
			{
				cout << ("Kinect Fusion camera tracking call failed!")<<endl;
				goto FinishFrame;
			}
		}
		else
		{
			// Here we try to find the correct camera pose, to re-localize camera tracking.
			// We can call either the version using AlignDepthFloatToReconstruction or the version 
			// using AlignPointClouds, which typically has a higher success rate with the camera pose finder.
			//tracking = FindCameraPoseAlignDepthFloatToReconstruction();
			tracking = FindCameraPoseAlignPointClouds();

			if (FAILED(tracking) && tracking != E_NUI_FUSION_TRACKING_ERROR)
			{
				cout << ("Kinect Fusion FindCameraPose call failed.")<<endl;
				goto FinishFrame;
			}
		}
	}
	else
	{
		if (m_bTrackingHasFailedPreviously)
		{
			char str[MAX_PATH];
			if (!m_bAutoFindCameraPoseWhenLost)
			{
				sprintf_s(str, "Kinect Fusion camera tracking RECOVERED! Residual energy=%f", alignmentEnergy);
			}
			else
			{
				sprintf_s(str, "Kinect Fusion camera tracking RECOVERED!");
			}
			cout << (str)<<endl;
		}

		m_worldToCameraTransform = calculatedCameraPose;
		SetTrackingSucceeded();              //yeah 
	}

	if (m_bAutoResetReconstructionWhenLost &&
		m_bTrackingFailed &&
		m_cLostFrameCounter >= cResetOnNumberOfLostFrames)
	{
		// Automatically Clear Volume and reset tracking if tracking fails
		hr = InternalResetReconstruction();

		if (SUCCEEDED(hr))
		{
			// Set bad tracking message
			cout << (
				"Kinect Fusion camera tracking failed, "
				"automatically reset volume.")<<endl;
		}
		else
		{
			cout << ("Kinect Fusion Reset Reconstruction call failed.")<<endl;
			goto FinishFrame;
		}
	}

	////////////////////////////////////////////////////////
	// Integrate Depth Data into volume

	// Don't integrate depth data into the volume if:
	// 1) tracking failed
	// 2) camera pose finder is off and we have paused capture
	// 3) camera pose finder is on and we are still under the m_cMinSuccessfulTrackingFramesForCameraPoseFinderAfterFailure
	//    number of successful frames count.
	bool integrateData = !m_bTrackingFailed && ((!cameraPoseFinderAvailable && !m_bPauseIntegration)
		|| (cameraPoseFinderAvailable && !(m_bTrackingHasFailedPreviously && m_cSuccessfulFrameCounter < m_cMinSuccessfulTrackingFramesForCameraPoseFinderAfterFailure)));

	if (integrateData)
	{
		if (cameraPoseFinderAvailable)
		{
			// If integration resumed, this will un-check the pause integration check box back on in the UI automatically
			m_bIntegrationResumed = true;
		}

		// Reset this flag as we are now integrating data again
		m_bTrackingHasFailedPreviously = false;

		if (integrateColor)
		{
			// Map the color frame to the depth - this fills m_pResampledColorImageDepthAligned
			MapColorToDepth();

			// Integrate the depth and color data into the volume from the calculated camera pose
			hr = m_pVolume->IntegrateFrame(
				m_pDepthFloatImage,
				m_pResampledColorImageDepthAligned,
				m_cMaxIntegrationWeight,
				NUI_FUSION_DEFAULT_COLOR_INTEGRATION_OF_ALL_ANGLES,
				&m_worldToCameraTransform);

			m_bCaptureColor = true;
		}
		else
		{
			// Integrate just the depth data into the volume from the calculated camera pose
			hr = m_pVolume->IntegrateFrame(
				m_pDepthFloatImage,
				nullptr,
				m_cMaxIntegrationWeight,
				NUI_FUSION_DEFAULT_COLOR_INTEGRATION_OF_ALL_ANGLES,
				&m_worldToCameraTransform);
		}

		if (FAILED(hr))
		{
			cout << ("Kinect Fusion IntegrateFrame call failed.")<<endl;
			goto FinishFrame;
		}
		/*
		cout << "intergate depth image to point cloud so we have world to camera trans: " <<
			m_worldToCameraTransform.M11 << "      " << m_worldToCameraTransform.M12 << "      " << m_worldToCameraTransform.M13 << "      " << m_worldToCameraTransform.M14 << "      " <<
			m_worldToCameraTransform.M21 << "      " << m_worldToCameraTransform.M22 << "      " << m_worldToCameraTransform.M23 << "      " << m_worldToCameraTransform.M24 << "      " <<
			m_worldToCameraTransform.M31 << "      " << m_worldToCameraTransform.M32 << "      " << m_worldToCameraTransform.M33 << "      " << m_worldToCameraTransform.M34 << "      " <<
			m_worldToCameraTransform.M41 << "      " << m_worldToCameraTransform.M42 << "      " << m_worldToCameraTransform.M43 << "      " << m_worldToCameraTransform.M44 << "      " <<
			endl;
			*/
	}

	////////////////////////////////////////////////////////
	// Check to see if we have time to raycast

	{
		double currentTime = m_timer.AbsoluteTime();

		// Is another frame already waiting?
		if (WaitForSingleObject(m_hNextDepthFrameEvent, 0) == WAIT_TIMEOUT)
		{
			// No: We should have enough time to raycast.
			raycastFrame = true;
		}
		else
		{
			// Yes: Raycast only if we've exceeded the render interval.
			double renderIntervalSeconds = (0.001 * cRenderIntervalMilliseconds);
			raycastFrame = (currentTime - m_fMostRecentRaycastTime > renderIntervalSeconds);
		}

		if (raycastFrame)
		{
			m_fMostRecentRaycastTime = currentTime;
		}
	}

	if (raycastFrame)
	{
		////////////////////////////////////////////////////////
		// CalculatePointCloud

		// Raycast even if camera tracking failed, to enable us to visualize what is 
		// happening with the system
		hr = m_pVolume->CalculatePointCloud(
			m_pRaycastPointCloud,
			(m_bCaptureColor ? m_pCapturedSurfaceColor : nullptr),
			&m_worldToCameraTransform);

		/*cout << "calculate Point cloud so we have world to camera trans: " <<
			m_worldToCameraTransform.M11 << "      " << m_worldToCameraTransform.M12 << "      " << m_worldToCameraTransform.M13 << "      " << m_worldToCameraTransform.M14 << "      " <<
			m_worldToCameraTransform.M21 << "      " << m_worldToCameraTransform.M22 << "      " << m_worldToCameraTransform.M23 << "      " << m_worldToCameraTransform.M24 << "      " <<
			m_worldToCameraTransform.M31 << "      " << m_worldToCameraTransform.M32 << "      " << m_worldToCameraTransform.M33 << "      " << m_worldToCameraTransform.M34 << "      " <<
			m_worldToCameraTransform.M41 << "      " << m_worldToCameraTransform.M42 << "      " << m_worldToCameraTransform.M43 << "      " << m_worldToCameraTransform.M44 << "      " <<
			endl;*/

		if (FAILED(hr))
		{
			cout << ("Kinect Fusion CalculatePointCloud call failed.")<<endl;
			goto FinishFrame;
		}

		////////////////////////////////////////////////////////
		// ShadePointCloud

		if (!m_bCaptureColor)
		{
			hr = NuiFusionShadePointCloud(
				m_pRaycastPointCloud,
				&m_worldToCameraTransform,
				&m_worldToBGRTransform,
				m_pShadedSurface,
				m_bDisplaySurfaceNormals ? m_pShadedSurfaceNormals : nullptr);

			if (FAILED(hr))
			{
				cout << ("Kinect Fusion NuiFusionShadePointCloud call failed.")<<endl;
				goto FinishFrame;
			}
		}
	}

	////////////////////////////////////////////////////////
	// Update camera pose finder, adding key frames to the database

	if (m_bAutoFindCameraPoseWhenLost && !m_bTrackingHasFailedPreviously
		&& m_cSuccessfulFrameCounter > m_cMinSuccessfulTrackingFramesForCameraPoseFinder
		&& m_cFrameCounter % m_cCameraPoseFinderProcessFrameCalculationInterval == 0
		&& colorSynchronized)
	{
		hr = UpdateCameraPoseFinder();       //there 

		if (FAILED(hr))
		{
			cout << ("Kinect Fusion UpdateCameraPoseFinder call failed.")<<endl;
			goto FinishFrame;
		}
	}

FinishFrame:


	////////////////////////////////////////////////////////
	// Copy the images to their frame buffers
	//m_pResampledColorImageDepthAligned
	//INuiFrameTexture *imageFrameTexture = m_pColorImage->pFrameTexture;
	INuiFrameTexture *imageFrameTexture = m_pResampledColorImageDepthAligned->pFrameTexture;
	NUI_LOCKED_RECT LockedRect;

	// Lock the frame data so the Kinect knows not to modify it while we're reading it
	imageFrameTexture->LockRect(0, &LockedRect, nullptr, 0);

	// Make sure we've received valid data
	if (LockedRect.Pitch != 0)
	{
		m_imgColor.data = LockedRect.pBits;
	}

	if (depthAvailable)
	{
		//StoreImageToFrameBuffer(m_pDepthFloatImage, m_frame.m_pDepthRGBX);
		INuiFrameTexture *imageFrameTexture = m_pDepthFloatImage->pFrameTexture;
		NUI_LOCKED_RECT LockedRect;

		// Lock the frame data so the Kinect knows not to modify it while we're reading it
		imageFrameTexture->LockRect(0, &LockedRect, nullptr, 0);

		// Make sure we've received valid data
		if (LockedRect.Pitch != 0)
		{
			m_imgDepth.data = LockedRect.pBits;
		}
	}

	if (raycastFrame)
	{
		if (m_bCaptureColor)
		{
			//StoreImageToFrameBuffer(m_pCapturedSurfaceColor, m_frame.m_pReconstructionRGBX);
			INuiFrameTexture *imageFrameTexture = m_pCapturedSurfaceColor->pFrameTexture;
			NUI_LOCKED_RECT LockedRect;

			// Lock the frame data so the Kinect knows not to modify it while we're reading it
			imageFrameTexture->LockRect(0, &LockedRect, nullptr, 0);

			// Make sure we've received valid data
			if (LockedRect.Pitch != 0)
			{
				cloudShade.data = LockedRect.pBits;
			}
		}
		else if (m_bDisplaySurfaceNormals)
		{
			//StoreImageToFrameBuffer(m_pShadedSurfaceNormals, m_frame.m_pReconstructionRGBX);
		}
		else
		{
			//StoreImageToFrameBuffer(m_pShadedSurface, m_frame.m_pReconstructionRGBX);         //带用户色彩点云投射
			INuiFrameTexture *imageFrameTexture = m_pShadedSurface->pFrameTexture;
			NUI_LOCKED_RECT LockedRect;

			// Lock the frame data so the Kinect knows not to modify it while we're reading it
			imageFrameTexture->LockRect(0, &LockedRect, nullptr, 0);

			// Make sure we've received valid data
			if (LockedRect.Pitch != 0)
			{
				cloudRGB.data = LockedRect.pBits;
			}
		}
	}

	// Display raycast depth image when in pose finding mode
	if (m_bTrackingFailed && cameraPoseFinderAvailable)
	{
		// StoreImageToFrameBuffer(m_pRaycastDepthFloatImage, m_frame.m_pTrackingDataRGBX);
		INuiFrameTexture *imageFrameTexture = m_pRaycastDepthFloatImage->pFrameTexture;
		NUI_LOCKED_RECT LockedRect;

		// Lock the frame data so the Kinect knows not to modify it while we're reading it
		imageFrameTexture->LockRect(0, &LockedRect, nullptr, 0);

		// Make sure we've received valid data
		if (LockedRect.Pitch != 0)
		{
			m_imgDepth.data = LockedRect.pBits;
		}

	}
	else
	{
		// Don't calculate the residual delta from reference frame every frame to reduce computation time
		if (m_bCalculateDeltaFrame)
		{
			if (!m_bAutoFindCameraPoseWhenLost)
			{
				// Color the float residuals from the AlignDepthFloatToReconstruction
				hr = ColorResiduals(m_pFloatDeltaFromReference, m_pShadedDeltaFromReference);
			}

			if (SUCCEEDED(hr))
			{
				// StoreImageToFrameBuffer(m_pShadedDeltaFromReference, m_frame.m_pTrackingDataRGBX);

				INuiFrameTexture *imageFrameTexture = m_pShadedDeltaFromReference->pFrameTexture;
				NUI_LOCKED_RECT LockedRect;

				// Lock the frame data so the Kinect knows not to modify it while we're reading it
				imageFrameTexture->LockRect(0, &LockedRect, nullptr, 0);

				// Make sure we've received valid data
				if (LockedRect.Pitch != 0)
				{
					dert.data = LockedRect.pBits;
				}

			}
		}
	}

	////////////////////////////////////////////////////////
	// Periodically Display Fps

	if (SUCCEEDED(hr))
	{
		// Update frame counter
		m_cFrameCounter++;

		// Display fps count approximately every cTimeDisplayInterval seconds
		double elapsed = m_timer.AbsoluteTime() - m_fFrameCounterStartTime;
		if (static_cast<int>(elapsed) >= cTimeDisplayInterval)
		{
			m_fFramesPerSecond = 0;

			// Update status display
			if (!m_bTrackingFailed)
			{
				m_fFramesPerSecond = static_cast<float>(m_cFrameCounter / elapsed);
			}

			m_cFrameCounter = 0;
			m_fFrameCounterStartTime = m_timer.AbsoluteTime();
			cout << "The Frame per second= " << m_fFramesPerSecond << endl;
		}
	}

	//pCapture >> stereoImg;   //双目图像获取

	//cout << "we gonna save this two pic~" << endl;
}


/// <summary>
/// Perform camera pose finding when tracking is lost using AlignPointClouds.
/// This is typically more successful than FindCameraPoseAlignDepthFloatToReconstruction.
/// </summary>
HRESULT KinectFusionProcessor::FindCameraPoseAlignPointClouds()
{
	HRESULT hr = S_OK;

	if (!IsCameraPoseFinderAvailable())
	{
		return E_FAIL;
	}

	bool resampled = false;

	hr = ProcessColorForCameraPoseFinder(resampled);

	if (FAILED(hr))
	{
		return hr;
	}

	// Start  kNN (k nearest neighbors) camera pose finding
	INuiFusionMatchCandidates *pMatchCandidates = nullptr;

	// Test the camera pose finder to see how similar the input images are to previously captured images.
	// This will return an error code if there are no matched frames in the camera pose finder database.
	hr = m_pCameraPoseFinder->FindCameraPose(
		m_pDepthFloatImage,
		resampled ? m_pResampledColorImage : m_pColorImage,
		&pMatchCandidates);

	if (FAILED(hr) || nullptr == pMatchCandidates)
	{
		goto FinishFrame;
	}

	unsigned int cPoses = pMatchCandidates->MatchPoseCount();

	float minDistance = 1.0f;   // initialize to the maximum normalized distance
	hr = pMatchCandidates->CalculateMinimumDistance(&minDistance);

	if (FAILED(hr) || 0 == cPoses)
	{
		goto FinishFrame;
	}

	// Check the closest frame is similar enough to our database to re-localize
	// For frames that have a larger minimum distance, standard tracking will run
	// and if this fails, tracking will be considered lost.
	if (minDistance >= m_fCameraPoseFinderDistanceThresholdReject)
	{
		cout << ("FindCameraPose exited early as not good enough pose matches.")<<endl;
		hr = E_NUI_NO_MATCH;
		goto FinishFrame;
	}

	// Get the actual matched poses
	const Matrix4 *pNeighbors = nullptr;
	hr = pMatchCandidates->GetMatchPoses(&pNeighbors);

	if (FAILED(hr))
	{
		goto FinishFrame;
	}

	////////////////////////////////////////////////////////
	// Smooth depth image

	hr = m_pVolume->SmoothDepthFloatFrame(
		m_pDepthFloatImage,
		m_pSmoothDepthFloatImage,
		m_cSmoothingKernelWidth,
		m_fSmoothingDistanceThreshold); // ON GPU

	if (FAILED(hr))
	{
		cout << ("Kinect Fusion SmoothDepth call failed.")<<endl;
		goto FinishFrame;
	}

	////////////////////////////////////////////////////////
	// Calculate Point Cloud from smoothed input Depth Image

	hr = NuiFusionDepthFloatFrameToPointCloud(
		m_pSmoothDepthFloatImage,
		m_pDepthPointCloud);

	if (FAILED(hr))
	{
		cout << ("Kinect Fusion NuiFusionDepthFloatFrameToPointCloud call failed.")<<endl;
		goto FinishFrame;
	}

	HRESULT tracking = S_OK;
	FLOAT alignmentEnergy = 0;

	unsigned short relocIterationCount = NUI_FUSION_DEFAULT_ALIGN_ITERATION_COUNT;

	double smallestEnergy = DBL_MAX;
	int smallestEnergyNeighborIndex = -1;

	int bestNeighborIndex = -1;
	Matrix4 bestNeighborCameraPose;
	SetIdentityMatrix(bestNeighborCameraPose);
	// Exclude very tiny alignment energy case which is unlikely to happen in reality - this is more likely a tracking error
	double bestNeighborAlignmentEnergy = m_fMaxAlignPointCloudsEnergyForSuccess;

	// Run alignment with best matched poses (i.e. k nearest neighbors (kNN))
	unsigned int maxTests = min(m_cMaxCameraPoseFinderPoseTests, cPoses);

	for (unsigned int n = 0; n < maxTests; n++)
	{
		////////////////////////////////////////////////////////
		// Call AlignPointClouds

		Matrix4 poseProposal = pNeighbors[n];

		// Get the saved pose view by raycasting the volume
		hr = m_pVolume->CalculatePointCloud(m_pRaycastPointCloud, nullptr, &poseProposal);

		tracking = m_pVolume->AlignPointClouds(
			m_pRaycastPointCloud,
			m_pDepthPointCloud,
			relocIterationCount,
			nullptr,
			&alignmentEnergy,
			&poseProposal);


		if (SUCCEEDED(tracking) && alignmentEnergy < bestNeighborAlignmentEnergy  && alignmentEnergy > m_fMinAlignPointCloudsEnergyForSuccess)
		{
			bestNeighborAlignmentEnergy = alignmentEnergy;
			bestNeighborIndex = n;

			// This is after tracking succeeds, so should be a more accurate pose to store...
			bestNeighborCameraPose = poseProposal;
		}

		// Find smallest energy neighbor independent of tracking success
		if (alignmentEnergy < smallestEnergy)
		{
			smallestEnergy = alignmentEnergy;
			smallestEnergyNeighborIndex = n;
		}
	}

	// Use the neighbor with the smallest residual alignment energy
	// At the cost of additional processing we could also use kNN+Mean camera pose finding here
	// by calculating the mean pose of the best n matched poses and also testing this to see if the 
	// residual alignment energy is less than with kNN.
	if (bestNeighborIndex > -1)
	{
		m_worldToCameraTransform = bestNeighborCameraPose;

		// Get the saved pose view by raycasting the volume
		hr = m_pVolume->CalculatePointCloud(m_pRaycastPointCloud, nullptr, &m_worldToCameraTransform);

		if (FAILED(hr))
		{
			goto FinishFrame;
		}

		// Tracking succeeded!
		hr = S_OK;

		SetTrackingSucceeded();

		// Run a single iteration of AlignPointClouds to get the deltas frame
		hr = m_pVolume->AlignPointClouds(
			m_pRaycastPointCloud,
			m_pDepthPointCloud,
			1,
			m_pShadedDeltaFromReference,
			&alignmentEnergy,
			&bestNeighborCameraPose);

		if (SUCCEEDED(hr))
		{
			//StoreImageToFrameBuffer(m_pShadedDeltaFromReference, m_frame.m_pTrackingDataRGBX);
			INuiFrameTexture *imageFrameTexture = m_pShadedDeltaFromReference->pFrameTexture;
			NUI_LOCKED_RECT LockedRect;

			// Lock the frame data so the Kinect knows not to modify it while we're reading it
			imageFrameTexture->LockRect(0, &LockedRect, nullptr, 0);

			// Make sure we've received valid data
			if (LockedRect.Pitch != 0)
			{
				dert.data = LockedRect.pBits;
			}
		}

		// Stop the residual image being displayed as we have stored our own
		m_bCalculateDeltaFrame = false;

		char str[MAX_PATH];
		sprintf_s(str,  "Camera Pose Finder SUCCESS! Residual energy=%f, %d frames stored, minimum distance=%f, best match index=%d", bestNeighborAlignmentEnergy, cPoses, minDistance, bestNeighborIndex);
		cout << (str);
	}
	else
	{
		m_worldToCameraTransform = pNeighbors[smallestEnergyNeighborIndex];

		// Get the smallest energy view by raycasting the volume
		hr = m_pVolume->CalculatePointCloud(m_pRaycastPointCloud, nullptr, &m_worldToCameraTransform);

		if (FAILED(hr))
		{
			goto FinishFrame;
		}

		// Camera pose finding failed - return the tracking failed error code
		hr = E_NUI_FUSION_TRACKING_ERROR;

		// Tracking Failed will be set again on the next iteration in ProcessDepth
		char str[MAX_PATH];
		sprintf_s(str, "Camera Pose Finder FAILED! Residual energy=%f, %d frames stored, minimum distance=%f, best match index=%d", smallestEnergy, cPoses, minDistance, smallestEnergyNeighborIndex);
		cout << (str);
	}

FinishFrame:

	SafeRelease(pMatchCandidates);

	return hr;
}


/// <summary>
/// Perform camera pose finding when tracking is lost using AlignPointClouds.
/// This is typically more successful than FindCameraPoseAlignDepthFloatToReconstruction.
/// </summary>
int  KinectFusionProcessor::gengeratePointsB(NUI_FUSION_IMAGE_FRAME*     m_pPointCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz)
{
	INuiFrameTexture * pCloudeImageTexture = m_pPointCloud->pFrameTexture;
	NUI_LOCKED_RECT CloudLockedRect;

	// Lock the frame data so the Kinect knows not to modify it while we're reading it
	HRESULT hr = pCloudeImageTexture->LockRect(0, &CloudLockedRect, nullptr, 0);

	if (CloudLockedRect.Pitch != 0)
	{
		cloud_xyz->clear();

		pcl::PointXYZ point;
		for (int i = 0; i < 480; i = i + 1)
		{
			float* pBuffer = (float*)CloudLockedRect.pBits + i * CloudLockedRect.Pitch / sizeof(float);
			for (int j = 0; j < 640; j = j + 1)
			{
				if (0.6 < (float)pBuffer[6 * j + 2] && (float)pBuffer[6 * j + 2] < 3.6)
				{
					point.x = (float)pBuffer[6 * j];
					point.y = (float)pBuffer[6 * j + 1];
					point.z = (float)pBuffer[6 * j + 2];
					//point.normal_x = (float)pBuffer[6 * j + 3];
					//point.normal_y = (float)pBuffer[6 * j + 4];
					//point.normal_z = (float)pBuffer[6 * j + 5];


					cloud_xyz->points.push_back(point);
				}
			}
		}
	}

	else
	{
		cout << "copy data to point_cloud_ptr failed!" << endl;
		return 0;
	}

	pCloudeImageTexture->UnlockRect(0);

	cloud_xyz->width = cloud_xyz->size();
	cloud_xyz->height = 1;

	return 1;
}

int  KinectFusionProcessor::gengeratePointsNB(NUI_FUSION_IMAGE_FRAME*     m_pPointCloud, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_xyzcnormals)
{
	INuiFrameTexture * pCloudeImageTexture = m_pPointCloud->pFrameTexture;
	NUI_LOCKED_RECT CloudLockedRect;

	// Lock the frame data so the Kinect knows not to modify it while we're reading it
	HRESULT hr = pCloudeImageTexture->LockRect(0, &CloudLockedRect, nullptr, 0);

	if (CloudLockedRect.Pitch != 0)
	{
		cloud_xyzcnormals->clear();

		pcl::PointNormal point;
		for (int i = 0; i < 480; i = i + 1)
		{
			float* pBuffer = (float*)CloudLockedRect.pBits + i * CloudLockedRect.Pitch / sizeof(float);
			for (int j = 0; j < 640; j = j + 1)
			{
				if (0.6 < (float)pBuffer[6 * j + 2] && (float)pBuffer[6 * j + 2] < 8)
				{
					point.x = (float)pBuffer[6 * j];
					point.y = (float)pBuffer[6 * j + 1];
					point.z = (float)pBuffer[6 * j + 2];
					point.normal_x = (float)pBuffer[6 * j + 3];
					point.normal_y = (float)pBuffer[6 * j + 4];
					point.normal_z = (float)pBuffer[6 * j + 5];


					cloud_xyzcnormals->points.push_back(point);
				}
			}
		}
	}

	else
	{
		cout << "copy data to point_cloud_ptr failed!" << endl;
		return 0;
	}

	pCloudeImageTexture->UnlockRect(0);

	cloud_xyzcnormals->width = cloud_xyzcnormals->size();
	cloud_xyzcnormals->height = 1;

	//pclVP.saveBasicWithNormCloud();

	return 1;
}

int KinectFusionProcessor::gengeratePointsNC(NUI_FUSION_IMAGE_FRAME*     m_pPointCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb)
{
	INuiFrameTexture * pCloudeImageTexture = m_pPointCloud->pFrameTexture;
	//m_pPointCloud->pFrameTexture->
	NUI_LOCKED_RECT CloudLockedRect;
	
	INuiFrameTexture * pCloudeImageTextureD = m_pResampledColorImageDepthAligned->pFrameTexture;
	NUI_LOCKED_RECT CloudLockedRectD;
	// Lock the frame data so the Kinect knows not to modify it while we're reading it
	HRESULT hr = pCloudeImageTexture->LockRect(0, &CloudLockedRect, nullptr, 0);

	hr = pCloudeImageTextureD->LockRect(0, &CloudLockedRectD, nullptr, 0);

	if (CloudLockedRect.Pitch != 0)
	{
		cloud_xyzrgb->clear();

		pcl::PointXYZRGB point;
		for (int i = 0; i < 480; i = i + 1)
		{
			float* pBuffer = (float*)CloudLockedRect.pBits + i * CloudLockedRect.Pitch / sizeof(float);
			uchar * rgbrun = (uchar*)CloudLockedRectD.pBits + i * CloudLockedRectD.Pitch;
			//uchar * rgbrun = (uchar*)cloudShade.data + i*cloudShade.cols;
			for (int j = 0; j < 640; j = j + 1)
			{
				if (0.6 < (float)pBuffer[6 * j + 2] && (float)pBuffer[6 * j + 2] < 8.0)
				{
					point.x = (float)pBuffer[6 * j];
					point.y = (float)pBuffer[6 * j + 1];
					point.z = (float)pBuffer[6 * j + 2];
					//point.normal_x = (float)pBuffer[6 * j + 3];
					//point.normal_y = (float)pBuffer[6 * j + 4];
					//point.normal_z = (float)pBuffer[6 * j + 5];
					point.b = rgbrun[4 * j];
					point.g = rgbrun[4 * j + 1];
					point.r = rgbrun[4* j+ 2];

					cloud_xyzrgb->points.push_back(point);
				}
			}
		}
	}

	else
	{
		cout << "copy data to point_cloud_ptr failed!" << endl;
		return 0;
	}

	pCloudeImageTexture->UnlockRect(0);
	pCloudeImageTextureD->UnlockRect(0);
	cloud_xyzrgb->width = cloud_xyzrgb->size();
	cloud_xyzrgb->height = 1;

	return 1;
}

/// <summary>
/// Perform camera pose finding when tracking is lost using AlignPointClouds.
/// This is typically more successful than FindCameraPoseAlignDepthFloatToReconstruction.
/// </summary>
int  KinectFusionProcessor::writeCameraPose()
{
	ofsCount++;
	char path[50];
	sprintf(path, "data/CameraPose%d.yml", ofsCount);
	fs.open(path, CV_STORAGE_WRITE);
	Mat camerPose(4, 4, CV_32FC1);
	camerPose.at<float>(0, 0) = float(m_worldToCameraTransform.M11);
	camerPose.at<float>(0, 1) = float(m_worldToCameraTransform.M12);
	camerPose.at<float>(0, 2) = float(m_worldToCameraTransform.M13);
	camerPose.at<float>(0, 3) = float(m_worldToCameraTransform.M14);
	camerPose.at<float>(1, 0) = float(m_worldToCameraTransform.M21);
	camerPose.at<float>(1, 1) = float(m_worldToCameraTransform.M22);
	camerPose.at<float>(1, 2) = float(m_worldToCameraTransform.M23);
	camerPose.at<float>(1, 3) = float(m_worldToCameraTransform.M24);
	camerPose.at<float>(2, 0) = float(m_worldToCameraTransform.M31);
	camerPose.at<float>(2, 1) = float(m_worldToCameraTransform.M32);
	camerPose.at<float>(2, 2) = float(m_worldToCameraTransform.M33);
	camerPose.at<float>(2, 3) = float(m_worldToCameraTransform.M34);
	camerPose.at<float>(3, 0) = float(m_worldToCameraTransform.M41);
	camerPose.at<float>(3, 1) = float(m_worldToCameraTransform.M42);
	camerPose.at<float>(3, 2) = float(m_worldToCameraTransform.M43);
	camerPose.at<float>(3, 3) = float(m_worldToCameraTransform.M44);

	if (fs.isOpened())
	{
		fs << "World2Camera" << camerPose;
		fs.release();
		cout << "we successfully saved this cameraPose!" << endl;
	}
	else
		cout << "something wrong with saving cameraPose!" << endl;
	return 1;
}

/// <summary>
/// Perform camera pose finding when tracking is lost using AlignDepthFloatToReconstruction.
/// </summary>
HRESULT KinectFusionProcessor::FindCameraPoseAlignDepthFloatToReconstruction()
{
	HRESULT hr = S_OK;

	if (!IsCameraPoseFinderAvailable())
	{
		return E_FAIL;
	}

	bool resampled = false;

	hr = ProcessColorForCameraPoseFinder(resampled);

	if (FAILED(hr))
	{
		return hr;
	}

	// Start  kNN (k nearest neighbors) camera pose finding
	INuiFusionMatchCandidates *pMatchCandidates = nullptr;

	// Test the camera pose finder to see how similar the input images are to previously captured images.
	// This will return an error code if there are no matched frames in the camera pose finder database.
	hr = m_pCameraPoseFinder->FindCameraPose(
		m_pDepthFloatImage,
		resampled ? m_pResampledColorImage : m_pColorImage,
		&pMatchCandidates);

	if (FAILED(hr) || nullptr == pMatchCandidates)
	{
		goto FinishFrame;
	}

	unsigned int cPoses = pMatchCandidates->MatchPoseCount();

	float minDistance = 1.0f;   // initialize to the maximum normalized distance
	hr = pMatchCandidates->CalculateMinimumDistance(&minDistance);

	if (FAILED(hr) || 0 == cPoses)
	{
		goto FinishFrame;
	}

	// Check the closest frame is similar enough to our database to re-localize
	// For frames that have a larger minimum distance, standard tracking will run
	// and if this fails, tracking will be considered lost.
	if (minDistance >= m_fCameraPoseFinderDistanceThresholdReject)
	{
		cout << ("FindCameraPose exited early as not good enough pose matches.")<<endl;
		hr = E_NUI_NO_MATCH;
		goto FinishFrame;
	}

	// Get the actual matched poses
	const Matrix4 *pNeighbors = nullptr;
	hr = pMatchCandidates->GetMatchPoses(&pNeighbors);

	if (FAILED(hr))
	{
		goto FinishFrame;
	}

	HRESULT tracking = S_OK;
	FLOAT alignmentEnergy = 0;

	unsigned short relocIterationCount = NUI_FUSION_DEFAULT_ALIGN_ITERATION_COUNT;

	double smallestEnergy = DBL_MAX;
	int smallestEnergyNeighborIndex = -1;

	int bestNeighborIndex = -1;
	Matrix4 bestNeighborCameraPose;
	SetIdentityMatrix(bestNeighborCameraPose);
	// Exclude very tiny alignment energy case which is unlikely to happen in reality - this is more likely a tracking error
	double bestNeighborAlignmentEnergy = m_fMaxAlignToReconstructionEnergyForSuccess;

	// Run alignment with best matched poses (i.e. k nearest neighbors (kNN))
	unsigned int maxTests = min(m_cMaxCameraPoseFinderPoseTests, cPoses);

	for (unsigned int n = 0; n < maxTests; n++)
	{
		////////////////////////////////////////////////////////
		// Call AlignDepthFloatToReconstruction

		// Run the camera tracking algorithm with the volume
		// this uses the raycast frame and pose to find a valid camera pose by matching the depth against the volume
		hr = SetReferenceFrame(pNeighbors[n]);

		if (FAILED(hr))
		{
			continue;
		}

		tracking = m_pVolume->AlignDepthFloatToReconstruction(
			m_pDepthFloatImage,
			relocIterationCount,
			m_pFloatDeltaFromReference,
			&alignmentEnergy,
			&(pNeighbors[n]));

		bool relocSuccess = SUCCEEDED(tracking) && alignmentEnergy < bestNeighborAlignmentEnergy && alignmentEnergy > m_fMinAlignToReconstructionEnergyForSuccess;

		if (relocSuccess)
		{
			if (SUCCEEDED(tracking))
			{
				bestNeighborAlignmentEnergy = alignmentEnergy;
				bestNeighborIndex = n;

				// This is after tracking succeeds, so should be a more accurate pose to store...
				m_pVolume->GetCurrentWorldToCameraTransform(&bestNeighborCameraPose);
			}
		}

		// Find smallest energy neighbor independent of tracking success
		if (alignmentEnergy < smallestEnergy)
		{
			smallestEnergy = alignmentEnergy;
			smallestEnergyNeighborIndex = n;
		}
	}

	// Use the neighbor with the smallest residual alignment energy
	// At the cost of additional processing we could also use kNN+Mean camera pose finding here
	// by calculating the mean pose of the best n matched poses and also testing this to see if the 
	// residual alignment energy is less than with kNN.
	if (bestNeighborIndex > -1)
	{
		m_worldToCameraTransform = bestNeighborCameraPose;
		hr = SetReferenceFrame(m_worldToCameraTransform);

		if (FAILED(hr))
		{
			goto FinishFrame;
		}

		// Tracking succeeded!
		hr = S_OK;

		SetTrackingSucceeded();

		// Force the residual image to be displayed
		m_bCalculateDeltaFrame = true;

		char str[MAX_PATH];
		sprintf_s(str, "Camera Pose Finder SUCCESS! Residual energy=%f, %d frames stored, minimum distance=%f, best match index=%d", bestNeighborAlignmentEnergy, cPoses, minDistance, bestNeighborIndex);
		cout << (str);
	}
	else
	{
		m_worldToCameraTransform = pNeighbors[smallestEnergyNeighborIndex];
		hr = SetReferenceFrame(m_worldToCameraTransform);

		if (FAILED(hr))
		{
			goto FinishFrame;
		}

		// Camera pose finding failed - return the tracking failed error code
		hr = E_NUI_FUSION_TRACKING_ERROR;

		// Tracking Failed will be set again on the next iteration in ProcessDepth
		char str[MAX_PATH];
		sprintf_s(str, "Camera Pose Finder FAILED! Residual energy=%f, %d frames stored, minimum distance=%f, best match index=%d", smallestEnergy, cPoses, minDistance, smallestEnergyNeighborIndex);
		cout << (str);
	}

FinishFrame:

	SafeRelease(pMatchCandidates);

	return hr;
}



/// <summary>
/// Performs raycasting for given pose and sets the tracking reference frame
/// </summary>
/// <param name="worldToCamera">The reference camera pose.</param>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT KinectFusionProcessor::SetReferenceFrame(const Matrix4 &worldToCamera)
{
	HRESULT hr = S_OK;

	// Raycast to get the predicted previous frame to align against in the next frame
	hr = m_pVolume->CalculatePointCloudAndDepth(
		m_pRaycastPointCloud,
		m_pRaycastDepthFloatImage,
		nullptr,
		&worldToCamera);

	if (FAILED(hr))
	{
		cout << ("Kinect Fusion CalculatePointCloud call failed.")<<endl;
		return hr;
	}

	// Set this frame as a reference for AlignDepthFloatToReconstruction
	hr = m_pVolume->SetAlignDepthFloatToReconstructionReferenceFrame(m_pRaycastDepthFloatImage);

	if (FAILED(hr))
	{
		cout << ("Kinect Fusion SetAlignDepthFloatToReconstructionReferenceFrame call failed.")<<endl;
		return hr;
	}

	return hr;
}

/// <summary>
/// Update the status on tracking failure.
/// </summary>
void KinectFusionProcessor::SetTrackingFailed()
{
	m_cLostFrameCounter++;
	m_cSuccessfulFrameCounter = 0;
	m_bTrackingFailed = true;
	m_bTrackingHasFailedPreviously = true;

	m_bIntegrationResumed = false;
}

/// <summary>
/// Update the status when tracking succeeds.
/// </summary>
void KinectFusionProcessor::SetTrackingSucceeded()
{
	m_cLostFrameCounter = 0;
	m_cSuccessfulFrameCounter++;
	m_bTrackingFailed = false;
}

/// <summary>
/// Reset the tracking flags
/// </summary>
void KinectFusionProcessor::ResetTracking()
{
	m_bTrackingFailed = false;
	m_bTrackingHasFailedPreviously = false;

	m_cLostFrameCounter = 0;
	m_cSuccessfulFrameCounter = 0;

	// Reset pause and signal that the integration resumed
	m_bPauseIntegration = false;
	m_bIntegrationResumed = true;
	m_bCaptureColor = true;

	if (nullptr != m_pCameraPoseFinder)
	{
		m_pCameraPoseFinder->ResetCameraPoseFinder();
	}
}

/// <summary>
/// Process the color image for the camera pose finder.
/// </summary>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT KinectFusionProcessor::ProcessColorForCameraPoseFinder(bool &resampled)
{
	HRESULT hr = S_OK;

	// If color and depth are different resolutions we first we re-sample the color frame using nearest neighbor
	// before passing to the CameraPoseFinder
	if (640*480 !=640*480)
	{
		if (m_pColorImage->width > m_pResampledColorImage->width)
		{
			// Down-sample
			unsigned int factor = m_pColorImage->width / m_pResampledColorImage->width;
			hr = DownsampleFrameNearestNeighbor(m_pColorImage, m_pResampledColorImage, factor);

			if (FAILED(hr))
			{
				cout << ("Kinect Fusion DownsampleFrameNearestNeighbor call failed.") << endl;
				return hr;
			}
		}
		else
		{
			// Up-sample
			unsigned int factor = m_pResampledColorImage->width / m_pColorImage->width;
			hr = UpsampleFrameNearestNeighbor(m_pColorImage, m_pResampledColorImage, factor);

			if (FAILED(hr))
			{
				cout << ("Kinect Fusion UpsampleFrameNearestNeighbor call failed.")<<endl;
				return hr;
			}
		}

		resampled = true;
	}
	else
	{
		resampled = false;
	}

	return hr;
}

/// <summary>
/// Update the camera pose finder data.
/// </summary>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT KinectFusionProcessor::UpdateCameraPoseFinder()
{
	HRESULT hr = S_OK;

	if (nullptr == m_pDepthFloatImage || nullptr == m_pColorImage
		|| nullptr == m_pResampledColorImage || nullptr == m_pCameraPoseFinder)
	{
		return E_FAIL;
	}

	bool resampled = false;

	hr = ProcessColorForCameraPoseFinder(resampled);       //使两者分辨率一致

	if (FAILED(hr))
	{
		return hr;
	}

	BOOL poseHistoryTrimmed = FALSE;
	//BOOL addedPose = FALSE;

	// This function will add the pose to the camera pose finding database when the input frame's minimum
	// distance to the existing database is equal to or above m_fDistanceThresholdAccept (i.e. indicating 
	// that the input has become dis-similar to the existing database and a new frame should be captured).
	// Note that the color and depth frames must be the same size, however, the horizontal mirroring
	// setting does not have to be consistent between depth and color. It does have to be consistent
	// between camera pose finder database creation and calling FindCameraPose though, hence we always
	// reset both the reconstruction and database when changing the mirror depth setting.
	hr = m_pCameraPoseFinder->ProcessFrame(
		m_pDepthFloatImage,
		resampled ? m_pResampledColorImage : m_pColorImage,
		&m_worldToCameraTransform,
		m_fCameraPoseFinderDistanceThresholdAccept,
		&addedPose,
		&poseHistoryTrimmed);

	if (TRUE == addedPose)
	{
		char str[MAX_PATH];
		sprintf_s(str, "Camera Pose Finder Added Frame! %d frames stored, minimum distance>=%f\n", m_pCameraPoseFinder->GetStoredPoseCount(), m_fCameraPoseFinderDistanceThresholdAccept);
		cout << (str);
	}

	if (TRUE == poseHistoryTrimmed)
	{
		cout << ("Kinect Fusion Camera Pose Finder pose history is full, overwritten oldest pose to store current pose.") << endl;
	}

	if (FAILED(hr))
	{
		cout << ("Kinect Fusion Camera Pose Finder Process Frame call failed.") << endl;
	}

	return hr;
}

/// <summary>
/// Reset the reconstruction camera pose and clear the volume on the next frame.
/// </summary>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT KinectFusionProcessor::ResetReconstruction()
{
	m_bResetReconstruction = true;
	return S_OK;
}

/// <summary>
/// Reset the reconstruction camera pose and clear the volume.
/// </summary>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT KinectFusionProcessor::InternalResetReconstruction()
{

	if (nullptr == m_pVolume)
	{
		return E_FAIL;
	}

	HRESULT hr = S_OK;

	SetIdentityMatrix(m_worldToCameraTransform);

	// Translate the world origin away from the reconstruction volume location by an amount equal
	// to the minimum depth threshold. This ensures that some depth signal falls inside the volume.
	// If set false, the default world origin is set to the center of the front face of the 
	// volume, which has the effect of locating the volume directly in front of the initial camera
	// position with the +Z axis into the volume along the initial camera direction of view.
	if (m_bTranslateResetPoseByMinDepthThreshold)
	{
		Matrix4 worldToVolumeTransform = m_defaultWorldToVolumeTransform;

		// Translate the volume in the Z axis by the minDepthThreshold distance
		float minDist = (m_fMinDepthThreshold < m_fMaxDepthThreshold) ? m_fMinDepthThreshold : m_fMaxDepthThreshold;
		worldToVolumeTransform.M43 -= (minDist * m_reconstructionParams.voxelsPerMeter);

		hr = m_pVolume->ResetReconstruction(&m_worldToCameraTransform, &worldToVolumeTransform);
	}
	else
	{
		hr = m_pVolume->ResetReconstruction(&m_worldToCameraTransform, nullptr);
	}

	m_cFrameCounter = 0;
	m_fFrameCounterStartTime = m_timer.AbsoluteTime();

	m_fFramesPerSecond = 0;

	if (SUCCEEDED(hr))
	{
		ResetTracking();
	}

	return hr;
}
