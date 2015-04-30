#include "./KinectOneTracker.h"

#include <string>
#include <vector>
#include <conio.h>

#include "./Recording.h"
#include "./KinectOneListener.h"

KinectOneTracker::KinectOneTracker()
  : m_doQuit(false)
  , m_pKinectSensor(NULL)
  , m_pMultiSourceFrameReader(NULL)
  , m_pCoordinateMapper(NULL)
  , m_pSkel(new Skeleton()) { }

KinectOneTracker::~KinectOneTracker() {
  if (m_pSkel != NULL) { delete m_pSkel; }
  SafeRelease(m_pMultiSourceFrameReader);
  SafeRelease(m_pCoordinateMapper);
  if (m_pKinectSensor) { m_pKinectSensor->Close(); }
  SafeRelease(m_pKinectSensor);
}

void KinectOneTracker::run() {
  while (!m_doQuit) {
    update();
    if (_kbhit()) { quit(); }
  }
}

bool KinectOneTracker::init() {
  HRESULT hr = GetDefaultKinectSensor(&m_pKinectSensor);
  if (FAILED(hr)) { return false; }

  // Initialize the Kinect, get coordinate mapper and frame reader
  if (m_pKinectSensor) {
    hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
    if (SUCCEEDED(hr)) { hr = m_pKinectSensor->Open(); }
    if (SUCCEEDED(hr)) {
      hr = m_pKinectSensor->OpenMultiSourceFrameReader(
        FrameSourceTypes::FrameSourceTypes_Depth |
        FrameSourceTypes::FrameSourceTypes_Color |
        FrameSourceTypes::FrameSourceTypes_BodyIndex |
        FrameSourceTypes::FrameSourceTypes_Body,
        &m_pMultiSourceFrameReader);
    }
  }

  if (!m_pKinectSensor || FAILED(hr)) { return false; }
  return true;
}

void KinectOneTracker::processBody(IMultiSourceFrame* pMultiSourceFrame) {
  // Get BodyFrame
  IBodyFrame* pBodyFrame = NULL;
  IBodyFrameReference* pBodyFrameReference = NULL;
  HRESULT hr = pMultiSourceFrame->get_BodyFrameReference(&pBodyFrameReference);
  if (SUCCEEDED(hr)) { hr = pBodyFrameReference->AcquireFrame(&pBodyFrame);}
  SafeRelease(pBodyFrameReference);
  if (FAILED(hr)) { return; }

  // Process BodyFrame
  INT64 nBodyTime = 0;
  hr = pBodyFrame->get_RelativeTime(&nBodyTime);
  IBody* ppBodies[BODY_COUNT] = {0};
  if (SUCCEEDED(hr)) { hr = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies); }
  SafeRelease(pBodyFrame);
  if (FAILED(hr)) { return; }

  for (int i = 0; i < BODY_COUNT; ++i) {
    IBody* pBody = ppBodies[i];
    if (pBody) {
      BOOLEAN bTracked = false;
      hr = pBody->get_IsTracked(&bTracked);
      if (SUCCEEDED(hr) && bTracked) {
        pBody->get_TrackingId(&m_pSkel->trackingId);
        m_pSkel->timestamp = nBodyTime;

        // Hand states
        m_leftHandState = HandState_Unknown;
        hr = pBody->get_HandLeftState(&m_leftHandState);
        if (SUCCEEDED(hr)) { m_pSkel->handLeftState = static_cast<Skeleton::HandState>(m_leftHandState); }
        pBody->get_HandLeftConfidence(&m_leftHandConfidence);
        m_pSkel->handLeftConfidence = static_cast<Skeleton::TrackingConfidence>(m_leftHandConfidence);
        m_rightHandState = HandState_Unknown;
        hr = pBody->get_HandRightState(&m_rightHandState);
        if (SUCCEEDED(hr)) { m_pSkel->handRightState = static_cast<Skeleton::HandState>(m_rightHandState); }
        pBody->get_HandRightConfidence(&m_rightHandConfidence);
        m_pSkel->handRightConfidence = static_cast<Skeleton::TrackingConfidence>(m_rightHandConfidence);

        // Joints
        hr = pBody->GetJoints(_countof(m_joints), m_joints);
        if (SUCCEEDED(hr)) {
          for (int j = 0; j < _countof(m_joints); ++j) {
            const auto& p = m_joints[j].Position;
            auto& pOut = m_pSkel->jointPositions[j];
            pOut[0] = p.X;  pOut[1] = p.Y;  pOut[2] = p.Z;

            if (m_joints[j].TrackingState == TrackingState_Inferred) { m_pSkel->jointConfidences[j] = 0.5f; }
            else if (m_joints[j].TrackingState == TrackingState_Tracked) { m_pSkel->jointConfidences[j] = 1.0f; }
            else { m_pSkel->jointConfidences[j] = 0.0f; }
          }
        }

        // Joint orientations
        hr = pBody->GetJointOrientations(_countof(m_orients), m_orients);
        if (SUCCEEDED(hr)) {
          for (int j = 0; j < _countof(m_orients); ++j) {
            const auto& p = m_orients[j].Orientation;
            auto& pOut = m_pSkel->jointOrientations[j];
            pOut[0] = p.x;  pOut[1] = p.y;  pOut[2] = p.z;  pOut[3] = p.w;
          }
        }

        // Activities
        hr = pBody->GetActivityDetectionResults(_countof(m_activities), m_activities);
        if (SUCCEEDED(hr)) {
          for (int j = 0; j < _countof(m_activities); ++j) {
            m_pSkel->activities[j] = static_cast<Skeleton::DetectionResult>(m_activities[j]);
          }
        }

        // Lean tracking state
        m_leanTrackingState = TrackingState_NotTracked;
        hr = pBody->get_LeanTrackingState(&m_leanTrackingState);
        if (SUCCEEDED(hr)) {
          pBody->get_Lean(&m_lean);
          m_pSkel->leanLeftRight = m_lean.X;
          m_pSkel->leanForwardBack = m_lean.Y;
          if (m_leanTrackingState == TrackingState_Tracked) {
            m_pSkel->leanConfidence = 1.0f;
          } else if (m_leanTrackingState == TrackingState_Inferred) {
            m_pSkel->leanConfidence = 0.5f;
          } else {
            m_pSkel->leanConfidence = 0.0f;
          }
        }

        // Frame edges
        pBody->get_ClippedEdges(&m_pSkel->clippedEdges);

        for (KinectOneListener* l : m_skelListeners) { l->onSkeleton(m_pSkel); }
      }
    }
  }

  for (int i = 0; i < _countof(ppBodies); ++i) { SafeRelease(ppBodies[i]); }
}

std::vector<std::pair<float, float>> KinectOneTracker::getDepthPixelCoordsInCameraSpace() {
  PointF* pTable;
  UINT32 nTableCount = 0;
  HRESULT hr = m_pCoordinateMapper->GetDepthFrameToCameraSpaceTable(&nTableCount, &pTable);
  std::vector<std::pair<float, float>> out(nTableCount);
  if (SUCCEEDED(hr)) {
    memcpy(out.data(), pTable, sizeof(pTable[0]) * nTableCount);
  }
  return out;
}

// Aligns colors in pColorBuffer to depth points in pDepthBuffer and returns aligned color in colorOut
void KinectOneTracker::depthToColor(const UINT nDepthSize, const uint16_t* pDepthBuffer,
                                    const UINT colorWidth, const UINT colorHeight, const uint32_t* pColorBuffer,
                                    uint32_t* colorOut) {
  ColorSpacePoint* pColorCoordinates = new ColorSpacePoint[nDepthSize];
  HRESULT hr = m_pCoordinateMapper->MapDepthFrameToColorSpace(nDepthSize, pDepthBuffer, nDepthSize, pColorCoordinates);
  if (FAILED(hr)) { return; }
  for (UINT depthIndex = 0; depthIndex < nDepthSize; ++depthIndex) {
    ColorSpacePoint colorPoint = pColorCoordinates[depthIndex];
    int colorX = static_cast<int>(floor(colorPoint.X + 0.5));
    int colorY = static_cast<int>(floor(colorPoint.Y + 0.5));
    if ((colorX >= 0) && (colorX < static_cast<int>(colorWidth)) &&
        (colorY >= 0) && (colorY < static_cast<int>(colorHeight))) {
      int colorIndex = colorX + (colorY * colorWidth);
      *colorOut = pColorBuffer[colorIndex];
    }
  }
  delete [] pColorCoordinates;
}

void KinectOneTracker::processColor(IMultiSourceFrame* pMultiSourceFrame) {
  IColorFrame* pColorFrame = NULL;
  IColorFrameReference* pColorFrameReference = NULL;
  HRESULT hr = pMultiSourceFrame->get_ColorFrameReference(&pColorFrameReference);
  if (SUCCEEDED(hr)) { hr = pColorFrameReference->AcquireFrame(&pColorFrame);}

  int64_t nColorTime = 0;
  IFrameDescription* pColorFrameDescription = NULL;
  int nColorWidth = 0;
  int nColorHeight = 0;
  ColorImageFormat imageFormat = ColorImageFormat_None;
  UINT nColorBufferSize = 0;
  RGBQUAD* pColorBuffer = NULL;

  if (SUCCEEDED(hr)) { hr = pColorFrame->get_RelativeTime(&nColorTime); }
  if (SUCCEEDED(hr)) { hr = pColorFrame->get_FrameDescription(&pColorFrameDescription); }
  if (SUCCEEDED(hr)) { hr = pColorFrameDescription->get_Width(&nColorWidth); }
  if (SUCCEEDED(hr)) { hr = pColorFrameDescription->get_Height(&nColorHeight); }
  if (SUCCEEDED(hr)) { hr = pColorFrame->get_RawColorImageFormat(&imageFormat); }
  if (SUCCEEDED(hr)) {
    //assert(imageFormat == ColorImageFormat_Yuy2);
    hr = pColorFrame->AccessRawUnderlyingBuffer(&nColorBufferSize, reinterpret_cast<BYTE**>(&pColorBuffer));
    for (KinectOneListener* l : m_colorListeners) { l->onColor(nColorTime, nColorBufferSize, pColorBuffer); }
  }

  SafeRelease(pColorFrameReference);
  SafeRelease(pColorFrameDescription);
  SafeRelease(pColorFrame);
}

void KinectOneTracker::processDepthAndBodyIndex(IMultiSourceFrame* pMultiSourceFrame) {
  // Depth Frame
  IDepthFrame* pDepthFrame = NULL;
  IDepthFrameReference* pDepthFrameReference = NULL;
  HRESULT hr = pMultiSourceFrame->get_DepthFrameReference(&pDepthFrameReference);
  if (SUCCEEDED(hr)) { hr = pDepthFrameReference->AcquireFrame(&pDepthFrame); }

  INT64 nDepthTime = 0;
  IFrameDescription* pDepthFrameDescription = NULL;
  int nDepthWidth = 0;
  int nDepthHeight = 0;
  UINT nDepthBufferSize = 0;
  UINT16* pDepthBuffer = NULL;

  if (SUCCEEDED(hr)) { hr = pDepthFrame->get_RelativeTime(&nDepthTime); }
  if (SUCCEEDED(hr)) { hr = pDepthFrame->get_FrameDescription(&pDepthFrameDescription); }
  if (SUCCEEDED(hr)) { hr = pDepthFrameDescription->get_Width(&nDepthWidth); }
  if (SUCCEEDED(hr)) { hr = pDepthFrameDescription->get_Height(&nDepthHeight); }
  if (SUCCEEDED(hr)) { hr = pDepthFrame->AccessUnderlyingBuffer(&nDepthBufferSize, &pDepthBuffer); }

  // Body Index Frame
  IBodyIndexFrame* pBodyIndexFrame = NULL;
  IBodyIndexFrameReference* pBodyIndexFrameReference = NULL;
  hr = pMultiSourceFrame->get_BodyIndexFrameReference(&pBodyIndexFrameReference);
  if (SUCCEEDED(hr)) { hr = pBodyIndexFrameReference->AcquireFrame(&pBodyIndexFrame); }

  INT64 nBodyIndexTime = 0;
  IFrameDescription* pBodyIndexFrameDescription = NULL;
  int nBodyIndexWidth = 0;
  int nBodyIndexHeight = 0;
  UINT nBodyIndexBufferSize = 0;
  BYTE* pBodyIndexBuffer = NULL;

  if (SUCCEEDED(hr)) { hr = pBodyIndexFrame->get_RelativeTime(&nBodyIndexTime); }
  if (SUCCEEDED(hr)) { hr = pBodyIndexFrame->get_FrameDescription(&pBodyIndexFrameDescription); }
  if (SUCCEEDED(hr)) { hr = pBodyIndexFrameDescription->get_Width(&nBodyIndexWidth); }
  if (SUCCEEDED(hr)) { hr = pBodyIndexFrameDescription->get_Height(&nBodyIndexHeight); }
  if (SUCCEEDED(hr)) { hr = pBodyIndexFrame->AccessUnderlyingBuffer(&nBodyIndexBufferSize, &pBodyIndexBuffer); }

  for (KinectOneListener* l : m_depthListeners) {
    l->onDepthAndBodyIndex(nDepthTime, nDepthBufferSize, pDepthBuffer, nBodyIndexBufferSize, pBodyIndexBuffer);
  }

  SafeRelease(pDepthFrameReference);
  SafeRelease(pDepthFrameDescription);
  SafeRelease(pDepthFrame);
  SafeRelease(pBodyIndexFrameReference);
  SafeRelease(pBodyIndexFrameDescription);
  SafeRelease(pBodyIndexFrame);
}

void KinectOneTracker::update() {
  if (!m_pMultiSourceFrameReader) { return; }

  IMultiSourceFrame* pMultiSourceFrame = NULL;
  HRESULT hr = m_pMultiSourceFrameReader->AcquireLatestFrame(&pMultiSourceFrame);
  if (FAILED(hr) || pMultiSourceFrame == NULL) { return; }

  if (!m_colorListeners.empty()) { processColor(pMultiSourceFrame); }
  if (!m_depthListeners.empty()) { processDepthAndBodyIndex(pMultiSourceFrame); }
  if (!m_skelListeners.empty())  { processBody(pMultiSourceFrame); }

  SafeRelease(pMultiSourceFrame);
}
