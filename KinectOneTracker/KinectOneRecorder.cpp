#include "./KinectOneRecorder.h"

#include <string>

using std::string;  using std::cout;  using std::cerr;  using std::endl;

KinectOneRecorder::KinectOneRecorder(const bool showCapture, const double fps, const string& recId)
  : m_pRecording(new Recording)
  , m_isLive(true)
  , m_showCapture(showCapture)
  , m_fps(fps)
  , m_frameDeltaTime(static_cast<int64_t>(1.0E7 / m_fps))
  , m_colorMatYUY2(kColorHeight, kColorWidth, CV_8UC2)
  , m_colorMatBGRSmall(kColorHeight / 2, kColorWidth / 2, CV_8UC3)
  , m_colorMatBGR(kColorHeight, kColorWidth, CV_8UC3)
  , m_depthMat(kDepthHeight, kDepthWidth, CV_16UC1)
  , m_depthMatSplit(kDepthHeight, kDepthWidth, CV_8UC2)
  , m_bodyIndexMat(kDepthHeight, kDepthWidth, CV_8UC1)
  , m_depthMatGray(kDepthHeight, kDepthWidth, CV_8U)
  , m_depthBodyIndexMat(kDepthHeight, kDepthWidth, CV_8UC3)
  , m_colorWorker(&KinectOneRecorder::consumeColor, this)
  , m_depthWorker(&KinectOneRecorder::consumeDepthAndBodyIndex, this) {
    m_pRecording->camera = {{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}};
    m_pRecording->id = recId;
    const int fourccLAGS = cv::VideoWriter::fourcc('L', 'A', 'G', 'S');
    const string
      colorFile = recId + ".color.avi",
      depthFile = recId + ".depth.avi";

    m_colorWriter.open(colorFile.c_str(), fourccLAGS, m_fps, m_colorMatBGRSmall.size());
    if (!m_colorWriter.isOpened()) {
      cerr << "Could not open color video file " << colorFile << endl;
    }

    m_depthWriter.open(depthFile.c_str(), fourccLAGS, m_fps, m_depthMat.size());
    if (!m_depthWriter.isOpened()) {
      cerr << "Could not open depth video file " << depthFile << endl;
    }

    if (!m_colorMatQ.is_lock_free()) {
      cerr << "Warning: frame consumer queues not lock-free." << endl;
    }
}

KinectOneRecorder::~KinectOneRecorder() {
  m_isLive = false;
  if (m_colorWorker.joinable()) { m_colorWorker.join(); }
  if (m_depthWorker.joinable()) { m_depthWorker.join(); }
  if (m_colorWriter.isOpened()) { m_colorWriter.release(); }
  if (m_depthWriter.isOpened()) { m_depthWriter.release(); }
  m_pRecording->isLive = false;
}

//! Returns system time as number of microseconds since Jan 1st 1601 UTC
inline uint64_t systemTimeNow() {
  FILETIME ft;
  GetSystemTimeAsFileTime(&ft);
  ULARGE_INTEGER t;
  t.HighPart = ft.dwHighDateTime;
  t.LowPart = ft.dwLowDateTime;
  return t.QuadPart / 10;
}

void KinectOneRecorder::onSkeleton(const Skeleton* skel) {
  if (!m_isLive) { return; }
  if (!m_pRecording->isLive) { m_pRecording->isLive = true; }
  if (m_pRecording->skeletons.empty()) {
    m_pRecording->startTime = systemTimeNow();
  }
  m_pRecording->skeletons.push_back(*skel);
  m_pRecording->endTime = systemTimeNow();
}

void KinectOneRecorder::onColor(const INT64 nTime, const UINT nColorBufferSize, const RGBQUAD* pColorBuffer) {
  if (!m_isLive) { return; }
  if (m_pRecording->colorTimestamps.empty() || (nTime - m_pRecording->colorTimestamps.back()) > m_frameDeltaTime) {
    memcpy(m_colorMatYUY2.data, pColorBuffer, nColorBufferSize);
    while (!m_colorMatQ.push(m_colorMatYUY2)) { }
    m_pRecording->colorTimestamps.push_back(nTime);
  }
}

void KinectOneRecorder::onDepthAndBodyIndex(const INT64 nTime, const UINT nDepthBufferSize, const UINT16* pDepthBuffer,
                                            const UINT nBodyIndexBufferSize, const BYTE* pBodyIndexBuffer) {
  if (!m_isLive) { return; }
  if (m_pRecording->depthTimestamps.empty() || (nTime - m_pRecording->depthTimestamps.back()) > m_frameDeltaTime) {
    memcpy(m_depthMatSplit.data, pDepthBuffer, sizeof(pDepthBuffer[0]) * nDepthBufferSize);
    memcpy(m_bodyIndexMat.data, pBodyIndexBuffer, sizeof(pBodyIndexBuffer[0]) * nBodyIndexBufferSize);
    cv::split(m_depthMatSplit, m_depthMatSplitChannels);
    cv::Mat in[] = { m_bodyIndexMat, m_depthMatSplitChannels[0], m_depthMatSplitChannels[1] };
    cv::merge(in, 3, m_depthBodyIndexMat);
    while (!m_depthBodyIndexMatQ.push(m_depthBodyIndexMat)) { }
    m_pRecording->depthTimestamps.push_back(nTime);
  }
}

void KinectOneRecorder::consumeColor() {
  cv::Mat matYUY2;
  while (m_isLive) {
    while (m_colorMatQ.pop(matYUY2)) {
      cv::cvtColor(matYUY2, m_colorMatBGR, cv::COLOR_YUV2BGR_YUY2);
      cv::resize(m_colorMatBGR, m_colorMatBGRSmall, m_colorMatBGRSmall.size(), 0, 0, cv::INTER_LINEAR);
      if (m_showCapture) {
        cv::imshow("Color", m_colorMatBGRSmall);
        cv::waitKey(1);
      }
      if (m_colorWriter.isOpened()) { m_colorWriter << m_colorMatBGRSmall; }
    }
  }
}

void KinectOneRecorder::consumeDepthAndBodyIndex() {
  cv::Mat matDepthAndBodyIndex;
  while (m_isLive) {
    while (m_depthBodyIndexMatQ.pop(matDepthAndBodyIndex)) {
      if (m_showCapture) {
        cv::imshow("Depth+BodyIndex", matDepthAndBodyIndex);
        cv::waitKey(1);
      }
      if (m_depthWriter.isOpened()) { m_depthWriter << matDepthAndBodyIndex; }
    }
  }
}
