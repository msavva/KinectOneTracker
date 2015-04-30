#ifndef KINECTONETRACKER_KINECTONERECORDER_H_
#define KINECTONETRACKER_KINECTONERECORDER_H_

#include <string>
#include <thread>

#include <boost/lockfree/spsc_queue.hpp>
#include <opencv2/opencv.hpp>

#include "./Recording.h"
#include "./KinectOneListener.h"

//! Accumulates skeletons into a Recording
class KinectOneRecorder : public KinectOneListener {
  static const int
    kDepthWidth   = 512,
    kDepthHeight  = 424,
    kColorWidth   = 1920,
    kColorHeight  = 1080;

 public:
  KinectOneRecorder(const bool showCapture = true, const double fps = 5.0, const std::string& recId = "rec_now");

  ~KinectOneRecorder();

  void start() {
    m_isLive = true;
  }

  void stop() {
    m_isLive = false;
  }

  void onSkeleton(const Skeleton* skel);

  void onColor(const INT64 nTime, const UINT nColorBufferSize, const RGBQUAD* pColorBuffer);

  void onDepthAndBodyIndex(const INT64 nTime, const UINT nDepthBufferSize, const UINT16* pDepthBuffer,
                           const UINT nBodyIndexBufferSize, const BYTE* pBodyIndexBuffer);

  Recording& getRecording() const {
    return *m_pRecording;
  }

 private:
  void consumeColor();
  void consumeDepthAndBodyIndex();

  bool m_isLive;
  const bool m_showCapture;
  const double m_fps;
  const int64_t m_frameDeltaTime;
  std::shared_ptr<Recording> m_pRecording;
  cv::VideoWriter
    m_colorWriter,
    m_depthWriter;
  boost::lockfree::spsc_queue<cv::Mat, boost::lockfree::capacity<200>>
    m_colorMatQ,
    m_depthBodyIndexMatQ;
  std::thread
    m_colorWorker,
    m_depthWorker;
  cv::Mat
    m_colorMatYUY2,
    m_colorMatBGR,
    m_colorMatBGRSmall,
    m_depthMat,
    m_depthMatGray,
    m_bodyIndexMat,
    m_depthBodyIndexMat,
    m_depthMatSplit,
    m_depthMatSplitChannels[2];
};

#endif  // KINECTONETRACKER_KINECTONERECORDER_H_
