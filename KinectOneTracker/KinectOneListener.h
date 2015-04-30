#ifndef KINECTONETRACKER_KINECTONELISTENER_H_
#define KINECTONETRACKER_KINECTONELISTENER_H_

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Kinect.h>

// Forward declaration
struct Skeleton;

// Interface for acquiring KinectOne frames
struct KinectOneListener {
  virtual void onSkeleton(const Skeleton* skel) = 0;
  virtual void onColor(const INT64 nTime, const UINT nColorBufferSize, const RGBQUAD* pColorBuffer) = 0;
  virtual void onDepthAndBodyIndex(const INT64 nTime, const UINT nDepthBufferSize, const UINT16* pDepthBuffer,
                                   const UINT nBodyIndexBufferSize, const BYTE* pBodyIndexBuffer) = 0;
};

#endif  // KINECTONETRACKER_KINECTONELISTENER_H_
