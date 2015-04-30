#ifndef KINECTONETRACKER_KINECTONETRACKER_H_
#define KINECTONETRACKER_KINECTONETRACKER_H_

#include <list>
#include <vector>

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Kinect.h>

// Forward declarations
struct KinectOneListener;
struct Skeleton;

// Kinect One skeleton tracker
class KinectOneTracker {
 public:
  KinectOneTracker();
  ~KinectOneTracker();
  bool init();
  void run();
  void attachSkeletonListener(KinectOneListener* skelListen) { m_skelListeners.push_back(skelListen); }
  void attachColorListener(KinectOneListener* colorListen) { m_colorListeners.push_back(colorListen); }
  void attachDepthListener(KinectOneListener* depthListen) { m_depthListeners.push_back(depthListen); }
  void quit() { m_doQuit = true; }

  std::vector<std::pair<float, float>> getDepthPixelCoordsInCameraSpace();

 private:
  void processBody(IMultiSourceFrame* pMultiSourceFrame);
  void processColor(IMultiSourceFrame* pMultiSourceFrame);
  void processDepthAndBodyIndex(IMultiSourceFrame* pMultiSourceFrame);
  void depthToColor(const unsigned int nDepthSize, const unsigned short* pDepthBuffer,
                    const unsigned int colorWidth, const unsigned int colorHeight,
                    const unsigned int* pColorBuffer, unsigned int* colorOut);
  void update();

  // Safe release for interfaces
  template<class Interface>
  inline void SafeRelease(Interface*& pInterfaceToRelease) {  // NOLINT
    if (pInterfaceToRelease != NULL) {
      pInterfaceToRelease->Release();
      pInterfaceToRelease = NULL;
    }
  }

  std::list<KinectOneListener*>
    m_skelListeners,
    m_colorListeners,
    m_depthListeners;
  bool m_doQuit;

  IKinectSensor*            m_pKinectSensor;
  IMultiSourceFrameReader*  m_pMultiSourceFrameReader;
  ICoordinateMapper*        m_pCoordinateMapper;

  Joint                     m_joints[JointType_Count];
  JointOrientation          m_orients[JointType_Count];
  DetectionResult           m_activities[Activity_Count];
  HandState                 m_leftHandState, m_rightHandState;
  TrackingConfidence        m_leftHandConfidence, m_rightHandConfidence;
  TrackingState             m_leanTrackingState;
  PointF                    m_lean;
  Skeleton*                 m_pSkel;
};

#endif  // KINECTONETRACKER_KINECTONETRACKER_H_
