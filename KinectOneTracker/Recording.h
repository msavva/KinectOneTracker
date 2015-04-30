#ifndef RECORDING_H_
#define RECORDING_H_

#include <cstdint>
#include <string>
#include <vector>
#include <array>

// Skeleton struct modeled after Kinect for Windows SDK v2 API's Body class.
// See http://msdn.microsoft.com/en-us/library/dn799271.aspx for more details.
struct Skeleton {
  enum JointType {
    JointType_SpineBase = 0,
    JointType_SpineMid = 1,
    JointType_Neck = 2,
    JointType_Head = 3,
    JointType_ShoulderLeft = 4,
    JointType_ElbowLeft = 5,
    JointType_WristLeft = 6,
    JointType_HandLeft = 7,
    JointType_ShoulderRight = 8,
    JointType_ElbowRight = 9,
    JointType_WristRight = 10,
    JointType_HandRight = 11,
    JointType_HipLeft = 12,
    JointType_KneeLeft = 13,
    JointType_AnkleLeft = 14,
    JointType_FootLeft = 15,
    JointType_HipRight = 16,
    JointType_KneeRight = 17,
    JointType_AnkleRight = 18,
    JointType_FootRight = 19,
    JointType_SpineShoulder = 20,
    JointType_HandTipLeft = 21,
    JointType_ThumbLeft = 22,
    JointType_HandTipRight = 23,
    JointType_ThumbRight = 24,
    JointType_Count = 25
  };
  enum HandState {
    HandState_Unknown = 0,
    HandState_NotTracked = 1,
    HandState_Open = 2,
    HandState_Closed = 3,
    HandState_Lasso = 4
  };
  enum TrackingConfidence {
    TrackingConfidence_Low = 0,
    TrackingConfidence_High = 1
  };
  enum TrackingState {
    TrackingState_NotTracked = 0,
    TrackingState_Inferred = 1,
    TrackingState_Tracked = 2
  };
  enum DetectionResult {
    DetectionResult_Unknown = 0,
    DetectionResult_No = 1,
    DetectionResult_Maybe = 2,
    DetectionResult_Yes = 3
  };
  enum Activity {
    Activity_EyeLeftClosed = 0,
    Activity_EyeRightClosed = 1,
    Activity_MouthOpen = 2,
    Activity_MouthMoved = 3,
    Activity_LookingAway = 4,
    Activity_Count = (Activity_LookingAway + 1)
  };
  enum FrameEdges {
    FrameEdge_None = 0,
    FrameEdge_Right = 0x1,
    FrameEdge_Left = 0x2,
    FrameEdge_Top = 0x4,
    FrameEdge_Bottom = 0x8
  };

  // Tracking id of current skeleton (unique for each tracked person)
  uint64_t                  trackingId;
  // Position of each joint in camera coordinate frame
  std::array<float, 3>      jointPositions[JointType_Count];
  // Confidence of track for each joint [0,1]
  float                     jointConfidences[JointType_Count];
  // Orientation of each joint expressed as quaternion
  std::array<float, 4>      jointOrientations[JointType_Count];
  // State of left and right hand
  HandState                 handLeftState, handRightState;
  // Tracking confidence of left and right hand
  TrackingConfidence        handLeftConfidence, handRightConfidence;
  // Detected activities (see Activity struct for definition of each index into array)
  DetectionResult           activities[Activity_Count];
  // Confidence of body leaning track, left-right [0,1] and forward-back [0,1] values
  float                     leanConfidence, leanLeftRight, leanForwardBack;
  // Mask indicating Skeleton clips edge (active bits or'ed according to FrameEdge)
  unsigned long             clippedEdges;
  // Device timestamp at which current Skeleton was tracked
  int64_t                   timestamp;
};

//! Recording containing a stream of Skeletons as well as optional color
//! and combined depth+bodyIndex frame timestamps. Actual frames are stored
//! externally in video files.
struct Recording {
  // Identifier of this recording
  std::string id;
  // 4x4 row-major camera transformation matrix
  std::array<float, 16> camera;
  // Device timestamp at start of recording (microseconds)
  uint64_t startTime;
  // Device timestamp at end of recording (microseconds)
  uint64_t endTime;
  // Vector of tracked Skeletons
  std::vector<Skeleton> skeletons;
  // Timestamps of recorded color frames
  std::vector<int64_t> colorTimestamps;
  // Timestamps of recorded depth frames
  std::vector<int64_t> depthTimestamps;

  // STATE - NOT STORED
  //! Whether this Recording is currently being recorded
  bool isLive;
  //! Whether this Recording has been loaded from file
  bool isLoaded;

  //! Save to JSON file
  bool saveToJSON(const std::string& file);
};

#endif  // RECORDING_H_
