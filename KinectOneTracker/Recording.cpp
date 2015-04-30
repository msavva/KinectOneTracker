#include "./Recording.h"

#include <string>
#include <iostream>
#include <fstream>
#include <functional>

using std::string;  using std::cout;  using std::cerr;  using std::endl;
using std::ostream;

// Return a function that outputs string s and optionally endline into ostream os on every call
std::function<void(void)> put(std::ostream& os, const string& s, bool endlines = false) {  // NOLINT
  return ([&os, s, endlines] () {
    os << s;
    if (endlines) {
      os << endl;
    }
  });
}

// Writes each element of array-like type x to os in JSON format: [a, b, ..., z]
template<typename T>
ostream& arr2json(ostream& os, const T& x, const size_t numXs) {  // NOLINT
  if (numXs == 0) {
    return os << "[]";
  }
  os << "[" << x[0];
  for (size_t i = 1; i < numXs; ++i) {
    os << "," << x[i];
  }
  return os << "]";
}

// Writes each element of array a to os in JSON format: [a, b, ..., z]
template <typename T, size_t DIM>
ostream& operator<<(ostream& os, const std::array<T,DIM>& a) {  // NOLINT
  return arr2json(os, a, DIM);
}
template <typename T>
ostream& operator<<(ostream& os, const std::vector<T>& v) {  // NOLINT
  return arr2json(os, v, v.size());
}

// Writes Recording as JSON format string to ostream
void rec2json(ostream& os, const Recording& rec, bool endlines) {  // NOLINT
  const std::function<string(string)> key = [ ] (const string& id) { return "\"" + id + "\": "; };
  const std::function<void(void)>     sep = put(os, ",", endlines);

  const auto skel2json = [&] (ostream& os, const Skeleton& s, bool endlines) {  // NOLINT
    os << "{";                      if (endlines) { os << endl; }
    os << key("trackingId")         << s.trackingId; sep();
    os << key("jointPositions");    arr2json(os, s.jointPositions, s.JointType_Count); sep();
    os << key("jointConfidences");  arr2json(os, s.jointConfidences, s.JointType_Count); sep();
    os << key("jointOrientations"); arr2json(os, s.jointOrientations, s.JointType_Count); sep();
    os << key("handState")          << "[" << s.handLeftState << "," << s.handLeftConfidence << ","
                                    << s.handRightState << "," << s.handRightConfidence << "]"; sep();
    os << key("activities");        arr2json(os, s.activities, s.Activity_Count); sep();
    os << key("leanState")          << "[" << s.leanLeftRight << "," << s.leanForwardBack
                                    << "," << s.leanConfidence << "]"; sep();
    os << key("clippedEdges")       << s.clippedEdges; sep();
    os << key("timestamp")          << s.timestamp;  if (endlines) { os << endl; }
    os << "}";                      if (endlines) { os << endl; }
  };

  os << "{";                        if (endlines) { os << endl; }
  os << key("id")                   << "\"" + rec.id + "\"";  sep();
  os << key("camera")               << rec.camera;  sep();
  os << key("startTime")            << rec.startTime;  sep();
  os << key("endTime")              << rec.endTime;  sep();
  os << key("skeletons") << "[";    if (endlines) { os << endl; }
  const size_t numSkels = rec.skeletons.size();
  for (size_t iSkel = 0; iSkel < numSkels; ++iSkel) {
    skel2json(os, rec.skeletons[iSkel], endlines);
    if (iSkel < numSkels - 1) { sep(); }
  }
  os << "]"; sep();
  os << key("colorTimestamps");     arr2json(os, rec.colorTimestamps, rec.colorTimestamps.size());  sep();
  os << key("depthTimestamps");     arr2json(os, rec.depthTimestamps, rec.colorTimestamps.size());
                                    if (endlines) { os << endl; }
  os << "}";                        if (endlines) { os << endl; }
}


bool Recording::saveToJSON(const std::string& file) {
  std::ofstream ofs(file);
  rec2json(ofs, *this, true);
  ofs.close();
  return true;
}
