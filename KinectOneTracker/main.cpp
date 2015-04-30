#include <string>
#include <vector>

#include "./KinectOneTracker.h"
#include "./KinectOneRecorder.h"

using std::string;  using std::cout;  using std::cerr;  using std::endl;

//! Return current time as Year-Month-Day-Hours-Minutes-Seconds string
string timeAsYMDHMS() {
  time_t rawtime;
  tm timeinfo;
  time(&rawtime);
  localtime_s(&timeinfo, &rawtime);
  char timestr[80];
  strftime(timestr, 80, "%Y-%m-%d-%H-%M-%S", &timeinfo);
  return timestr;
}

int main(int argc, const char** argv) {
  // Parameters
  const string id_time     = "rec_" + timeAsYMDHMS();
  const double fps         = 5.0;
  const bool   showCapture = true;

  // Initialize tracker and skeleton recorder
  KinectOneTracker tracker;
  tracker.init();
  KinectOneRecorder kinectRec(showCapture, fps, id_time);
  tracker.attachSkeletonListener(&kinectRec);
  tracker.attachColorListener(&kinectRec);
  tracker.attachDepthListener(&kinectRec);

  // Spawn tracker thread
  std::thread trackerThread(&KinectOneTracker::run, std::ref(tracker));

  // Wait for character to quit
  cout << "Recording started. Press any key to stop." << endl;
  char c = getchar();

  tracker.quit();
  trackerThread.join();
  kinectRec.stop();

  // Dump recording to file and report
  Recording& rec = kinectRec.getRecording();
  const string recFile = rec.id + ".json";
  rec.saveToJSON(recFile);

  cout << "Exiting..." << endl;

  return 0;
}
