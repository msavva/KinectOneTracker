# KinectOneTracker
A simple recording utility for the KinectOne sensor.

## Build
- Install [biicode](http://www.biicode.com)
- Install [CMake](http://www.cmake.org), or get biicode to install it for you with `bii setup:cpp`.
- Run `bii init -l` in the project directory, followed by `bii configure -G "CMake Generator Name"` (e.g., `bii configure -G "Visual Studio 12 Win64"`)
- Run `bii buzz` to build (binaries will be in bin/ and generated project files are in bii/build/)

## Prerequisites
- [Kinect SDK v2](https://www.microsoft.com/en-us/kinectforwindows/develop/)
- [Lagarith lossless codec](http://lags.leetcode.net/codec.html)
- KinectOne sensor connected to USB3 port
- Internal dependencies on OpenCV and Boost are handled by biicode

## Run

Start compiled binary in bin folder to record.  Press a key to stop recording, and save files.  Each recording is stored as a JSON header containing skeletal tracking information (see [Recording.cpp](KinectOneTracker/Recording.cpp)), and two AVI files encoded losslessly with Lagarith: one for depth and the other for color.

Some basic parameters are currently hardcoded in [main.cpp](KinectOneTracker/main.cpp#L21):
- id : recording id used as prefix in files
- fps : frames per second for depth and color video
- showCapture : whether to show live depth and color frames
