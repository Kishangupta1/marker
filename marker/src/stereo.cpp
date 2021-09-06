#include "StereoRig.h"
#include <opencv2/highgui.hpp>
#include <iostream>

// Commandline arguments
const cv::String keys =
  "{dir | |  directory  }"
  "{help h usage ? | | Stereocalibration for a pair of cameras}";


int main(int argc, char** argv)
{
  // Read directory to use for files from commandline
  cv::CommandLineParser cl(argc, argv, keys);

  // Request for help
  if(cl.has("help"))
    { cl.printMessage(); return 0; }
  
  // Stereocalibration
  CV_Assert(cl.has("dir"));
  cv::String dir = cl.get<cv::String>("dir");
  std::cout<<"\nReading files from directory "<<dir.c_str(); std::fflush( stdout );
  
  char filename[1000];
  sprintf(filename, "%s/lcam/calib/camera.xml", dir.c_str());
  vc::Camera lcam(filename);
  sprintf(filename, "%s/rcam/calib/camera.xml", dir.c_str());
  vc::Camera rcam(filename);
  sprintf(filename, "%s/lcam/calib/corners.xml", dir.c_str());
  vc::ChessboardCorners lcorners(filename);
  sprintf(filename, "%s/rcam/calib/corners.xml", dir.c_str());
  vc::ChessboardCorners rcorners(filename);
  CV_Assert(lcam.IsCalibrated() && rcam.IsCalibrated());

  // Create stereo-rig
  vc::StereoRig rig(lcam, lcorners, rcam, rcorners);
  sprintf(filename, "%s/stereorig.xml", dir.c_str());
  rig.Save(filename);
 }
