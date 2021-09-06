#include "Camera.h"
#include <iostream>

// Reads DIR/calib/chessboard.xml
// Calibrates a camera based on the images provided.

// Commandline arguments
const cv::String keys =
  "{dir | |  directory  }"
  "{help h usage man ? | | Calibrate a camera}";

int main(int argc, char** argv)
{
  // Read directory to use for files from commandline
  cv::CommandLineParser cl(argc, argv, keys);

  // Help
  if(cl.has("help"))
    { cl.printMessage(); return 0; }
  
  // Read the working directory
  CV_Assert(cl.has("dir"));
  cv::String dir = cl.get<cv::String>("dir");
  std::cout<<"\nReading files from directory "<<dir.c_str();
  std::fflush( stdout );

  // Create chessboard
  char filename[1000];
  sprintf(filename, "%s/calib/chessboard.xml",dir.c_str());
  vc::Chessboard cb(filename);
  
  // Create camera and corners for chessboard
  vc::Camera cam;
  vc::ChessboardCorners corners;
  CV_Assert(!cam.IsCalibrated());
  cam.Calibrate(cb, corners);
  CV_Assert(cam.IsCalibrated());
  sprintf(filename, "%s/calib/camera.xml",dir.c_str());
  cam.Save(filename);
  sprintf(filename, "%s/calib/corners.xml",dir.c_str());
  corners.Save(filename);
}
