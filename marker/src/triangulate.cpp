#include <Triangulator.h>
#include <fstream>
#include <iostream>

int main()
{
  // Create rig
  vc::StereoRig rig("./stereorig.xml");

  // Create triangulator
  vc::Triangulator tri(rig);

  // Read the set of pixels to triangulate from the left camera
  std::vector<cv::Point2d> LeftPixels({});
  std::fstream pfile;
  pfile.open((char*)"left.txt", std::ios::in);
  // Format: i j
  CV_Assert(pfile.good());
  double x, y;
  pfile >> x;
  while(pfile.good())
    {
      pfile >> y;
      LeftPixels.push_back(cv::Point2d(x,y));
      pfile >> x;
    }
  pfile.close();

  // Read the set of pixels to triangulate from the right camera
  std::vector<cv::Point2d> RightPixels({});
  pfile.open((char*)"right.txt", std::ios::in);
  // Format: i j
  CV_Assert(pfile.good());
  pfile >> x;
  while(pfile.good())
    {
      pfile >> y;
      RightPixels.push_back(cv::Point2d(x,y));
      pfile >> x;
    }
  pfile.close();

  CV_Assert(LeftPixels.size()==RightPixels.size());
  std::cout<<"\nRead "<<LeftPixels.size()<<" corresponding pixels "<<std::flush;

  const int nPixels = static_cast<int>(LeftPixels.size());

  // Triangulate each pair of corresponding pixels
  for(int p=0; p<nPixels; ++p)
    {
      cv::Point3d X;
      tri.Triangulate<vc::TriMethod::Unsym>(LeftPixels[p], RightPixels[p], X);
      std::cout<<"\n"<<X.x<<", "<<X.y<<", "<<X.z<<std::flush;
    }
}
