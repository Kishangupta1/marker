#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <aruco.hpp>
#include <fstream>
#include <string>
using namespace std;
using namespace cv;

void createArucoMarkers();

int main()
{
  createArucoMarkers();
}

void createArucoMarkers()
{
  const int nRows = 5; // Number of rows of markers
  const int nCols = 4; // Number of cols of markers
  const int prod = nRows*nCols;
  const int nPixelsPerMarker = 250; // #pixels per marker side
  const int nbufferPixels = 30; // #buffer pixels
  // TODO make marker size a multiple of the number of bits+border
  
  // Size of the image
  const int Ny = 1*(nCols*nPixelsPerMarker+(nCols+1)*nbufferPixels);
  const int Nx = 1*(nRows*nPixelsPerMarker+(nRows+1)*nbufferPixels);

  // Output image
  cv::Mat_<uchar> img(Nx,Ny,static_cast<uchar>(255));  //grayscale
    //  cv::Mat_<uchar> img(5100,7600,static_cast<uchar>(255));
  //cv::Mat img(Nx,Ny, CV_8UC3 ,cv::Scalar(255,255,0));
    std::cout<<Nx<<" "<<Ny<<"\n";
  // Create a dictionary
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);  

  int count = 50;
  
  // Create markers and paint the image
  for(int i=0; i<nRows; ++i)    
    for(int j=0; j<nCols; ++j)
      {
	// Generate the marker (i,j)
       	int mnum = nCols*i+j;
	//int mnum = count;
	//	std::cout<<mnum<<" ";
	cv::Mat marker;
	dictionary->drawMarker(mnum, nPixelsPerMarker, marker, 1);
	
	// Paint the image with the marker
	int kstart = i*nPixelsPerMarker+(i+1)*nbufferPixels;
	int Lstart = j*nPixelsPerMarker+(j+1)*nbufferPixels;
	for(int k=kstart; k<kstart+nPixelsPerMarker; ++k)
	  for(int L=Lstart; L<Lstart+nPixelsPerMarker; ++L)
	    img.at<uchar>(k,L) = marker.at<uchar>(k-kstart, L-Lstart);
	   
	++count;
      }
   
  cv::imwrite("1.jpg", img);
  std::cout<<"\n\nTotal number of marker is "<<prod<<"\n\n"<<std::flush;
  
}
