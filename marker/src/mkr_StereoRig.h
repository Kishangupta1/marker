#ifndef MKR_STEREO_RIG_H
#define MKR_STEREO_RIG_H

#include <mkr_Camera.h>
#include <mkr_ChessboardCorners.h>
#include <mkr_CharucoboardCorners.h>

namespace mkr
{
  class StereoRig
  {
    public:

    // Default constructor
    StereoRig();
    
    // Constructor, creates references to left and right camera
    // Performs the stereo calibration
    // Assumes that both cameras were calibrated simultaneously with the same chessboard
    // If update is true, the intrinsic camera parameters can be adjusted.
    StereoRig(const Camera& lcam,
	      const ChessboardCorners& lcorners,
	      const Camera& rcam,
	      const ChessboardCorners& rcorners);

    // Constructor, creates references to left and right camera
    // Performs the stereo calibration
    // Assumes that both cameras were calibrated simultaneously with the same charucoboard
    // If update is true, the intrinsic camera parameters can be adjusted.
    StereoRig(const Camera& lcam,
	      const CharucoboardCorners& lcorners,
	      const Camera& rcam,
	      const CharucoboardCorners& rcorners);

    // Constructor to read stereorig data from a file
    StereoRig(const cv::String xmlfile);
    
    // Copy constructor
    StereoRig(const StereoRig& obj);
    
     // Destructor
    ~StereoRig();

    // Returns the image size
    const cv::Size& GetImageSize() const;
    
    // Returns camera matrix
    const Camera& GetLeftCamera() const;

    const Camera& GetRightCamera() const;
    
    // Matrices for the stereo-system
    const cv::Mat& GetFundamentalMatrix() const;
    
    const cv::Mat& GetEssentialMatrix() const;
    
    void GetCoordinateMap(cv::Mat& R, cv::Mat& T) const;
    
    void GetStereoMatrices(cv::Mat& R, cv::Mat& T,
			   cv::Mat& E, cv::Mat& F) const;

    void GetProjectionMatrices(cv::Mat& lP, cv::Mat& rP) const;

    // Create from file
    void Create(const cv::String xmlfile);
    
    // Saves stereo-rig details to a file
    void Save(const cv::String xmlfile) const;

    // Perform a consistency check
    // Checks det(E) = det(F) = 0,
    // relationship between E and F,
    // relationship between E, R and T.
    bool ConsistencyTest(const double EPS) const;
    
  private:
    cv::Size _imgSize; // Image size
    Camera _lcam, _rcam; // Left and right cameras
    cv::Mat _rotMat, _transMat; // Rotation and translation from left to right camera
    cv::Mat _essentialMat; // essential matrix
    cv::Mat _fundamentalMat; // fundamental matrix
    cv::Mat _lP, _rP; // Projection matrices
  };
}

#endif