#ifndef MKR_ICP_H
#define MKR_ICP_H

#include <vector>
#include <opencv2/core.hpp>

namespace mkr
{
  //! Computes a rigid body transformation between between a given set of corresponding points
  //! Specifically, finds (R,t) such that A = R B + t
  //! Uses the SVD method
  void ComputeIsometry(const std::vector<cv::Point3d>& A,
		       const std::vector<cv::Point3d>& B,
		       double R[][3], double* t);

  //! Apply a rigid body transformation to a given point set
  template<class T>
  void ApplyIsometry(const double Rmat[][3], const double* tvec, T& pts);

    void GetIndexCorrespondences(const std::vector<int>& A, const std::vector<int>& B,
				 std::vector<int>& Aindx, std::vector<int>& Bindx);

}


#endif
