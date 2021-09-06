#include <mkr_ICP.h>
#include <map>

namespace mkr
{
  // Computes a rigid body transformation between between a given set of corresponding points
  // Specifically, finds (R,t) such that A = R B + t
  // Uses the SVD method
  void ComputeIsometry(const std::vector<cv::Point3d>& tgt, const std::vector<cv::Point3d>& src,
		       double R[][3], double* tvec)
  {
    const int nPoints = static_cast<int>(tgt.size());
    CV_Assert((nPoints>0 && static_cast<int>(src.size())==nPoints) &&
	      "ICP::ComputeIsometry- Unexpected number of points");
    
    // Centroid of each point set
    double tgt_centroid[] = {0.,0.,0.};
    double src_centroid[] = {0.,0.,0.};
    for(int i=0; i<nPoints; ++i)
      {
	tgt_centroid[0] += tgt[i].x; tgt_centroid[1] += tgt[i].y; tgt_centroid[2] += tgt[i].z;
	src_centroid[0] += src[i].x; src_centroid[1] += src[i].y; src_centroid[2] += src[i].z;
      }
    for(int j=0; j<3; ++j)
      {
	tgt_centroid[j] /= static_cast<double>(nPoints);
	src_centroid[j] /= static_cast<double>(nPoints);
      }
    
    // Compute the covariance matrix
    cv::Mat_<double> covMat(3,3);
    covMat = cv::Mat_<double>::zeros(3,3);
    for(int p=0; p<nPoints; ++p)
      {
	const double a[] = {tgt[p].x, tgt[p].y, tgt[p].z};
	const double b[] = {src[p].x, src[p].y, src[p].z};
	for(int i=0; i<3; ++i)
	  for(int j=0; j<3; ++j)
	    covMat(i,j) += (b[i]-src_centroid[i])*(a[j]-tgt_centroid[j]);
      }
    
    // SVD of the covariance matrix
    cv::Mat_<double> sVals, Umat, Vtmat;
    cv::SVD::compute(covMat, sVals, Umat, Vtmat);
    double det = cv::determinant(Umat*Vtmat);
    cv::Mat_<double> resetVals = cv::Mat_<double>::eye(3,3);
    if(det<0.) resetVals(2,2) = -1.;
    cv::Mat_<double> RMat(3,3);
    RMat = Vtmat.t()*resetVals*Umat.t();
    CV_Assert(std::abs(cv::determinant(RMat)-1.)<1.e-6 &&
	      "ICP::ComputeIsometry- inconsistent determinant for rotation.\n");

    // Copy the rotation matrix
    for(int i=0; i<3; ++i)
      for(int j=0; j<3; ++j)
	R[i][j] = RMat(i,j);
     
    // Translation vector
    for(int i=0; i<3; ++i)
      {
	tvec[i] = tgt_centroid[i];
	for(int j=0; j<3; ++j)
	  tvec[i] -= R[i][j]*src_centroid[j];
      }
    return;
  }


  // Helper method to apply a rigid body transformation to a given point set
  template<>
  void ApplyIsometry(const double Rmat[][3], const double* tvec,
		     std::vector<std::vector<cv::Point3d>>& pts)
  {
    const int nPts = static_cast<int>(pts.size());
    CV_Assert(nPts>0 && "ICP::ApplyIsometry- unxpected number of points to transform");
    double X[3], Y[3]; // Y = RX+t
    for(int c=0; c<nPts; ++c)
      for(int j=0; j<4; ++j)
	{
	  X[0] = pts[c][j].x;
	  X[1] = pts[c][j].y;
	  X[2] = pts[c][j].z; 
	  for(int k=0; k<3; ++k)
	    {
	      Y[k] = tvec[k];
	      for(int L=0; L<3; ++L)
		Y[k] += Rmat[k][L]*X[L];
	    }
	  pts[c][j].x = Y[0];
	  pts[c][j].y = Y[1];
	  pts[c][j].z = Y[2];
	}
    return;
  }


  // Helper method to apply a rigid body transformation to a given point set
  template<>
  void ApplyIsometry(const double Rmat[][3], const double* tvec,
		     std::vector<cv::Point3d>& pts)
  {
    const int nPts = static_cast<int>(pts.size());
    CV_Assert(nPts>0 && "ICP::ApplyIsometry- unxpected number of points to transform");
    double X[3], Y[3]; // Y = RX+t
    for(int c=0; c<nPts; ++c)
      {
	X[0] = pts[c].x;
	X[1] = pts[c].y;
	X[2] = pts[c].z;
	for(int k=0; k<3; ++k)
	  {
	    Y[k] = tvec[k];
	    for(int L=0; L<3; ++L)
	      Y[k] += Rmat[k][L]*X[L];
	  }

	pts[c].x = Y[0];
	pts[c].y = Y[1];
	pts[c].z = Y[2];
      }
    return;
  }

      // Helper method to identify common markers
    // A[Aindx[i]] = B[Bindx[i]].
    void GetIndexCorrespondences(const std::vector<int>& A, const std::vector<int>& B,
				  std::vector<int>& Aindx, std::vector<int>& Bindx)
    {
      Aindx.clear(); Bindx.clear();
      const int nAmarkers = static_cast<int>(A.size());
      const int nBmarkers = static_cast<int>(B.size());

      //CV_Assert(nAmarkers+nBmarkers>0 && "GetMarkerCorrespondence- Unexpected number of markers");
      if(nAmarkers==0 || nBmarkers==0) return;
    
      // Create mapping from markers->index for list B
      std::map<int, int> Bmarker2indx;
      for(int i=0; i<nBmarkers; ++i)
	Bmarker2indx[B[i]] = i;

      // Detect coincident markers
      for(int i=0; i<nAmarkers; ++i)
	{
	  int  mnum = A[i];
	  // Does 'mnum' appear in list B?
	  auto it = Bmarker2indx.find(mnum);
	  if(it!=Bmarker2indx.end())
	    {
	      Aindx.push_back( i );
	      Bindx.push_back( it->second );
	    }
	}

      // Sanity check
      const int nCommon = static_cast<int>(Aindx.size());
      assert(static_cast<int>(Bindx.size())==nCommon &&
	     "GetMarkerCorrespondences- Unexpected scenario");
      
      for(int i=0; i<nCommon; ++i)	
	assert(A[Aindx[i]]==B[Bindx[i]] &&
	       "GetMarkerCorrespondences- Unexpected scenario");

      // Done
      return;
    }

}