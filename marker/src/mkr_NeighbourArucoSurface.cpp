#include <mkr_ArucoSurface.h>
#include <mkr_Triangulator.h>
#include <mkr_ICP.h>
#include <iostream>

using namespace mkr;

void NeighbourReadTemplateXML(const cv::String xmlfile,
			 int& nPoses,
			 std::vector<cv::String>& lcam_fnames,
			 std::vector<cv::String>& rcam_fnames,
			 cv::String& dictionaryName,
			 int& tgtPose,
			 std::vector<std::pair<int,int>>& mergeSeq,
			 cv::String& debug_folder,
			 std::set<int>& bgMarkers)
{
      // Open the xml file to create this surface from
      cv::FileStorage fs(xmlfile, cv::FileStorage::READ);
      CV_Assert(fs.isOpened() && "ReadTemplateXML Could not open xml file to create surface");

      // Read the number of poses
      auto fn = fs["nPoses"];
      CV_Assert(!fn.empty() && "ReadTemplateXML Could not read number of poses");
      cv::read(fn, nPoses, -1);
      CV_Assert(nPoses>=1 && "ReadTemplateXML Unexpected number of poses");

      // Read string of left camera filenames
      lcam_fnames.clear();
      fn = fs["lcam_filenames"];
      CV_Assert(!fn.empty() && "ReadTemplateXML Could not read left camera filenames");
      cv::read(fn, lcam_fnames);
      CV_Assert(static_cast<int>(lcam_fnames.size())==nPoses &&
		"ReadTemplateXML Unexpected number of left camera files");
    
      // Read string of right camera filenames
      rcam_fnames.clear();
      fn = fs["rcam_filenames"];
      CV_Assert(!fn.empty() && "ReadTemplateXML Could not read right camera filenames");
      cv::read(fn, rcam_fnames);
      CV_Assert(static_cast<int>(rcam_fnames.size())==nPoses &&
		"ReadTemplateXML Unexpected number of right camera files");

      // Name of the dictionary used
      fn = fs["dictionary_name"];
      CV_Assert(!fn.empty() && "ReadTemplateXML Could not read dictionary name");
      cv::read(fn, dictionaryName, "");

      // Read the target pose
      fn = fs["target_pose"];
      CV_Assert(!fn.empty() && "ReadTemplateXML Could not read target pose");
      cv::read(fn, tgtPose, -1);
      CV_Assert(tgtPose>=0 && tgtPose<nPoses);

      // Read the merge sequence, if provided
      fn = fs["merge_sequence"];
      mergeSeq.clear();
      if(fn.empty()) // Create a default
	{
	  if(nPoses>1) // nPoses = 1 does not require a merge sequence
	    { std::cout<<"\nMerge sequence not provided. Generating a default sequence."<<std::flush;
	      for(int i=0; i<nPoses; ++i)
		if(i!=tgtPose)
		  mergeSeq.push_back( std::pair<int,int>(i,tgtPose) );
	    } }
      else
	{
	  cv::Mat_<int> ms;
	  cv::read(fn, ms);
	  CV_Assert(ms.cols==2);
	  mergeSeq.resize(ms.rows);
	  for(int i=0; i<ms.rows; ++i)
	    mergeSeq[i] = std::pair<int,int>(ms.at<int>(i,0), ms.at<int>(i,1));
	}
      // At most ids of background markers provided
      auto fnids = fs["bg_marker_ids"];
     bgMarkers.clear();
      if(!fnids.empty()) // IDs are provided
	{
	  cv::Mat_<int> bgids;
	  cv::read(fnids, bgids);
	  CV_Assert( (bgids.rows==1) &&  "ReadTemplateXML::Number of rows should be 1.");
	  for(int i=0; i<bgids.cols; ++i)
	    bgMarkers.insert( bgids.at<int>(0,i) );
	}
      
      // Done reading
      fs.release();
      return;
    }

void NeighbourConfigReadXML3D(const cv::String xmlfile, int& nPoses, std::vector<cv::String>&fnames, int& tgtPose, std::vector<std::pair<int,int>>& mergeSeq)
{
  // Open the xml file to create this surface from
  cv::FileStorage fs(xmlfile, cv::FileStorage::READ);
  CV_Assert(fs.isOpened() && "ReadTemplateXML Could not open xml file to create surface");

  // Read the number of poses
  auto fn = fs["nPoses"];
  CV_Assert(!fn.empty() && "ReadTemplateXML Could not read number of poses");
  cv::read(fn, nPoses, -1);
  CV_Assert(nPoses>=1 && "ReadTemplateXML Unexpected number of poses");

  // Read string of left camera filenames
  fnames.clear();
  fn = fs["filenames"];
  CV_Assert(!fn.empty() && "ReadTemplateXML Could not read left camera filenames");
  cv::read(fn, fnames);
  CV_Assert(static_cast<int>(fnames.size())==nPoses &&
	    "ReadTemplateXML Unexpected number of left camera files");

  // Read the target pose
  fn = fs["target_pose"];
  CV_Assert(!fn.empty() && "ReadTemplateXML Could not read target pose");
  cv::read(fn, tgtPose, -1);
  CV_Assert(tgtPose>=0 && tgtPose<nPoses);

  // Read the merge sequence, if provided
  fn = fs["merge_sequence"];
  mergeSeq.clear();
  if(fn.empty()) // Create a default
    {
      if(nPoses>1) // nPoses = 1 does not require a merge sequence
	{ std::cout<<"\nMerge sequence not provided. Generating a default sequence."<<std::flush;
	  for(int i=0; i<nPoses; ++i)
	    if(i!=tgtPose)
	      mergeSeq.push_back( std::pair<int,int>(i,tgtPose) );
	} }
  else
    {
      cv::Mat_<int> ms;
      cv::read(fn, ms);
      CV_Assert(ms.cols==2);
      mergeSeq.resize(ms.rows);
      for(int i=0; i<ms.rows; ++i)
	mergeSeq[i] = std::pair<int,int>(ms.at<int>(i,0), ms.at<int>(i,1));
    }
  // Done reading
  fs.release();
  return;
}

void NeighbourReadXML(const std::string filename, std::vector<int>& ids, std::vector<std::vector<cv::Point2d>>& uv)
{
  cv::FileStorage fs;
  fs.open(filename, cv::FileStorage::READ);
  assert(fs.isOpened());
  cv::Mat_<int> mat_ids;
  cv::Mat_<cv::Point2d> mat_uv;
  auto fn = fs["Marker_ids"];
  assert(!fn.empty());
  fn >> mat_ids;
  fn = fs["marker_coords"];
  assert(!fn.empty());
  fn >> mat_uv;
  fs.release();

  // Check
  const int nMarkers = mat_ids.rows;
  assert(mat_ids.cols==1 && mat_uv.rows==nMarkers);

  ids.resize(nMarkers);
  uv.resize(nMarkers, std::vector<cv::Point2d>(4));
  for(int m=0; m<nMarkers; ++m)
    {
      ids[m] = mat_ids.at<int>(m,0);
      for(int c=0; c<4; ++c)
      uv[m][c] = mat_uv.at<cv::Point2d>(m,c);
    }
  
  return;
}

 // Helper method to encapsulate calls to read a pose
    void NeighbourReadPose(const ArucoSet& arSet,
		  const Triangulator& tri,  
		  const cv::String lcamfile, const cv::String rcamfile,
			   const std::set<int> bgMarkers, ArucoPose& arPose, int p)
 
{
  std::vector<int> L_ids, R_ids;
  std::vector<std::vector<cv::Point2d>> L_uv, R_uv;
  NeighbourReadXML(lcamfile, L_ids, L_uv);
  NeighbourReadXML(rcamfile, R_ids, R_uv);

  std::vector<int> L_indx, R_indx;
  mkr::GetIndexCorrespondences(R_ids, L_ids, R_indx, L_indx);

  std::fstream pfile;
pfile.open("Dbg/Mkr"+std::to_string(p)+".xyz", std::ios::out);
 // Partition markers into surface and background 
      auto& surf = arPose.surf;
      auto& bg = arPose.bg;
      for(unsigned int i=0; i<R_indx.size(); ++i)
	{
	  // This marker
	  const auto& m = R_ids[R_indx[i]];
	  assert(R_ids[R_indx[i]] == L_ids[L_indx[i]] && "Corresponding IDs not same");
	  std::vector<cv::Point3d> Corner ;
	  for(int j=0; j<4; ++j)
	    {
	      cv::Point3d X;
	      tri.Triangulate<TriMethod::Sym>(L_uv[L_indx[i]][j], R_uv[R_indx[i]][j], X);
	      pfile<<X.x<<" "<<X.y<<" "<<X.z;
	      pfile<<"\n";
	      Corner.push_back(X);
	    }

	  // Is this a background marker
	  if(bgMarkers.find(m)!=bgMarkers.end())
	    {
	      bg.markerIDs.push_back(m);
	      bg.CornerPts.push_back( Corner );
	      ++bg.nMarkers;
	    }
	  else
	    {
	      surf.markerIDs.push_back(m);
	      surf.CornerPts.push_back( Corner );
	      ++surf.nMarkers;
	    }   
	}
      pfile.close();
        // -- done --
      return;
    }

//Read config poses from xmlfiles
void NeighbourConfigReadPoseXMl(const cv::String filename, ArucoPose& arPose,int p)
{
  std::vector<int> ids;
  std::vector<std::vector<cv::Point3d>> uv;
  
  cv::FileStorage fs;
  fs.open(filename, cv::FileStorage::READ);
  assert(fs.isOpened());
  cv::Mat_<int> mat_ids;
  cv::Mat_<cv::Point3d> mat_uv;
  auto fn = fs["surf_marker_ids"];
  assert(!fn.empty());
  fn >> mat_ids;
  fn = fs["surf_marker_coords"];
  assert(!fn.empty());
  fn >> mat_uv;
  fs.release();

  // Check
  const int nMarkers = mat_ids.rows;
  assert(mat_ids.cols==1 && mat_uv.rows==nMarkers);

  ids.resize(nMarkers);
  uv.resize(nMarkers, std::vector<cv::Point3d>(4));

  //write ids and coords to arucopose
  auto& surf = arPose.surf;
  for(int m=0; m<nMarkers; ++m)
    {
      ids[m] = mat_ids.at<int>(m,0);
      surf.markerIDs.push_back(ids[m]);
      std::vector<cv::Point3d> Corner ;
      for(int c=0; c<4; ++c)
	{
	  uv[m][c] = mat_uv.at<cv::Point3d>(m,c);
	  Corner.push_back(uv[m][c]);
	}
      surf.CornerPts.push_back( Corner );
       ++surf.nMarkers;
     }
  
  return;
}

//Print xmlfiles for common markers in poses
 void NeighbourPrintXML(const Triangulator& tri,  
			const cv::String lcamfile, const cv::String rcamfile, int p)

 {
   std::vector<int> L_ids, R_ids;
   std::vector<std::vector<cv::Point2d>> L_uv, R_uv;
   NeighbourReadXML(lcamfile, L_ids, L_uv);
   NeighbourReadXML(rcamfile, R_ids, R_uv);

   std::vector<int> L_indx, R_indx;
   mkr::GetIndexCorrespondences(R_ids, L_ids, R_indx, L_indx);

   std::fstream pfile;
   cv::String xmlfile = "Dbg/Mkr"+std::to_string(p)+".xml";

   // Open this file
    cv::FileStorage fs(xmlfile, cv::FileStorage::WRITE);
    CV_Assert(fs.isOpened() && "ArucoSurface::PrintXML- Could not open file");

   // Details of surface markers
   fs.writeComment((char*)"Number of surface markers", 0);
   assert(R_indx.size() == L_indx.size());
   const int& nSurfMarkers = R_indx.size();
   fs<<"num_surf_markers"<<nSurfMarkers;

   cv::Mat_<int> m_surfIDs(nSurfMarkers, 1);
   for(int i=0; i<nSurfMarkers; ++i)
     m_surfIDs(i,0) = R_ids[R_indx[i]];
   fs.writeComment((char*)"IDs of surface markers", 0);
   fs<<"surf_marker_ids"<<m_surfIDs;
    
   cv::Mat_<cv::Point3d> m_surfCoords(nSurfMarkers, 4);
   for(unsigned int i=0; i<R_indx.size(); ++i)
     {
       assert(R_ids[R_indx[i]] == L_ids[L_indx[i]] && "Corresponding IDs not same");
       for(int j=0; j<4; ++j)
	 {
	   cv::Point3d X;
	   tri.Triangulate<TriMethod::Sym>(L_uv[L_indx[i]][j], R_uv[R_indx[i]][j], X);
	   m_surfCoords(i,j) = X;
	 }
     }
    fs.writeComment((char*)"Coordinates of surface marker corners", 0);
    fs<<"surf_marker_coords"<<m_surfCoords;

    m_surfIDs.release();
    m_surfCoords.release();

   // -- done --
   return;
 }

//Constructor
ArucoSurface::ArucoSurface(const cv::String xmlfile,
			   const Triangulator& Tri)
			   
  {
    // Read XML file with details
    std::vector<cv::String> lcam_fnames, rcam_fnames; // XMLfiles from left/right cameras
    cv::String dictionaryName; // Dictionary of aruco markers
    std::set<int> bgMarkers; // IDs of background markers
    
  NeighbourReadTemplateXML(xmlfile, nPoses, lcam_fnames, rcam_fnames, dictionaryName, tgtPose, mergeSequence, debug_folder, bgMarkers);

    // Read all poses
    arPoses.resize(nPoses);
    ArucoSet arSet(dictionaryName);
    for(int p=0; p<nPoses; ++p)
      {
       	std::cout<<"\nReading pose: "<<p<<std::flush;
	NeighbourReadPose(arSet, Tri, lcam_fnames[p], rcam_fnames[p], bgMarkers, arPoses[p],p);
	NeighbourPrintXML(Tri, lcam_fnames[p], rcam_fnames[p], p);
      }
    
     // Note that merging is not done
    is_merged = false;
}

//Merge Configuration
ArucoSurface::ArucoSurface(const cv::String xmlfile)
{
  //Read XML files with details
  std::vector<cv::String> fnames; //Confg xmlfiles
  NeighbourConfigReadXML3D(xmlfile, nPoses, fnames, tgtPose, mergeSequence);

  // Read all poses
    arPoses.resize(nPoses);
    for(int p=0; p<nPoses; ++p)
      {
       	std::cout<<"\nReading pose: "<<p<<std::flush;
	NeighbourConfigReadPoseXMl(fnames[p],arPoses[p],p);
      }
    
     // Note that merging is not done
    is_merged = false;
}
