#include <mkr_ArucoSurface.h>
#include <mkr_Triangulator.h>
#include <mkr_ICP.h>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

using namespace mkr;

void PrintXML(const cv::String xmlfile, std::map<int,std::vector<cv::Point2d>> CornerMap);

void ReadTemplateXML(const cv::String xmlfile,
			 int& nPoses,
			 std::vector<cv::String>& lcam_fnames,
		     std::vector<cv::String>& rcam_fnames)
			 
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

 
}

int main()
{
  std::vector<cv::String> lcam_fnames, rcam_fnames; // Images from left/right cameras
  int nPoses; //!< Number of poses
  ReadTemplateXML("Img.xml", nPoses, lcam_fnames, rcam_fnames);

  int b = 0;  //for rcam poses
  cv::Mat img;
  
  for(int p=0; p<nPoses*2; ++p)
    {
  ArucoSet aset2("DICT_4X4_50");
  ArucoSetImage R;
  // aset2.Detect("../lcam/IMG_0011.JPG",R);

  if(p < nPoses)
    {
   aset2.Detect(lcam_fnames[p],R);
   img = cv::imread(lcam_fnames[p],0);
   std::cout<<"\n"<<"Markers Detected in lcam "<<std::to_string(p)<<"#:"<<R.nMarkers<<std::flush<<"\n";
  aset2.Visualize(R,"Visualize/ldet"+std::to_string(p)+".jpg");
    }
  else
    {
      
   aset2.Detect(rcam_fnames[b],R);
   img = cv::imread(rcam_fnames[b],0);
   std::cout<<"\n"<<"Markers Detected in rcam "<<std::to_string(b)<<"#:"<<R.nMarkers<<std::flush<<"\n";
  aset2.Visualize(R,"Visualize/rdet"+std::to_string(b)+".jpg");
    }
  
  //Read IDs and neighbor from Left img
  std::map<int,std::array<int,5>> Lcor;
  std::fstream pfile;
  pfile.open((char*)"CommonNumWord.dat",std::ios::in);
  assert(pfile.good());
  double xyz[6];
  pfile >> xyz[0];
  while(pfile.good())
    {
      pfile >> xyz[1];  pfile >> xyz[2];  pfile >> xyz[3];
      pfile >> xyz[4];  pfile >> xyz[5];
      std::array<int,5> temp;
      for(int i=0; i<5; ++i) temp[i] = xyz[i+1];
      Lcor.insert(std::pair<int,std::array<int,5>>(xyz[0],temp));
      pfile>>xyz[0];
    }
  pfile.close();

  //Store pixel values of all markers and corresponding IDs
  
  typedef boost::geometry::model::d2::point_xy<double> point_type;
  typedef boost::geometry::model::polygon<point_type> polygon_type;
std::multimap<int,polygon_type> id;
 
  for(int i=0; i<R.markerIDs.size(); ++i)
  //  for(int i=3; i<4; ++i)
  {
      //bounding box of mkr
      int a = static_cast<int>(R.markerCorners[i][0].x) ; int b = static_cast<int>(R.markerCorners[i][0].y) ;
      int c = static_cast<int>(R.markerCorners[i][1].x) ; int d = static_cast<int>(R.markerCorners[i][1].y) ;
      int e = static_cast<int>(R.markerCorners[i][2].x) ; int f = static_cast<int>(R.markerCorners[i][2].y) ;
      int g = static_cast<int>(R.markerCorners[i][3].x) ; int h = static_cast<int>(R.markerCorners[i][3].y) ;
      polygon_type poly;
      poly.outer().push_back(point_type(a,b)); 
      poly.outer().push_back(point_type(c,d));
      poly.outer().push_back(point_type(e,f));
      poly.outer().push_back(point_type(g,h));
      poly.outer().push_back(point_type(a,b));
      
     id.insert(std::pair<int,polygon_type>(R.markerIDs[i],poly));
    }
  
 
  //Find words of all markers
  std::map<int,std::array<int,5>> WordMap; // contain ID and corresponding word
  std::map<int,std::vector<cv::Point2d>> CornerMap; //contain ID and corresponding corners
  int M=0;  //Given commom IDs to marker in lcam and rcam

   for(int m=0; m<R.markerIDs.size(); ++m)
  //  for(int m=7; m<8; ++m)
    {
      int word[5] = {-1,-2,-3,-4,-5};
      word[0] = R.markerIDs[m];

      //Get mid-point of marker
      cv::Point2d center = {0.,0.};
      for(int j=0; j<4; ++j)
	{
	  center.x += R.markerCorners[m][j].x/4; center.y += R.markerCorners[m][j].y/4;
	}
       
      for(int c=0; c<4; ++c)
      //  for(int c=3; c<4; ++c)
   	{
	  //  word[c+1] = -5;// -(c+2);
	  cv::Point2d x1 = R.markerCorners[m][c];
	  cv::Point2d x2;
	  if(c == 3) x2 = R.markerCorners[m][c-c];
	  else x2 = R.markerCorners[m][c+1];
      
	  double x0 = (x1.x+x2.x)/2, y0 = (x1.y+x2.y)/2; //MidPoint of line joining 2 corners
	  //  std::cout<<"\nMp:"<<x0<<" "<<y0;
	  
	  //Check mid-point lies on which side
	  int sign1 = (center.y-x1.y) * (x2.x-x1.x) - (center.x - x1.x) * (x2.y-x1.y);
	  
	  //Slope stuff
	  double s = ((x1.y-x2.y)/(x1.x-x2.x)); 
	  double s0 = -1/s;  //std::cout<<"\n"<<m0;
	  double th = (std::atan(s0));
	  double u = std::cos(th), v = std::sin(th); //std::cout<<"\n\n"<<th<<" "<<u<<" "<<v<<"\n";

	  std::map<int,polygon_type>::iterator it;
	  bool flag = false;

	  //check point for search direction
	  int pxi = x0 + 5*u, pyi = y0 + 5*v;  //std::cout<<"\npcordsi:"<<pxi<<" "<<pyi;
	  
	   for(int k= 0; k<100;  )
	    {
	      //Change search direction if necessary
	       int sign2 = (pyi-x1.y) * (x2.x-x1.x) - (pxi - x1.x) * (x2.y-x1.y);
	       // std::cout<<"\nsign2:"<<sign2;
	       if((sign1 > 0 && sign2 >0) || (sign1 < 0 && sign2 < 0)) k -= 10;
	       else k += 10;
	      int px = x0 + k*u, py = y0 + k*v; // std::cout<<"\npcords:"<<px<<" "<<py;
	      cv::Point2d query = {px,py};
	      point_type p(query.x, query.y);
	      for(it = id.begin(); it!=id.end(); ++it)
		{
		  //for(int i=0; i<it->second.size(); ++i)
		  //  {
		  if((boost::geometry::covered_by(p, it->second)))
		    //if(query == it->second[i])
			{
			  
			  word[c+1]=(it -> first);
			  flag = true; break;
			}
		      //  }
		  if(flag == true) break;
		}
	      if(flag == true) break;
	      if(k == 100) break;
	      if(k <- 100) break;
	    } //  std::cout<<"\n"<<flag<<" "<<word[c+1];
	}
      //  exit(1);
     
      const int len = sizeof(word)/sizeof(word[0]);
      std::array<int,len> temp;
      for(int i=0; i<len; ++i) temp[i] = word[i];
      // WordMap.insert(std::pair<int,std::array<int,5>>(M,temp));
    
      std::vector<cv::Point2d> Corner ;
      for(int j=0; j<4; ++j) Corner.push_back (R.markerCorners[m][j]); 
     
      std::map<int,std::array<int,5>>::iterator it;
      std::map<int,std::array<double,2>>::iterator it1;

      //Loop through lcam IDs and word and write marker with same word in rcam
      for(it=Lcor.begin(); it!=Lcor.end(); ++it)
	{
	  if(temp == it->second){
	    WordMap.insert(std::pair<int,std::array<int,5>>(it->first,temp));
	    CornerMap.insert(std::pair<int,std::vector<cv::Point2d>>(it->first,Corner));

	    //Print IDs on img
	    if(p<nPoses)
	      {
		cv::Point2d CornerAvg;
		for(int j=0; j<4; ++j){
		  CornerAvg.x += R.markerCorners[m][j].x/4;
	       	  CornerAvg.y += R.markerCorners[m][j].y/4;}
			cv::putText(img,std::to_string(it->first),CornerAvg,cv::FONT_HERSHEY_SIMPLEX,4,cv::Scalar(255),6);
			cv::imwrite("DbgImg/lcam"+std::to_string(p)+".jpg", img);
	      }
	    else
	      {
		cv::Point2d CornerAvg;
		for(int j=0; j<4; ++j){
		  CornerAvg.x += R.markerCorners[m][j].x/4;
		  CornerAvg.y += R.markerCorners[m][j].y/4;}
			cv::putText(img,std::to_string(it->first),CornerAvg,cv::FONT_HERSHEY_SIMPLEX,4,cv::Scalar(255),6);
			cv::imwrite("DbgImg/rcam"+std::to_string(b)+".jpg", img);
	      }
	  }
	}
      
      ++M;
    }

  // check all Words are unique
   cv::Mat img;
   // img = cv::imread("rcam.JPG");
  std::map<int,std::array<int,5>>::iterator itr;
    std::map<int,std::vector<cv::Point2d>>::iterator iter;
  for(auto& it:WordMap)
    {
      int c = 0;
      std::array<int,5> a = it.second;   
      for(itr = WordMap.begin(); itr!=WordMap.end(); ++itr)
	 
	if(a == itr->second) ++c;
      assert(c==1 && "Repeated words");
      if(c!=1) {std::cout<<it.first<<"\n";
	 iter = CornerMap.find(it.first);
	 	 if(iter!=CornerMap.end()){
		   std::cout<<iter->first<<" "<<iter->second<<"\n";
		   //	 cv::circle(img, iter->second[0], 20, cv::Scalar(255), -1);
	 }
       }
      // cv::imwrite("dbgR.jpg",img);
    }
    
  //write IDs and neighbor to file
   
    pfile.open((char*)"NumWordLcam2.dat",std::ios::out);
  std::map<int,std::array<int,5>>::iterator it;
  for(it = WordMap.begin(); it!=WordMap.end(); ++it){
    pfile<<it->first<<" ";
    for(int i=0; i<5; ++i) pfile<<it->second[i]<<" "; pfile<<"\n";}
  pfile.close();

  //write IDs and coords to file
  pfile.open((char*)"Lc2.dat",std::ios::out);
  std::map<int,std::vector<cv::Point2d>>::iterator itra;
  for(itra = CornerMap.begin(); itra!=CornerMap.end(); ++itra){
    pfile<<itra->first<<" ";
    pfile<<itra->second<<" "; pfile<<"\n";}
  pfile.close();

  //write IDs and midpoint coords to xml file
  if(p < nPoses)
    PrintXML("Data/XMLfiles/LC"+std::to_string(p)+".xml", CornerMap);
  else
    {
    PrintXML("Data/XMLfiles/RC"+std::to_string(b)+".xml", CornerMap);
  ++b; //increment for rcam poses
    }
}
}

void PrintXML(const cv::String xmlfile, std::map<int,std::vector<cv::Point2d>> CornerMap)
{
  //Open this file
  cv::FileStorage fs(xmlfile, cv::FileStorage::WRITE);
  CV_Assert(fs.isOpened() && "Can't open file");

  //Details of markers
  fs.writeComment((char*)"Number of markers", 0);
  const int& nMarkers = CornerMap.size();
  fs<<"num_markers"<<nMarkers;

  //Marker IDs
  cv::Mat_<int> m_markerIDs(nMarkers, 1);
  //  std::map<int,std::array<double,2>>::iterator itra;
  std::map<int,std::vector<cv::Point2d>>::iterator itra;
  int i = 0;
  for(itra = CornerMap.begin(); itra!=CornerMap.end(); ++itra)
    {
      m_markerIDs(i,0) = itra->first; ++i;
    }
  fs.writeComment((char*)"IDs of markers", 0);
  fs<<"Marker_ids"<<m_markerIDs;

  //Marker Coords
  cv::Mat_<cv::Point2d> m_markerCoords(nMarkers,4);
  i = 0;
  for(itra = CornerMap.begin(); itra!=CornerMap.end(); ++itra)
    {
      for(int j=0; j<4; ++j)
	m_markerCoords(i,j) = itra->second[j];
      ++i;
    }
  fs.writeComment((char*)"Coords of markers Midpoint", 0);
  fs<<"marker_coords"<<m_markerCoords;

  m_markerIDs.release();
  m_markerCoords.release();
}
