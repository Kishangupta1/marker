#include <CornerMarkerDictionary.h>
#include <limits>
#include <aruco.hpp>
#include <typeinfo>

namespace mkr
{
  // Constructor
  CornerMarkerDictionary::
  CornerMarkerDictionary(const int nrows, const int ncols)
    :nRows(nrows), nCols(ncols),
     MarkerNums(nrows, std::vector<int>(ncols, -1)),
     MaxColor(1), ColorNum(0), ChkMaxColor(0), dictionary({})
  {
    std::cout<<"\nInitializing dictionary "<<std::flush;
    InitializeEightColoring();

    std::cout<<"\nMax colors used: "<<GetNumColors()<<std::flush;

    BoundaryMarkers();
  }

  // Eight coloring for markers
  void CornerMarkerDictionary::InitializeEightColoring()
  {
    // The color of a marker does not coincide with any of its 8 neighbors
    ColorNum = 0;  //Value is not 0 only if needed to make more than one dictionaries
    ChkMaxColor = MaxColor;
    
    //Random number generator
    // std::random_device rd;
    //std::mt19937 gen(rd());
     std::default_random_engine e1; //alwz gen same random no.
    std::uniform_int_distribution<> dis(0,MaxColor);
    for(int i=0; i<nRows; ++i)
      for(int j=0; j<nCols; ++j)
	{
	  // 	  MarkerNums[i][j] = dis(gen);
	 	  MarkerNums[i][j] = dis(e1);
	}

   std::cout<<"\nUniquating dictionary:"<<std::flush;
   UniquateMarkers();
    return;
  }

  // Make marker identities unique
  //Numerbing used:
  //       1
  //  -------------
  //  4  | 0  | 2
  // -------------
  //       3            
	
  void CornerMarkerDictionary::UniquateMarkers()
  {
    // Run over the 2D grid.
    // At the corner with index (i,j), form the 4-letter word to its North-West
    
    WORD NWword;
    for(int i=1; i<nRows; ++i)
      {
	int chk=0;
	if(i == nRows) break;
	for(int j=1; j<=nCols; ++j)
	  {   
	    NWword[0] = MarkerNums[i-1][j-1]; 
	    if((j-2) < 0) NWword[4] = -5;
	    else NWword[4] = MarkerNums[i-1][j-2];
	    if(j == nCols) NWword[2] = -3;
	    else NWword[2] = MarkerNums[i-1][j];  
	    if( (i-2) < 0) NWword[1] = -2;
	    else NWword[1] = MarkerNums[i-2][j-1];
		
	    // Assign a color to (i-1,j-1)
	    int mycolor = -1;
	
	    const int currcolor = MarkerNums[i][j-1];
	      
	    for(int L=0; L<=(MaxColor - ColorNum) && mycolor==-1; ++L)
	      {
		int k = (currcolor+L)%(MaxColor+1);
		//  if(k<ColorNum) k = ColorNum + L ;
		    
		// Try assigning color 'k' to (i-1,j)
		NWword[3] = k;
		  
		// Is this word in the dictionary
		if(!IsAWord(NWword))
		  mycolor = k;
	      }

	    // If (i-1,j) has not been assigned a color, add a new color
	    if(mycolor==-1)
	      { ++MaxColor; mycolor = MaxColor; NWword[3] = mycolor; }

	    // Upate color for (i-1,j)
	    MarkerNums[i][j-1] = mycolor; //Update south marker so it doesnot affect previous words in dictionary

	    // Update the dictionary
	    // InsertWord(NWword);
	    dictionary.insert(NWword);
 
	    if(ChkMaxColor != MaxColor)
	      {
		dictionary.clear();
		++chk;
		InitializeEightColoring(); break;
	      }
	  }if(chk!= 0) break;
      }
    return;
  }

  //check for unique boundary mkrs
  void CornerMarkerDictionary::BoundaryMarkers()
  {
    //change boundary mkrs
    for(int i=0; i<nRows; ++i)
      for(int j=0; j<nCols; ++j)
	{
	  if(j==0 || j==(nCols-1) || i==0 || i==(nRows-1))
	     MarkerNums[i][j] = MarkerNums[i][j] + (MaxColor+1) ;
	}
  }
  
  //Print words of dictionary
  void CornerMarkerDictionary::
  PrintDictionary(const char* filename) const
  {
    std::fstream outfile;
    outfile.open(filename, std::ios::out);
      assert(outfile.good() &&
	     "CornerMarkerDictionary::PrintMarkerNumbers- Could not open file");
      outfile<<nRows<<"\t"<<nCols;
      for(auto& it:dictionary)
	outfile <<"\n"<<it[0]<<" "<< it[1]<<" "<< it[2]<<" "<< it[3];
      outfile.flush();
      outfile.close();
      return;
  }

  // Print marker numbers
  // Format: Line 1 rows, cols, Then: (i,j,num)
  void CornerMarkerDictionary::
  PrintMarkerNumbers(const char* filename) const
  {
    std::fstream outfile;
    outfile.open(filename, std::ios::out);
      assert(outfile.good() &&
	     "CornerMarkerDictionary::PrintMarkerNumbers- Could not open file");
      outfile<<nRows<<"\t"<<nCols;
      for(int i=0; i<nRows; ++i)
	for(int j=0; j<nCols; ++j)
	  outfile<<"\n"<<i<<" "<<j<<" "<<MarkerNums[i][j];
      outfile.flush();
      outfile.close();
      return;
  }
  
  // Print marker numbers in matrix
   void CornerMarkerDictionary::
   PrintMarkerMatrix(const char* filename, const int nPixelsPerLen, const int nBufferPixels) const
  //PrintMarkerMatrix( std::string filename, const int nPixelsPerLen, const int nBufferPixels) const
  {
    cv::Mat_<uchar> img(nRows*nPixelsPerLen+(nRows+1)*nBufferPixels,
    		nCols*nPixelsPerLen+(nCols+1)*nBufferPixels,static_cast<uchar>(255));

    //cv::Mat_<uchar> img(2974,510);
    int kstart, kend, Lstart, Lend;
    for(int i=0; i<nRows; ++i)
      for(int j=0; j<nCols; ++j)
	{
	  // Number of this marker
	  const int& num = MarkerNums[i][j];
	  //int fmkrs = 2*6; // mkrs used on front side of mobius *2 for bound mkrs
	  // const int& num = MarkerNums[i][j] + fmkrs;  
	  
     	  // Paint this marker
	  kstart = i*nPixelsPerLen + (i+1)*nBufferPixels;
	  kend = kstart + nPixelsPerLen;
	  Lstart = j*nPixelsPerLen + (j+1)*nBufferPixels;
	  Lend = Lstart + nPixelsPerLen;
	  int x = static_cast<int>(nPixelsPerLen*0.8);
	  int y = static_cast<int>(nPixelsPerLen/5);
	  int size = static_cast<int>(nPixelsPerLen/33);
	  
	  cv::rectangle(img, cv::Point(Lstart,kstart), cv::Point(Lend,kend), cv::Scalar(0),-1);
	  cv::putText(img,std::to_string(num),cv::Point(Lstart+y,kstart+x),cv::FONT_HERSHEY_SIMPLEX,size,cv::Scalar(255),5);
	}
	  cv::imwrite(filename,img);
	  return;
	
  }
    // Print markers pattern
    void CornerMarkerDictionary::CreateMarkerPattern(const char* filename, const int nPixelsPerLen,
 						     const int nBufferPixels) const
    {
       cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
      //cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_16h5);
      // Output image
       cv::Mat_<uchar> img(nRows*nPixelsPerLen+(nRows+1)*nBufferPixels,
       nCols*nPixelsPerLen+(nCols+1)*nBufferPixels,static_cast<uchar>(255));

       //cv::Mat_<uchar> img(10250, 1840, static_cast<uchar>(255));
      int kstart, kend, Lstart, Lend;
      for(int i=0; i<nRows; ++i)
	for(int j=0; j<nCols; ++j)
	//	for(int j=14; j<28; ++j)
	  {
	    // Number of this marker
	     const int& num = MarkerNums[i][j];
	    //int fmkrs = 2*6; // mkrs used on front side of mobius *2 for bound mkrs
	    //const int& num = MarkerNums[i][j] + fmkrs;  
	   
	    // Paint this marker
	    kstart = i*nPixelsPerLen + (i+1)*nBufferPixels;
	    kend = kstart + nPixelsPerLen;
	     Lstart = j*nPixelsPerLen + (j+1)*nBufferPixels;
	    Lend = Lstart + nPixelsPerLen;
	    for(int k=kstart; k<kend; ++k)
	      for(int L=Lstart; L<Lend; ++L)
		{
		  CV_Assert(k>=0 && k<img.rows);
		  CV_Assert(L>=0 && L<img.cols);
		  cv::Mat marker;
		  dictionary->drawMarker( num, nPixelsPerLen, marker, 1);
		  img.at<uchar>(k,L) = marker.at<uchar>(k-kstart, L-Lstart);
		
		}
	  }
      cv::imwrite(filename, img); 
      return;
      }
  
}

