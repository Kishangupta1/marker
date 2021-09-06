#ifndef MKR_CORNER_MARKER_DICTIONARY_H
#define MKR_CORNER_MARKER_DICTIONARY_H

#include <set>
#include <array>
#include <vector>
#include <map>
#include <random>
#include <iostream>
#include <cassert>
#include <fstream>
#include <algorithm>
#include <mkr_ArucoSet.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace mkr
{
  class CornerMarkerDictionary
  {
    // using WORD = std::array<int, 4>; // Defining a word as a list of 4 marker numbers
     using WORD = std::array<int, 5>; // Defining a word as a list of 5 marker numbers
    
  public:
    // Constructor
    CornerMarkerDictionary(const int nrows, const int ncols);
		       
    // Destructor
    inline virtual ~CornerMarkerDictionary() {}

    // Disable copy and assignment
    CornerMarkerDictionary(const CornerMarkerDictionary&) = delete;
    CornerMarkerDictionary& operator=(const CornerMarkerDictionary&) = delete;

    // Returns the number of colors used
    inline int GetNumColors() const
    { return MaxColor+1; }

    // Returns the number of rows of markers
    inline int GetNumMarkerRows()  const
    { return nRows; }

    // Print markers pattern
    void CreateMarkerPattern(const char* filename, const int nPixelsPerLen,
			     const int nBufferPixels) const;

    // Returns the number of cols of markers
    inline int GetNumMarkerCols() const
      { return nCols; }

    //Print words of dictionary
    void PrintDictionary(const char* filename) const;
    
    // Print marker numbers to file
    // Format: Line 1 rows, cols, Then: (i,j,num)
    void PrintMarkerNumbers(const char* filename) const;

    // Print marker numbers in matrix
    void PrintMarkerMatrix(const char* filename, const int nPixelsPerLen,
    //void PrintMarkerMatrix(std::string filename, const int nPixelsPerLen,
			     const int nBufferPixels) const;
      
  protected:
    // Initialize markers with 8-coloring
    void InitializeEightColoring();
    
    // Make marker identities unique
    void UniquateMarkers();

    // Make boundary marker identities unique
    void BoundaryMarkers();

    // Defines the set of cyclic & acyclic variants of a word
    static void GetAnagrams(const WORD& word,  std::set<WORD>& anagrams);

    //! Defines the word at a given corner
    void GetWordAtCorner(const int i, const int j, WORD& word) const;
    
    // Insert a word and its cyclic orderings into the dictionary
    // To be implemented by the derived classes
    void InsertWord(const WORD& word);

    // Check if a word exists in the dictionary
    inline bool IsAWord(const WORD& word) const
    { return dictionary.find(word)!=dictionary.end(); }

    // members
    const int nRows; // Number of rows of markers
    const int nCols; // Number of cols of markers
    std::vector<std::vector<int>> MarkerNums; // Marker numbering
    int MaxColor; // max color used
    int ColorNum; //starting numbering of color
    int ChkMaxColor; //chk max color reqd to form the mkrs
    std::set<WORD> dictionary; // List of all existing words
   
   
  };

}

#endif
