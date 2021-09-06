#include <opencv2/core.hpp>
#include <vector>
#include <iostream>
#include <fstream>
#include <cassert>
#include <array>
#include <random>
#include <gsl/gsl_linalg.h>

const int points = 2120; // no. of points in def confg
const int intpts = 3260; //no. of pts for interpolation
const int nc = 193; //no of unique random center
void neuron(std::vector<std::array<double,3>>& pt, double* wt, double c[][2], double* sig);

int main()
{
  // Read deformed coordinate file
  std::string filename = "phi_x.xyz";
  std::ifstream infile;
  infile.open(filename);
  assert(infile.good() && "the input file is not good.");

  std::vector<std::array<double,3>> in_val({});
  double vec[3];
  while(infile.good())
    {
      infile >> vec[0];
      infile >> vec[1];
      infile >> vec[2];
      in_val.push_back(std::array<double,3>({vec[0],vec[1],vec[2]})); 
    }
  infile.close();
  //   std::cout<<"\n"<<in_val[2][2];

   //Read ref upsampled points to be interpolated
   std::string filename2 = "refUp.xyz";
  std::ifstream infile2;
  infile2.open(filename2);
  assert(infile2.good() && "the input file is not good.");

  std::vector<std::array<double,2>> val({});
  double vec2[2];
  while(infile2.good())
    {
      infile2 >> vec2[0];
      infile2 >> vec2[1];
      //infile2 >> vec2[2];
      val.push_back(std::array<double,2>({vec2[0],vec2[1]})); 
    }
  infile2.close();
  //  std::cout<<"\n"<<val[2][0];

  //Read kmeans center file
   std::string filename3 = "kmeansCt.xyz";
  std::ifstream infile3;
  infile3.open(filename3);
  assert(infile3.good() && "the input file is not good.");
  double c[nc][2];
  for(int i=0; i<nc; ++i)
    for(int j=0; j<2; ++j)
      infile3 >> c[i][j];

   //Compute distance between all centers
   std::vector<double> dvec;
   for(int i=0; i<nc; ++i)
     for(int j=0; j<nc; ++j)
       {
         double di = std::sqrt(pow((c[i][0]-c[j][0]),2) + pow((c[i][1]-c[j][1]),2) );
	 dvec.push_back(di);
       }

    //select maxm center distance
   double dis_c = *max_element(dvec.begin(), dvec.end());
   double sig = 0.5 / (pow((dis_c / (std::sqrt(nc))),2));
  double wt[nc+1];
  std::cout<<"\n"<<sig;
  neuron(in_val, wt, c, &sig);
  std::cout<<"\nCoefficients: "<<wt[10]<<" "<<wt[1]<<" "<<wt[nc]<<"\n";
  
  for(int i=0; i<nc+1; ++i)  
    std::cout<<wt[i]<<" ";

   //Interpolation of upsampled ref confg
   double z[intpts];
   for(int i=0; i<intpts; ++i)
     for(int j=0;j<nc; ++j)
         z[i] += (wt[j] * exp(-(sig) * (pow((val[i][0]-c[j][0]),2) + pow((val[i][1]-c[j][1]),2)))) + (wt[nc]/nc);
   
   //Write interpolated value to file
   std::fstream file;
   file.open((char*)"IntpolVal_x.xyz", std::ios::out);
   for(int i=0; i<intpts; ++i)
   {
   file<<val[i][0]<<" "<<val[i][1]<<" "<<z[i];
   file<<"\n";
   }
   file.close();
}

void neuron(std::vector<std::array<double,3>>& pt, double* wt, double c[][2], double* sig)
{
  //Two RBF neurons
  //double c[2][2] = {{1., 1.}, {0., 0.}};
  double Mat[points][nc+1];
  double rhs[points];
  for(int n=0; n<points; ++n)
    rhs[n] = pt[n][2];
  
  for(int i=0; i<points; ++i)
      for(int j=0; j<nc; ++j)
	Mat[i][j] = exp(-(*sig) * (pow((pt[i][0]-c[j][0]),2) + pow((pt[i][1]-c[j][1]),2)));

  //  for(int i=0; i<points; ++i)
  //   Mat[i][nc] = 1.;
 
  /*   for(int i=0; i<4; ++i)
      for(int j=0; j<3; ++j)
      std::cout<<"\n"<<Mat[i][j];*/

   // Solve A X = b using GSL
  gsl_matrix* a = gsl_matrix_alloc(points, nc+1);
  gsl_vector * x = gsl_vector_alloc (nc+1);
  gsl_vector * b = gsl_vector_alloc (points);
  gsl_vector * t = gsl_vector_alloc (nc+1);
  for(int i=0; i<points; ++i)
    for(int j=0; j<nc+1; ++j)
      gsl_matrix_set (a, i, j, Mat[i][j]);

  for(int i=0; i<points; ++i)
    gsl_vector_set (b, i, rhs[i]);

  gsl_vector* p = gsl_vector_alloc (points);
  gsl_linalg_QR_decomp(a, t);
  gsl_linalg_QR_lssolve(a, t,b, x,p);

  for(int i=0; i<nc+1; ++i)
     wt[i] = x->data[i];

  gsl_vector_free(p);
  gsl_vector_free(t);
  gsl_vector_free(x);
}
