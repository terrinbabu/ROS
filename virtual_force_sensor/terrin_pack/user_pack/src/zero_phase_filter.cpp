#define _FILE_OFFSET_BITS 64
#include <ros/ros.h>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <list>
#include <err.h>
#include <errno.h>
#include <strings.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <eigen3/Eigen/StdVector>
#include <tsupport/basic.h>

using namespace std;

int main(int argc, char** argv) 
{
ros::init(argc, argv, "zero_phase_filter");
ros::NodeHandle nh;

//  Input 

std::vector<double> input;
ifstream input_file ("/home/terrin/projects/virtual_force_sensor/files/log_filtering/zero_phase_filter/Dq.txt");

std::string::size_type sz;

input.clear();
double value;
string line;

if (input_file.is_open())
{
    while ( getline (input_file,line) )
    {
        value = stod(line, &sz);
        input.push_back(value);
    }
input_file.close();
}
else 
cout << "Unable to open file"; 




int iSize = input.size();

Eigen::VectorXd  Input(iSize),Output(iSize);

tsupport::basic::StdVectoEignVec(input,Input);

// filter

int N = 15;
tsupport::basic::zero_phase_MA_filter (Input,N,Output);

// // // MA filtering forward
// // 
// // MA = Input;
// // 
// // for (int n=N;n<iSize;n++)
// // {
// //     MA[n] = 0;
// //     
// //     for(int i=0;i<N;i++)
// //     {
// //         double temp = Input[n-i]/N;
// //         MA[n] = MA[n]+temp;
// //     }
// // }
// // 
// // // flip
// // 
// // int fn = iSize-1;
// // 
// // for (int i=0;i<iSize;i++)
// // {
// //    Flip_MA[fn] = MA[i];
// //    fn = fn-1;
// // }
// // 
// // // MA filtering backward
// // 
// // Flip_FF = Flip_MA;
// // 
// // for (int n=N;n<iSize;n++)
// // {
// //     Flip_FF[n] = 0;
// //     
// //     for(int i=0;i<N;i++)
// //     {
// //         double temp = Flip_MA[n-i]/N;
// //         Flip_FF[n] = Flip_FF[n]+temp;
// //     }
// // }
// // 
// // // flip 
// // 
// // int ffn = iSize-1;
// // 
// // for (int i=0;i<iSize;i++)
// // {
// //    FF[ffn] = Flip_FF[i];
// //    ffn = ffn-1;
// // }

//   output file

  ofstream output_file;
  output_file.open ("/home/terrin/projects/virtual_force_sensor/files/log_filtering/zero_phase_filter/ff_Dq.txt");

  for ( int i = 0; i < iSize; i++ )
      output_file << Output[i] << "\n";
  output_file.close();

  return 0;
}
