#include <cv.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "sensor_msgs/Image.h"
#include "../include/inverse_perspective_mapping/inverse_perspective_mapping.hpp"

using namespace cv;
using namespace std;



int main(int argc, char* argv[])
{
  cout<<"Hallo!"<<endl;
  IPM test_object;
  test_object.lolol.red = 10;
  cout<<test_object.lolol.red<<endl;
  return 0;
}


/*

This program will only be used to implement and test the IPM class

*/
