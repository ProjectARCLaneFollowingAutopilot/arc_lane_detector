#include <cv.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "../include/inverse_perspective_mapping/inverse_perspective_mapping.hpp"

using namespace cv;
using namespace std;

// Global variables.
float camera_height = 2.0;
float pitch_angle = 45.0;
int focal_length_px = 799;

int main(int argc, char* argv[])
{
  cout<<"Test application!"<<endl;
  cv::Mat input_image = cv::imread("/home/nikku/catkin_ws/src/arc_lane_following_autopilot/images/blue.png", CV_LOAD_IMAGE_COLOR);
  if(! input_image.data )                              // Check for invalid input
  {
    cout <<  "Could not open or find the image" << std::endl ;
    return -1;
  }
  cout<<"Input image has been loaded properly."<<endl;
  cout<<input_image.cols<<" "<<input_image.rows<<endl;

  IPM test_object;
  test_object.IPM::getImage(input_image);
  test_object.IPM::setParam(camera_height, pitch_angle, focal_length_px, input_image.cols, input_image.rows);

  //cv::Mat dst = test_object.IPM::invPerspectiveMapping();

  return 0;
}

/*

This program will only be used to implement and test the IPM class

*/
