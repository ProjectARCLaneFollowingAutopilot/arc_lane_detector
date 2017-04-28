#include <cv.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "../include/inverse_perspective_mapping/inverse_perspective_mapping.hpp"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/String.h"

using namespace cv;
using namespace std;

// Global variables.
IPM test_object;
Mat src_img;
Mat after_ipm;
float camera_height = 1.3;
float pitch_angle = 90;
float focal_length_px = 628.0;
// Variable to only set the parameters once.
int counter = 0;

int main(int argc, char* argv[])
{

  return 0;
}

/*
This program is a playground for Nikhilesh.
*/
