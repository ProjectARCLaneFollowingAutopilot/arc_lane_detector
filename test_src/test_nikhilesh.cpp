#include <cv.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "../include/inverse_perspective_mapping/inverse_perspective_mapping.hpp"

using namespace cv;
using namespace std;

// Global variables.
Mat src_img;
Mat dst_img;
Mat src_gray;
Mat after_ipm;
float camera_height = 1.3;
float pitch_angle = 90;
float focal_length_px = 628.0;
// Variable to only set the parameters once.
int counter = 0;

int main(int argc, char* argv[])
{
  // Load image.
    src_img = cv::imread("/home/nikku/Desktop/ipm_of_Speed.jpg");
    // Check if valid image.
  	if (!src_img.data)
  	{
      cout<<"Failed to load image!"<<endl;
  		return -1;
  	}

    cvtColor( src_img, src_gray, CV_BGR2GRAY );

    int kernel_size = 3;
    int scale = 1;
    int delta = 0;
    int ddepth = CV_16S;
    Laplacian( src_gray, dst_img, ddepth, kernel_size, scale, delta, BORDER_DEFAULT );

    cv::imshow("Laplacian", dst_img);
    cv::waitKey(0);

  return 0;
}

/*
This program is a playground for Nikhilesh.
*/
