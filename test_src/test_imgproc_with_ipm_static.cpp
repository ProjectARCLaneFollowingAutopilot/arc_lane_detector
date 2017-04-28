#include <cv.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "../include/inverse_perspective_mapping/inverse_perspective_mapping.hpp"

using namespace cv;
using namespace std;

// Global variables.
Mat src_img;
Mat after_ipm;
Mat after_ipm_without_green;
Mat bird_gray;
Mat bird_canny;
float camera_height = 1.3;      //0.7;         // 1.3 0.78;
float pitch_angle = 90;                 //80.0;
float focal_length_px = 628.0;
// Variable to only set the parameters once.


int main(int argc, char* argv[])
{
  // Load image.
  src_img = cv::imread("/home/nikku/Desktop/Speed.jpg");
  // Check if valid image.
	if (!src_img.data)
	{
    cout<<"Failed to load image!"<<endl;
		return -1;
	}

  // Create an object.
  IPM test_object;

  /// Give test_object the image.
  test_object.IPM::getImage(src_img);

  // Set the extrinsics and intrinsics of the camera.
  test_object.IPM::setParam(camera_height, pitch_angle, focal_length_px, src_img.cols, src_img.rows);

  // Do the IPM transformation on the received image.
  after_ipm = test_object.IPM::invPerspectiveMapping();

  // Convert to grayscale.
  cv::cvtColor(after_ipm, bird_gray, CV_BGR2GRAY );

  // Do filtering.
  cv::medianBlur(bird_gray, bird_gray, 5);
  cv::Size ksize(5,5);
  cv::GaussianBlur(bird_gray, bird_gray, ksize, 0.0, 10.0);

  // Run canny.
  cv::Canny(bird_gray, bird_canny, 40, 130);

  // Display.
  cv::imshow("Ipmed", after_ipm);
  //cv::imshow("Canny Bird", bird_canny);

  cv::waitKey(0);

  cv::imwrite("/home/nikku/Desktop/IPM/after_ipm.jpg", after_ipm);
  cv::imwrite("/home/nikku/Desktop/IPM/gaussfilter.jpg", bird_gray);

  // Destroy the object.
  test_object.IPM::~IPM();
  return 0;
}

/*
This program will be used to test the implementation of the IPM class with imgproc results.
*/
