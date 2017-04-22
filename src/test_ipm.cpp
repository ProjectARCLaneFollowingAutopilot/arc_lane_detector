#include <cv.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "../include/inverse_perspective_mapping/inverse_perspective_mapping.hpp"

using namespace cv;
using namespace std;

// Global variables.
float camera_height = 2.0;
float pitch_angle = 45.0;
float focal_length_px = 799.0;

int main(int argc, char* argv[])
{
  cout<<"Test application!"<<endl;
  cv::Mat input_image = cv::imread("/home/nikku/catkin_ws/src/arc_lane_following_autopilot/images/rect.jpg", CV_LOAD_IMAGE_COLOR);
  // Check for invalid input.
  if(! input_image.data )
  {
    cout <<  "Could not open or find the image" << std::endl ;
    return -1;
  }
  cout<<"Input image has been loaded properly."<<endl;


  IPM test_object;
  test_object.IPM::getImage(input_image);
  test_object.IPM::setParam(camera_height, pitch_angle, focal_length_px, input_image.cols, input_image.rows);
  test_object.IPM::invPerspectiveMapping();
  test_object.IPM::~IPM();
  //cv::Mat dst = test_object.IPM::invPerspectiveMapping();

  /* Try out perspective Transformation methods of opencv.
  Point2f a(0.0f, 0.0f);
  Point2f b(479.0f, 0.0f);
  Point2f c(0.0f, 479.0f);
  Point2f d(479.0f, 479.0f);

  Point2f src[4];
  src[0] = a;
  src[1] = b;
  src[2] = c;
  src[3] = d;

  Point2f A(220.0f, 200.0f);
  Point2f B(260.0f, 200.0f);
  Point2f C(180.0f, 280.0f);
  Point2f D(300.0f, 280.0f);

  Point2f dst[4];
  dst[0] = A;
  dst[1] = B;
  dst[2] = C;
  dst[3] = D;

  Mat trafo = cv::getPerspectiveTransform(src, dst);
  Mat output_image = input_image.clone();
  cv::warpPerspective(input_image, output_image, trafo, output_image.size());

  cout<<output_image.size()<<endl;
  imshow("Input",input_image);
  imshow("Output",output_image);

   waitKey(0);
   */

  return 0;
}

/*

This program will only be used to implement and test the IPM class

*/
