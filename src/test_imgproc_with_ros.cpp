#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/String.h"

using namespace cv;
using namespace std;

// Global variables.
// Variable to save the incoming image from the webcam globally (maybe not needed).
Mat src_img;
Mat src_img_gray;


// Callback functions.
void imageCallback(const sensor_msgs::Image::ConstPtr& incoming_image)
{

  // Create a pointer, where the incoming ros-image gets assigned to.
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(incoming_image, sensor_msgs::image_encodings::BGR8);

  // FLIP THE TABLE. Flip and save the image to global variable.
  cv::flip(cv_ptr->image, src_img, 0);

  // Check for valid image.
  if (!(src_img).data)
  {
    std::cout<<"Failed to load image"<<std::endl;

  }

  cv::cvtColor(src_img, src_img_gray, CV_BGR2GRAY );
  cv::medianBlur(src_img_gray, src_img_gray, 5);

  src_img_gray = src_img_gray(Rect(0,170,639,239));
  cv::namedWindow("Original", CV_WINDOW_AUTOSIZE);
  cv::namedWindow("After canny", CV_WINDOW_AUTOSIZE);

  cv::Canny(src_img_gray, src_img_gray, 40, 130);

  vector<Vec2f> lines;
  // Run hough-line transform on dst_canny to detect lines.
  cv::HoughLines(src_img_gray, lines, 1, CV_PI/180, 110, 0, 0 );
  // Convert back to color image, such that colored lines can be drawn.
  cv::cvtColor(src_img_gray, src_img_gray, CV_GRAY2BGR);
  // Draw the detected lines to src.
  for( size_t i = 0; i < lines.size(); i++ )
  {
    float rho = lines[i][0], theta = lines[i][1];
    Point pt1, pt2;
    double a = cos(theta), b = sin(theta);
    double x0 = a*rho, y0 = b*rho;
    pt1.x = cvRound(x0 + 1000*(-b));
    pt1.y = cvRound(y0 + 1000*(a));
    pt2.x = cvRound(x0 - 1000*(-b));
    pt2.y = cvRound(y0 - 1000*(a));
    line( src_img_gray, pt1, pt2, Scalar(0,255,0), 3, CV_AA);
    }

  cv::imshow("After canny", src_img_gray);
  cv::imshow("Original", src_img);

  cv::waitKey(3);
/*


  // Convert to Grayscale.
  cvtColor(src_img, src_img, CV_RGB2GRAY);

  // Crop image.
  Mat croppedImage = src_img(Rect(0,240,639,239));

  // Define size of gaussian filter.
  Size lol (5, 5);
  // Filter.
  cv::GaussianBlur(croppedImage, croppedImage, lol, 0.6, 0.6);

  // Run the canny operator (gradient, edge detection and thinning) and save the edge-image to dst_canny.
  Canny(croppedImage, croppedImage, 50, 220);
  // Invert dst_canny (black->white, white->black). Not necessary, optional.
  //threshold(croppedImage, croppedImage, 128, 255, THRESH_BINARY_INV);

  //cv::namedWindow("Original", CV_WINDOW_NORMAL);
  //cv::imshow("Original", croppedImage);
  //cv::waitKey(5);

*/
/*

  // Vector to save the parameters of the detected lines.
  vector<Vec2f> lines;
  // Run hough-line transform on dst_canny to detect lines.
  cv::HoughLines(croppedImage, lines, 1, CV_PI/180, 150, 0, 0 );
  // Convert back to color image, such that colored lines can be drawn.
  cv::cvtColor(croppedImage, croppedImage, CV_GRAY2BGR);
  // Draw the detected lines to src.
  for( size_t i = 0; i < lines.size(); i++ )
  {
    float rho = lines[i][0], theta = lines[i][1];
    cv::Point pt1, pt2;
    double a = cos(theta), b = sin(theta);
    double x0 = a*rho, y0 = b*rho;
    pt1.x = cvRound(x0 + 1000*(-b));
    pt1.y = cvRound(y0 + 1000*(a));
    pt2.x = cvRound(x0 - 1000*(-b));
    pt2.y = cvRound(y0 - 1000*(a));
    line( croppedImage, pt1, pt2, Scalar(0,255,0), 3, CV_AA);
    }
    // Display image with detected lines.
    cv::imshow("Detected lines", croppedImage);
    // Wait.
    cv::waitKey(3);
    */
}


int main(int argc, char* argv[])
{
  // Initialize the node.
  ros::init(argc, argv, "test_imgproc_with_ros");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/usb_cam/image_raw", 10, imageCallback);
  ros::spin();

  return 0;
}

/* This program will be used to test different imamgeprocessing techniques in.
For example: Filter, color channels, edge detections, gradient operators,...?
*/
