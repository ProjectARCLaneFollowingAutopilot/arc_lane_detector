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
// Several openCV images, which represent the different filtered images.
Mat src, dst_canny, dst_canny_inv;

// Callback functions.
// Get sensor_msgs::Image from Webcam-Topic, convert to openCV format, run a Canny filter and apply a hough-line transform to extract lines.
void chatterCallback(const sensor_msgs::Image::ConstPtr& incoming_image)
{
      cout<<"Got image."<<endl;
      // Create a pointer, where the incoming ros-image gets assigned to.
      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(incoming_image, sensor_msgs::image_encodings::BGR8);

      // Check for valid image.
      if (!(cv_ptr->image).data)
      {
        std::cout<<"Failed to load image"<<std::endl;

      }

      // Assign the incoming image to src (source).
      src = cv_ptr->image;
      // Run the canny operator (gradient, edge detection and thinning) and save the edge-image to dst_canny.
      Canny(src, dst_canny, 50, 350);
      // Invert dst_canny (black->white, white->black). Not necessary, optional.
      threshold(dst_canny, dst_canny_inv, 128, 255, THRESH_BINARY_INV);
      // Vector to save the parameters of the detected lines.
      vector<Vec2f> lines;
      // Run hough-line transform on dst_canny to detect lines.
      HoughLines(dst_canny, lines, 1, CV_PI/180, 150, 0, 0 );
      // Convert back to color image, such that colored lines can be drawn.
      cvtColor(dst_canny, dst_canny_inv, CV_GRAY2BGR);
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
        line( src, pt1, pt2, Scalar(0,255,0), 3, CV_AA);
        }
        // Display image with detected lines.
        imshow("Detected lines", src);
        // Wait.
        waitKey(3);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "lane_detector");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("maincamera/image_raw", 10, chatterCallback);
  ros::spin();
  return 0;
}

// Source
// http://www.transistor.io/revisiting-lane-detection-using-opencv.html
