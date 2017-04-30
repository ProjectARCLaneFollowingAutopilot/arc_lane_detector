#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <cmath>
#include <Eigen/Dense>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/String.h"
#include "../include/inverse_perspective_mapping/inverse_perspective_mapping.hpp"

using namespace cv;
using namespace std;

// Global variables.
// Current two lines.
float theta_left_rad;
float rho_left_rad;
float theta_right_rad;
float rho_right_rad;
// Lines.
vector<Vec2f> lines;
// Variable to set initial lane once.
int init_counter = 0;
// Vector to save initialization points.
cv::Point2f init_points[4];
// Pointer for cv::Bridge to copy the converted sensor message to.
cv_bridge::CvImagePtr cv_ptr;
// Save the incoming image which is not upside down.
Mat src;
// The flipped and cropped version of the fetched image.
Mat src_roi;
// The final image with only two lines.
Mat dst;

// FUNCTION DECLARATIONS.
// NIKHILESH'S FUNCTIONS.
// Callback function which fetches new images and gets shit done.
void webcamCallback(const sensor_msgs::Image::ConstPtr& incoming_image);
// Funtion which  filters the found Hough-Lines in order to find only two lines anymore.
void findTwoNearLines();
// Callback function for setMouseCallback and returns the point clicked on.
void getClickedPixel(int event, int x, int y, int flags, void *ptr);
// Function to prompt the user to set four input control points, which are on the two lines.
void setCtrlPts(Mat& calibration_image);
// Function which takes two points on a line and returns the polar parameters (rho, theta) in the openCV convention.
Eigen::Vector2f getRhoAndTheta(float x_a, float x_b, float y_a, float y_b);

// ROBIN'S FUNCTIONS.
// Function to show an Image in a new window.
void showImage(Mat show, string name);
// Does the Hough-Transform and draws the lines.
void houghTransform(Mat contours, Mat &draw_to);

int main(int argc, char* argv[])
{
  // Initialize ROS Node.
  ros::init(argc, argv, "lane_detector");
  ros::NodeHandle n;

  // Declare callback function to get Webcam image.
  ros::Subscriber sub = n.subscribe("/usb_cam/image_raw", 10, webcamCallback);

  // Run that shit.
  ros::spin();

  return 0;
}

// FUNCTION DEFINITIONS.
// NIKHILESH'S FUNCTIONS.
// Callback function which fetches new images and gets shit done.
void webcamCallback(const sensor_msgs::Image::ConstPtr& incoming_image)
{
  // Save incoming image.
  cv_ptr = cv_bridge::toCvCopy(incoming_image, sensor_msgs::image_encodings::BGR8);
  // Check for valid image.
  if (!(cv_ptr->image).data)
  {
    std::cout<<"Got bad image from webcam!"<<std::endl;
  }
  // Flip and save the image to global variable.
  cv::flip(cv_ptr->image, src, -1);

  // Crop src to ROI.
  src_roi = src.clone();
	int x_roi = 0;
	int y_roi = 220;
	int width_roi = 640;         //src.cols;
	int height_roi = 200;            //src.rows - y_roi;
	Rect region_roi = Rect(x_roi, y_roi, width_roi, height_roi);
  src_roi = src(region_roi);

  // Initialize the cropped image with two lines, which the user can choose.
  if(init_counter == 0)
  {
    // Prompt user to select two lines on the cropped input image.
    setCtrlPts(src_roi);

    // Avoid anothet initialization when function gets called again.
    init_counter = init_counter + 1;

    // Display image with initialized lines.
    Mat src_roi_drawn = src_roi.clone();
    cv::line(src_roi_drawn, init_points[0], init_points[1], Scalar(0,255,0), 3, 8);
    cv::line(src_roi_drawn, init_points[2], init_points[3], Scalar(0,255,0), 3, 8);
    //cv::imshow("Initlines", src_roi_drawn);
    //cv::waitKey(5000);

    // Transform and save x, y to rho, theta.
    Eigen::Vector2f polar_parameters;
    // Left line.
    polar_parameters = getRhoAndTheta(init_points[0].x, init_points[1].x, init_points[0].y, init_points[1].y);
    theta_left_rad = polar_parameters[0];
    rho_left_rad = polar_parameters[1];
    // Right line;
    polar_parameters = getRhoAndTheta(init_points[2].x, init_points[3].x, init_points[2].y, init_points[3].y);
    theta_right_rad = polar_parameters[0];
    rho_right_rad = polar_parameters[1];
  }

  // Filter the image.
  // Filter with Gauss. Several parameters to be determined.
  Size gaussian_blur_size(23, 23);
  Mat src_roi_gauss;
  GaussianBlur(src_roi, src_roi_gauss, gaussian_blur_size, 15, 3, 0);
  // Filter with median. Parameter to determined.
  Mat src_roi_median;
  medianBlur(src_roi, src_roi_median, 15);

  // Decide for a filtering method.
  Mat src_roi_filtered = src_roi_median;

  // Do the Hough Transform.
  Mat contours_inverted = src_roi_filtered.clone();
  //Run Canny. Parameter to be determined.
  Canny(src_roi_filtered, contours_inverted, 30, 50);
  Mat contours = contours_inverted.clone();
  threshold(contours_inverted, contours, 128, 255, THRESH_BINARY_INV);
  //Hough-Transform.
  Mat draw_detected_hough = src_roi.clone();
  houghTransform(contours_inverted, draw_detected_hough);
  // Iterate through all lines, which were found by Hough to find the two lines which are closest to the previous two lines.
  findTwoNearLines();

  // Show filtered hough image.
  Point pt1;
  Point pt2;
  Point pt3;
  Point pt4;
  double a_left = cos(theta_left_rad);
  double b_left = sin(theta_left_rad);
  double a_right = cos(theta_right_rad);
  double b_right = sin(theta_right_rad);

  double x0_left = a_left*rho_left_rad;
  double y0_left = b_left*rho_left_rad;
  double x0_right = a_right*rho_right_rad;
  double y0_right = b_right*rho_right_rad;

  pt1.x = cvRound(x0_left + 1000*(-b_left));
  pt1.y = cvRound(y0_left + 1000*(a_left));
  pt2.x = cvRound(x0_left - 1000*(-b_left));
  pt2.y = cvRound(y0_left - 1000*(a_left));

  pt3.x = cvRound(x0_right + 1000*(-b_right));
  pt3.y = cvRound(y0_right + 1000*(a_right));
  pt4.x = cvRound(x0_right - 1000*(-b_right));
  pt4.y = cvRound(y0_right - 1000*(a_right));

  dst = src_roi.clone();

  line(dst, pt1, pt2, Scalar(0, 255, 0), 3, CV_AA);
  line(dst, pt3, pt4, Scalar(0, 255, 0), 3, CV_AA);

  imshow("Result", dst);
  waitKey(1);
}

// Function which  filters the found Hough-Lines in order to find only two lines anymore.
void findTwoNearLines()
{
  // Initialize counter variables with ridiculously high values.
  float cost_left = 10.0;
  float cost_right = 10.0;
  int minimal_cost_left = 0;
  int minimal_cost_right = 0;
  for(int i = 0; i < lines.size(); i++)
  {
    // For each line calculate the relative error in rho (left).
    float rel_error_rho_left_loop = (lines[i][0] - rho_left_rad)/rho_left_rad;
    // For each line calculate the relative error in rho (right).
    float rel_error_rho_right_loop = (lines[i][0] - rho_right_rad)/rho_right_rad;
    // For each line calculate the relative error in theta (left).
    float rel_error_theta_left_loop = (lines[i][1] - theta_left_rad)/theta_left_rad;
    // For each line calculate the relative error in theta (right).
    float rel_error_theta_right_loop = (lines[i][1] - theta_left_rad)/theta_left_rad;
    // For each line calculate a cost function (left).
    float cost_left_loop = sqrt(pow(rel_error_rho_left_loop, 2) + pow(rel_error_theta_left_loop, 2));
    // For each line calculate a cost function (right).
    float cost_right_loop = sqrt(pow(rel_error_rho_right_loop, 2) + pow(rel_error_theta_right_loop, 2));
    // If the cost is lower than the previous, save the line as the correct one (left).
    if(cost_left_loop < cost_left)
    {
      minimal_cost_left = i;
      cost_left = cost_left_loop;
    }
    // If the cost is lower than the previous, save the line as the correct one (right).
    if(cost_right_loop < cost_right)
    {
      minimal_cost_right = i;
      cost_right = cost_right_loop;
    }
  }

  // Assign new parameters, only if error is not too big.
  if(lines[minimal_cost_right][1] > 7*PI/4.0 && lines[minimal_cost_right][1] < 2.0*PI)
  {
    rho_right_rad = lines[minimal_cost_right][0];
    theta_right_rad = lines[minimal_cost_right][1];
  }
  if(lines[minimal_cost_left][1] > 0 && lines[minimal_cost_left][1] < PI/4.0)
  {
    rho_left_rad = lines[minimal_cost_left][0];
    theta_left_rad = lines[minimal_cost_left][1];
  }
}

// Callback function for setMouseCallback and returns the point clicked on.
void getClickedPixel(int event, int x, int y, int flags, void *ptr)
{
  if(event == cv::EVENT_LBUTTONDOWN)
  {
    std::cout<<"CLICK"<<std::endl;
    cv::Point2f *p = (cv::Point2f*)ptr;
    p->x = x;
    p->y = y;
  }
}

// Function to prompt the user to set four input control points, which are on the two lines.
void setCtrlPts(Mat& calibration_image)
{
  // Let user select the input points.
  cv::Point2f p;
  cv::namedWindow("Set Control Points", CV_WINDOW_AUTOSIZE);

  // Prompt user to select four points in perspectively distorted input image of the groundplane.
  std::cout<<"Select two lines to Initialize."<<std::endl;
  for(int i = 0; i < 4; i++)
  {
    std::cout<<"Point: "<<i+1<<" out of "<<4<<std::endl;
    std::cout<<"You have now 10 sec to click on your point"<<std::endl;
    cv::imshow("Set Control Points", calibration_image);
    cv::setMouseCallback("Set Control Points", getClickedPixel, &p);
    cv::waitKey(10000);
    std::cout<<"Saved pixels: "<<std::endl;
    std::cout<<p<<std::endl;
    init_points[i] = p;
    std::cout<<"Input Points saved!"<<std::endl;
  }
  setMouseCallback("Set Control Points", NULL);
}

Eigen::Vector2f getRhoAndTheta(float x_a, float x_b, float y_a, float y_b)
{
  // Calculate theta.
  float theta = atan2(x_a - x_b, y_b - y_a);
  // Solve linear system of equation to find rho.
  Eigen::Matrix2f A;
  Eigen::Vector2f b;
  A << sin(theta),(y_a - y_b),cos(theta),(x_b - x_a);
  b << y_a,x_a;
  cout << "Here is the matrix A:\n" << A << endl;
  cout << "Here is the vector b:\n" << b << endl;
  Eigen::Vector2f x = A.colPivHouseholderQr().solve(b);
  cout << "The solution is:\n" << x << endl;
  float rho = x[0];

  Eigen::Vector2f resultat;
  resultat << theta,rho;

  return resultat;
}

// ROBIN'S FUNCTIONS
// Function to show an Image in a new window.
void showImage(Mat show, string name)
{
	namedWindow(name, CV_WINDOW_AUTOSIZE );
	imshow(name, show);
  waitKey(1);
}

// Does the Hough-Transform and draws the lines.
void houghTransform(Mat contours, Mat &draw_to)
{
  // Hough transform. Parameter to be determined.
 	HoughLines(contours, lines, 1, CV_PI/180, 90, 0, 0);
	for(int i = 0; i < lines.size(); i++)
	{
		float rho = lines[i][0];
		float theta = lines[i][1];
		// cout << i << "   " <<theta <<endl;
		Point pt1;
		Point pt2;
		double a = cos(theta);
		double b = sin(theta);
		double x0 = a*rho;
		double y0 = b*rho;
		pt1.x = cvRound(x0 + 1000*(-b));
		pt1.y = cvRound(y0 + 1000*(a));
		pt2.x = cvRound(x0 - 1000*(-b));
		pt2.y = cvRound(y0 - 1000*(a));
		line(draw_to, pt1, pt2, Scalar(0, 255, 0), 3, CV_AA);
	}
}
