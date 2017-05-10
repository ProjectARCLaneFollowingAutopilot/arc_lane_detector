#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <cmath>
#include <Eigen/Dense>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/String.h"
#include "../../arc_lane_tracking_tools/include/inverse_perspective_mapping/inverse_perspective_mapping.hpp"

using namespace cv;
using namespace std;

// Global variables.
// Current two lines. Parameters for the cropped image.
float theta_left_rad;
float rho_left;
float theta_right_rad;
float rho_right;
float parallel_distance;
// Camera intrinsics.
float camera_height = 1.3;
float camera_angle = PI/2.0;
float focal_length = 628.0;
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
// The final image with only two lines in cropped image.
Mat dst_roi;
// Track the updates.
int update_counter_left = 0;
int update_counter_right = 0;

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
// Function to draw the detected lines with the current rho and theta into an image of the original size.
void drawTwoLinesOriginal(Mat &image_to_draw);
// Function to draw the detected lines with the current rho and theta into an image of the cropped size.
void drawTwoLinesCropped(Mat &image_to_draw);
// Function to draw the specified lines into a specified image in a specified color.
void drawLinesToImage(Mat &image, vector<Vec2f> &lines_to_draw, int b, int g, int r);

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
  dst = src.clone();

  // Crop src to ROI.
  src_roi = src(Rect(0, 250, 640, 170));

  // Initialize the cropped image with two lines, which the user can choose.
  if(init_counter == 0)
  {
    // Prompt user to select two lines on the cropped input image.
    setCtrlPts(src_roi);

    // Avoid another initialization when function gets called again.
    init_counter = init_counter + 1;

    // Transform and save x, y to rho, theta.
    Eigen::Vector2f polar_parameters;
    // Left line.
    polar_parameters = getRhoAndTheta(init_points[0].x, init_points[1].x, init_points[0].y, init_points[1].y);
    theta_left_rad = polar_parameters[0];
    rho_left = polar_parameters[1];
    // Right line;
    polar_parameters = getRhoAndTheta(init_points[2].x, init_points[3].x, init_points[2].y, init_points[3].y);
    theta_right_rad = polar_parameters[0];
    rho_right = polar_parameters[1];
  }

  // Filter the image.
  Mat src_roi_filtered;
  medianBlur(src_roi, src_roi_filtered, 15);

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

  // Draw the two found lines in the original image.
  drawTwoLinesOriginal(dst);
  drawTwoLinesCropped(dst_roi);

  // Show filtered hough lines in original image.
  imshow("All lines", draw_detected_hough);
  imshow("Result", dst);
  imshow("Result_ROI", dst_roi);
  waitKey(1);
}

// Function which  filters the found Hough-Lines in order to find only two lines anymore.
void findTwoNearLines()
{
  bool update_left = false;
  bool update_right = false;

  vector<Vec2f> lines_left;
  vector<Vec2f> lines_right;

  // Set the constraints needed to filter. These constraints shall reset them to default values after five loops without two lines.
  /*

  */

  if(lines.size() > 0)
  {
    // Assign all lines to lines_left and lines_right. In the same step also filter out all obvious bad matches (to big jumps).
    for(int i = 0; i<lines.size(); i++)
    {
      float bottom_crossing_x = lines[i][0]*cos(lines[i][1]) - sin(lines[i][1])*((src_roi.rows - lines[i][0]*sin(lines[i][1]))/(cos(lines[i][1])));
      if(bottom_crossing_x < src_roi.cols/2.0)
      {
        float delta_theta_l = std::abs(theta_left_rad - lines[i][1]);
        if(delta_theta_l < 0.7)
        {
          lines_left.push_back(lines[i]);
        }
      }
      else
      {
        float delta_theta_r = std::abs(theta_right_rad - lines[i][1]);
        if(delta_theta_r < 0.3)
        {
          lines_right.push_back(lines[i]);
        }
      }
    }

    // After having excluded the obvious mismatches, assign the best matches for the next frame -->Update Step.
    if(lines_left.size() > 0)
    {
      float cost_left = 100.0;
      int index_minimal_cost_left = 0;
      for(int i = 0; i < lines_left.size(); i++)
      {
       // For each line calculate the relative error in rho.
       float rel_error_rho_left_loop = (lines_left[i][0] - rho_left)/rho_left;
       // For each line calculate the relative error in theta.
       float rel_error_theta_left_loop = (lines_left[i][1] - theta_left_rad)/theta_left_rad;
       // For each line calculate a cost function.
       float cost_left_loop = sqrt(pow(rel_error_rho_left_loop, 2) + pow(rel_error_theta_left_loop, 2));
       // If the cost is lower than the previous, save the line as the correct one.
       if(cost_left_loop < cost_left)
       {
         index_minimal_cost_left = i;
         cost_left = cost_left_loop;
       }
      }
      // Update left.
      rho_left = lines_left[index_minimal_cost_left][0];
      theta_left_rad = lines_left[index_minimal_cost_left][1];
      update_counter_left = 0;
    }
    else
    {
      // No update for left line.
      update_counter_left += 1;

    }
    if(lines_right.size() > 0)
    {
      float cost_right = 100.0;
      int index_minimal_cost_right = 0;
      for(int i = 0; i < lines_right.size(); i++)
      {
       // For each line calculate the relative error in rho.
       float rel_error_rho_right_loop = (lines_right[i][0] - rho_right)/rho_right;
       // For each line calculate the relative error in theta.
       float rel_error_theta_right_loop = (lines_right[i][1] - theta_right_rad)/theta_right_rad;
       // For each line calculate a cost function.
       float cost_right_loop = sqrt(pow(rel_error_rho_right_loop, 2) + pow(rel_error_theta_right_loop, 2));
       // If the cost is lower than the previous, save the line as the correct one.
       if(cost_right_loop < cost_right)
       {
         index_minimal_cost_right = i;
         cost_right = cost_right_loop;
       }
      }
      // Update right.
      rho_right = lines_right[index_minimal_cost_right][0];
      theta_right_rad = lines_right[index_minimal_cost_right][1];
      update_counter_right = 0;
    }
    else
    {
      // No update for right line.
      update_counter_right += 1;
    }
  }
  else
  {
    // No lines were updated.
    update_counter_left += 1;
    update_counter_right += 1;
    std::cout<<"No lines found at all!"<<std::endl;
  }

  // Based on update_counter_left and update_counter_right, decide if: Extrapolate left from right, extrapolate right from left or take previous parameters.
  if((update_counter_left > 0) && (update_counter_right == 0))
  {
    // Extrapolate left line, using right line and lateral distance from last frame.
    // 1. Get two points in the imag which define the RIGHT Line.
    // Image coordinates as defined in manuscript.
    Point right_far_img;
    Point right_near_img;

    float y_top_roi = 0.0;
    float y_bottom_roi = src_roi.rows;

    right_far_img.x = rho_right*cos(theta_right_rad) - sin(theta_right_rad)*((y_top_roi - rho_right*sin(theta_right_rad))/(cos(theta_right_rad)));
    right_near_img.x = rho_right*cos(theta_right_rad) - sin(theta_right_rad)*((y_bottom_roi - rho_right*sin(theta_right_rad))/(cos(theta_right_rad)));
    right_far_img.y = 250.0;
    right_near_img.y = 420.0;

    // 2. Using IPM-matrices, transform the two image points to world points in the local vehicle frame.
    // World coordinates as defined in manuscript.
    Point right_far_world;
    Point right_near_world;
    // 2a. Calculate the lambda from equation (7) to see, where the ray crosses the ground/street-plane.
    float lambda_right_far = camera_height/(focal_length*cos(camera_angle) - (src.rows/2.0 - right_far_img.y)*sin(camera_angle));
    float lambda_right_near = camera_height/(focal_length*cos(camera_angle) - (src.rows/2.0 - right_near_img.y)*sin(camera_angle));
    // 2b. Knowing lambda, calculate the projections of the two points on the groundplane.
    right_far_world.x = lambda_right_far*(cos(camera_angle)*(src.rows/2.0 - right_far_img.y) + focal_length*sin(camera_angle));
    right_far_world.y = lambda_right_far*(-1.0)*(right_far_img.x - src.cols/2.0);
    right_near_world.x = lambda_right_near*(cos(camera_angle)*(src.rows/2.0 - right_near_img.y) + focal_length*sin(camera_angle));;
    right_near_world.y = lambda_right_near*(-1.0)*(right_near_img.x - src.cols/2.0);

    // 3. Find the directional vector of those two world points, find the left-facing normal vector (in world coordinates).
    Point2f dir_right;
    Point2f norm_right;

    dir_right.x = right_far_world.x - right_near_world.x;
    dir_right.y = right_far_world.y - right_near_world.y;

    norm_right.x = -dir_right.y/sqrt(pow(dir_right.x, 2) + pow(dir_right.y, 2));
    norm_right.y = dir_right.x/sqrt(pow(dir_right.x, 2) + pow(dir_right.y, 2));
    // 4. Using the normal vector, the parallel_distance and one point of the right line, find a point on the left line.
    // World coordinates as defined in manuscript.
    Point left_near_world;

    left_near_world.x = right_near_world.x + parallel_distance*norm_right.x;
    left_near_world.y = right_near_world.y + parallel_distance*norm_right.y;
    // 5. Find another point on the left line.
    Point left_far_world;

    left_far_world.x = right_far_world.x + parallel_distance*norm_right.x;
    left_far_world.y = right_far_world.y + parallel_distance*norm_right.y;

    std::cout<<"Left Far: "<<left_far_world<<std::endl;
    std::cout<<"Right Far: "<<right_far_world<<std::endl;
    std::cout<<"Left Near: "<<left_near_world<<std::endl;
    std::cout<<"Right Near: "<<right_near_world<<std::endl;
    // 6. Transform the two left line's point to the image, using PM.
    // Transform to camera coordinate system.
    Point3f left_far_camera;
    Point3f left_near_camera;

    left_far_camera.x = 0*(left_far_world.x) -1.0*(left_far_world.y) + 0*(-camera_height);
    left_far_camera.y = cos(camera_angle)*(left_far_world.x) + 0*(left_far_world.y) + sin(camera_angle)*(-camera_height);
    left_far_camera.z = -sin(camera_angle)*(left_far_world.x) + 0*(left_far_world.y) + cos(camera_angle)*(-camera_height);

    left_near_camera.x = 0*(left_near_world.x) -1.0*(left_near_world.y) + 0*(-camera_height);
    left_near_camera.y = cos(camera_angle)*(left_near_world.x) + 0*(left_near_world.y) + sin(camera_angle)*(-camera_height);
    left_near_camera.z = -sin(camera_angle)*(left_near_world.x) + 0*(left_near_world.y) + cos(camera_angle)*(-camera_height);
    // Transform to pixels. In image coordinates.
    float lambda_px_left_far = -focal_length/(left_far_camera.z);
    float lambda_px_left_near = -focal_length/(left_near_camera.z);

    cv::Point2f left_far;
    cv::Point2f left_near;

    left_far.x = lambda_px_left_far*left_far_camera.x + src.cols/2.0;
    left_far.y = src.cols/2.0 - lambda_px_left_far*left_far_camera.y;
    left_near.x = lambda_px_left_near*left_near_camera.x + src.cols/2.0;
    left_near.y = src.cols/2.0 - lambda_px_left_near*left_near_camera.y;
    // 7. Convert to rho, theta for the cropped image.
    // Get rho and theta.
    Eigen::Vector2f params_left = getRhoAndTheta(left_near.x, left_far.x, left_near.y, left_far.y);

    theta_left_rad = params_left[0];
    rho_left = params_left[1];

    std::cout<<"Extrapolated left from right."<<std::endl;
    // Only for now.
    update_counter_left = 0;
  }
  else if((update_counter_left == 0) && (update_counter_right > 0))
  {
    // Extrapolate right line, using left line and lateral distance from last frame.
    // 1. Get two points in the image which define the LEFT Line.
    // 2. Using IPM-matrices, transform the two image points to world points in the local vehicle frame.
    // 3. Find the directional vector of those two world points, find the right-facing normal vector.
    // 4. Using the normal vector, the parallel_distance and one point of the left line, find a point on the right line.
    // 5. Find another point on the right line, using the left line's directional vector.
    // 6. Transform the two right line's point to the image, using PM.
    // 7. Convert to rho_right, theta_right_rad and save them to global variables.

    // FOR NOW: JUST LEAVE IT TO THE OLD VALUES OR THE ONES WHICH WERE ACTUALLY FOUND.
  }
  else if((update_counter_left > 0) && (update_counter_right > 0))
  {
    // Just take the previous parameters. --> Do nothing, do not change the parameters.
  }

  // Final step: Calculate parallel_distance,...
  parallel_distance = 3;
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
  //cout << "Here is the matrix A:\n" << A << endl;
  //cout << "Here is the vector b:\n" << b << endl;
  Eigen::Vector2f x = A.colPivHouseholderQr().solve(b);
  //cout << "The solution is:\n" << x << endl;
  float rho = x[0];

  Eigen::Vector2f resultat;
  resultat << theta,rho;

  return resultat;
}

// Function to draw the detected lines with the current rho and theta into an image of the original size.
void drawTwoLinesOriginal(Mat &image_to_draw)
{
  // Draw the lines into original image.
  // Calculate point coordinates (cartesian) where the lines cross the upper and lower horizontal limitation of the cropped image.
  float y_top = 0.0;
  float y_bottom = src_roi.rows;
  float x_top_left = rho_left*cos(theta_left_rad) - sin(theta_left_rad)*((y_top - rho_left*sin(theta_left_rad))/(cos(theta_left_rad)));
  float x_bottom_left = rho_left*cos(theta_left_rad) - sin(theta_left_rad)*((y_bottom - rho_left*sin(theta_left_rad))/(cos(theta_left_rad)));
  float x_top_right = rho_right*cos(theta_right_rad) - sin(theta_right_rad)*((y_top - rho_right*sin(theta_right_rad))/(cos(theta_right_rad)));
  float x_bottom_right = rho_right*cos(theta_right_rad) - sin(theta_right_rad)*((y_bottom - rho_right*sin(theta_right_rad))/(cos(theta_right_rad)));

  Point left_top_dst;
  Point left_bottom_dst;
  Point right_top_dst;
  Point right_bottom_dst;

  left_top_dst.x = x_top_left;
  left_top_dst.y = 250.0;
  left_bottom_dst.x = x_bottom_left;
  left_bottom_dst.y = 420.0;
  right_top_dst.x = x_top_right;
  right_top_dst.y = 250.0;
  right_bottom_dst.x = x_bottom_right;
  right_bottom_dst.y = 420.0;

  line(image_to_draw, left_top_dst, left_bottom_dst, Scalar(0, 255, 0), 3, CV_AA);
  line(image_to_draw, right_top_dst, right_bottom_dst, Scalar(0, 255, 0), 3, CV_AA);
}

// Function to draw the detected lines with the current rho and theta into an image of the cropped size.
void drawTwoLinesCropped(Mat &image_to_draw)
{
  // Show filtered hough lines in cropped image.
  Point pt1;
  Point pt2;
  Point pt3;
  Point pt4;
  double a_left = cos(theta_left_rad);
  double b_left = sin(theta_left_rad);
  double a_right = cos(theta_right_rad);
  double b_right = sin(theta_right_rad);

  double x0_left = a_left*rho_left;
  double y0_left = b_left*rho_left;
  double x0_right = a_right*rho_right;
  double y0_right = b_right*rho_right;

  pt1.x = cvRound(x0_left + 1000*(-b_left));
  pt1.y = cvRound(y0_left + 1000*(a_left));
  pt2.x = cvRound(x0_left - 1000*(-b_left));
  pt2.y = cvRound(y0_left - 1000*(a_left));

  pt3.x = cvRound(x0_right + 1000*(-b_right));
  pt3.y = cvRound(y0_right + 1000*(a_right));
  pt4.x = cvRound(x0_right - 1000*(-b_right));
  pt4.y = cvRound(y0_right - 1000*(a_right));

  image_to_draw = src_roi.clone();

  line(image_to_draw, pt1, pt2, Scalar(0, 255, 0), 3, CV_AA);
  line(image_to_draw, pt3, pt4, Scalar(0, 255, 0), 3, CV_AA);
}

// Function to draw the specified lines into a specified image.
void drawLinesToImage(Mat &image, vector<Vec2f> &lines_to_draw, int b, int g, int r)
{
  for(int i = 0; i < lines_to_draw.size(); i++)
	{
		float rho = lines_to_draw[i][0];
		float theta = lines_to_draw[i][1];
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
		line(image, pt1, pt2, Scalar(b, g, r), 3, CV_AA);
	}
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
  drawLinesToImage(draw_to, lines, 0, 255, 0);
}
