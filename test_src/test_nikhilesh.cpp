#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <cmath>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <vector>
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
float alpha_deg;
float beta_deg;

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
// Updates.
int update_counter_left = 0;
int update_counter_right = 0;
bool draw_left = 1;
bool draw_right = 1;
Vec2f default_left(-229.39, -2.4081);
Vec2f default_right(-200.435, 2.12559);
float default_alpha;
float default_beta;
int reset_trigger = 20;
float del_alpha_limit = 5;
float del_beta_limit = 5;
float del_rho_left_limit = 50;
float del_rho_right_limit = 10;

// FUNCTION DECLARATIONS.
// NIKHILESH'S FUNCTIONS.
// Callback function which fetches new images and gets shit done.
void webcamCallback(const sensor_msgs::Image::ConstPtr& incoming_image);
// New method to filter out the lines vector to find only two lines.
void filterLines();
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
void drawLinesToImage(Mat &image, vector<Vec2f> &lines_to_draw);

// ROBIN'S FUNCTIONS.
// Function to show an Image in a new window.
void showImage(Mat show, string name);
//Changed Functions.
void houghTransform(Mat contours, Mat &draw_to, vector<Vec2f> &lines_hT, int threshold);
//Following Function are new:.
//Functions return a Lines-Vector.
vector<Vec2f> HoughClassic (Mat src_HC);
vector<Vec2f> GrayProperty (Mat src);
vector<Vec2f> InRange (Mat src_IR);
vector<Vec2f> CompareGray (Mat src_CG);
//Functions used by the Functions above.
Mat FindGray(Mat src);
Mat showChannel(Mat RGB, bool B, bool G, bool R);
Mat RoadThreshold(Mat src_RT);
Vec3b IntensityOfArea(Mat &src_IOA, int x_gray, int y_gray, int width_gray, int height_gray);

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
  //src = cv_ptr->image;
  dst = src.clone();

  // Crop src to ROI.
  cv::Mat src_copy = src.clone();
  src_roi = src_copy(Rect(0, 250, 640, 170));

  // Initialize the cropped image with two lines, which the user can choose.
  if(init_counter == 0)
  {
    // Prompt user to select two lines on the cropped input image.
    setCtrlPts(src_roi);
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

    // Set the default values as initial value.
    default_left[1] = theta_left_rad;
    default_right[1] = theta_right_rad;
    default_left[0] = rho_left;
    default_right[0] = rho_right;

    float top_crossing_x_left = rho_left*cos(theta_left_rad) - sin(theta_left_rad)*((0 - rho_left*sin(theta_left_rad))/(cos(theta_left_rad)));
    float bottom_crossing_x_left = rho_left*cos(theta_left_rad) - sin(theta_left_rad)*((src_roi.rows - rho_left*sin(theta_left_rad))/(cos(theta_left_rad)));
    
    std::cout<<top_crossing_x_left<<" <-> "<<bottom_crossing_x_left<<std::endl;

    float top_crossing_x_right = rho_right*cos(theta_right_rad) - sin(theta_right_rad)*((0 - rho_right*sin(theta_right_rad))/(cos(theta_right_rad)));
    float bottom_crossing_x_right = rho_right*cos(theta_right_rad) - sin(theta_right_rad)*((src_roi.rows - rho_right*sin(theta_right_rad))/(cos(theta_right_rad)));

    float del_x = std::abs(top_crossing_x_left - bottom_crossing_x_left);
    float del_y = std::abs(src_roi.rows);
    float m = del_y/del_x;
    alpha_deg = 45.0;    //std::atan(m)*180.0/PI;

    del_x = std::abs(top_crossing_x_right - bottom_crossing_x_right);
    del_y = std::abs(src_roi.rows);
    m = del_y/del_x;
    beta_deg = std::atan(m)*180.0/PI;

    default_alpha = alpha_deg;
    default_beta = beta_deg;

    std::cout<<"Alpha deg: "<<alpha_deg<<std::endl;

    // Avoid another initialization when function gets called again.
    init_counter = init_counter + 1;
  }

  // Hough-Transform.
  vector<Vec2f> test0 = HoughClassic (src_roi);
  vector<Vec2f> test1 = GrayProperty(src_roi);
  //vector<Vec2f> test2 = InRange(src_roi);
  vector<Vec2f> test3 = CompareGray (src_roi);

  // Append all vectors.
  lines.insert(lines.end(), test0.begin(), test0.end());
  lines.insert(lines.end(), test1.begin(), test1.end());
  //lines.insert(lines.end(), test2.begin(), test2.end());
  lines.insert(lines.end(), test3.begin(), test3.end());

  std::cout<<"Found lines in image: "<< lines.size()<< std::endl;

  // Filter all detected lines to find left and right line and saves to global variables.
  //findTwoNearLines();
  // New function to filter out.
  filterLines();

  // Draw all lines into cropped image.
  Mat all_lines = src.clone();
  all_lines = all_lines(Rect(0, 250, 640, 170));
  drawLinesToImage(all_lines, lines);


  // Show filtered hough lines in original image.
  drawTwoLinesOriginal(dst);
  imshow("Result", dst);
  
  imshow("All lines", all_lines);
  waitKey(1);
  lines.clear();
}

// New method to filter out the lines vector to find only two lines.
void filterLines()
{
  // At a first step, remove all lines passing through the VI sensor.
  vector<Vec2f> lines_temporary;
  for(int i = 0; i<lines.size(); i++)
  { 
    float x_middle = src_roi.cols/2.0;
    float y_middle = (-cos(lines[i][1])/sin(lines[i][1]))*x_middle + (lines[i][0])/sin(lines[i][1]);
    if(y_middle < 85)
    {
      lines_temporary.push_back(lines[i]);
    }
  }
  lines = lines_temporary;

  vector<Vec2f> lines_left;
  vector<Vec2f> lines_right;

  // Check if any of the lines haven't been updated for a long time (always kept constant from previous frame).
  // If any line wasn't updated for long time: Set that line back to default and a bool, such that this line doesn't get drawn in.
  if((update_counter_left > reset_trigger) && (update_counter_right > reset_trigger))
  {
    // Reset both.
    draw_left = 0;
    draw_right = 0;
    rho_left = default_left[0];
    theta_left_rad = default_left[1];
    rho_right = default_right[0];
    theta_right_rad = default_right[1];
    alpha_deg = default_alpha;
    beta_deg = default_beta;
  }
  else if((update_counter_left > reset_trigger) && (update_counter_right < reset_trigger))
  {
    // Reset left.
    draw_left = 0;
    rho_left = default_left[0];
    theta_left_rad = default_left[1];
    alpha_deg = default_alpha;

  }
  else if((update_counter_left < reset_trigger) && (update_counter_right > reset_trigger))
  {
    // Reset right.
    draw_right = 0;
    rho_left = default_left[0];
    theta_left_rad = default_left[1];
    beta_deg = default_beta;
  }

  // For loop: Split the lines vector to a left and a right vector. Only assign those where the gradient angle is below a specific threshold.
  if(lines.size() > 0)
  {
    for(int i = 0; i<lines.size(); i++)
    {      
      float top_crossing_x = lines[i][0]*cos(lines[i][1]) - sin(lines[i][1])*((0 - lines[i][0]*sin(lines[i][1]))/(cos(lines[i][1])));
      float bottom_crossing_x = lines[i][0]*cos(lines[i][1]) - sin(lines[i][1])*((src_roi.rows - lines[i][0]*sin(lines[i][1]))/(cos(lines[i][1])));
      // Assign left line.
      if((bottom_crossing_x < src_roi.cols/2.0) && (bottom_crossing_x > - 320))
      {
        float del_x = std::abs(top_crossing_x - bottom_crossing_x);
        float del_y = std::abs(src_roi.rows);
        float m = del_y/del_x;
        float alpha = std::atan(m)*180.0/PI;
        float del_rho = std::abs(rho_left - lines[i][0]);
        if((std::abs(alpha - alpha_deg) < del_alpha_limit))
        {
          lines_left.push_back(lines[i]);

        }
      }
      // Assign right line.
     else if((bottom_crossing_x > src_roi.cols/2.0) && (bottom_crossing_x < 960))
      {
        float del_x = std::abs(top_crossing_x - bottom_crossing_x);
        float del_y = std::abs(src_roi.rows);
        float m = del_y/del_x;
        float beta = std::atan(m)*180.0/PI;
        float del_rho = std::abs(rho_right - lines[i][0]);
        if((std::abs(beta - beta_deg) < del_beta_limit) && (del_rho < del_rho_right_limit))
        {
          lines_right.push_back(lines[i]);
        }
      }
    } 
  }

  // Only left lines vector has elements.
  if((lines_left.size() > 0) && (lines_right.size() == 0))
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
     // Update.
     rho_left = lines_left[index_minimal_cost_left][0];
     theta_left_rad = lines_left[index_minimal_cost_left][1];
     update_counter_left = 0;
     update_counter_right += 1;
     draw_left = 1;
  }
  // Only right lines vector has elements.
  else if((lines_left.size() == 0) && (lines_right.size() > 0))
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
     // Update.
     rho_right = lines_right[index_minimal_cost_right][0];
     theta_right_rad = lines_right[index_minimal_cost_right][1];
     update_counter_right = 0;
     update_counter_left += 1;
     draw_right = 1;
  }
  else if((lines_left.size()> 0) && (lines_right.size() > 0))
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
     // Update.
     rho_left = lines_left[index_minimal_cost_left][0];
     theta_left_rad = lines_left[index_minimal_cost_left][1];
     rho_right = lines_right[index_minimal_cost_right][0];
     theta_right_rad = lines_right[index_minimal_cost_right][1];
     update_counter_left = 0;
     update_counter_right = 0;
     draw_left = 1;
     draw_right = 1;
  }
  else if((lines_left.size() == 0) && (lines_right.size() == 0))
  {
    update_counter_left += 1;
    update_counter_right += 1;
  }

    float top_crossing_x_left = rho_left*cos(theta_left_rad) - sin(theta_left_rad)*((0 - rho_left*sin(theta_left_rad))/(cos(theta_left_rad)));
    float bottom_crossing_x_left = rho_left*cos(theta_left_rad) - sin(theta_left_rad)*((src_roi.rows - rho_left*sin(theta_left_rad))/(cos(theta_left_rad)));

    float top_crossing_x_right = rho_right*cos(theta_right_rad) - sin(theta_right_rad)*((0 - rho_right*sin(theta_right_rad))/(cos(theta_right_rad)));
    float bottom_crossing_x_right = rho_right*cos(theta_right_rad) - sin(theta_right_rad)*((src_roi.rows - rho_right*sin(theta_right_rad))/(cos(theta_right_rad)));

    float del_x = std::abs(top_crossing_x_left - bottom_crossing_x_left);
    float del_y = std::abs(src_roi.rows);
    float m = del_y/del_x;
    alpha_deg = std::atan(m)*180.0/PI;

    del_x = std::abs(top_crossing_x_right - bottom_crossing_x_right);
    del_y = std::abs(src_roi.rows);
    m = del_y/del_x;
    beta_deg = std::atan(m)*180.0/PI;


  // Only candidates for left lines.
    // Find the nearest line to the previous frame.
      // Set a bool for left, such that left line gets drawn.
      // Set counter for left to zero.
      // Increase counter for right, that no update has been done.
  // Only candidates for right lines.
    // Find the nearest line to the previous frame.
      // Set a bool for right, such that right line gets drawn.
      // Set counter for right to zero.
      // Increase counter for left, that no update has been done.
  // Candidates for left and right lines.
   // Find the nearest lines to the previous frame.
   // Set the bool, such that both lines will be drawn.
   // Set both counter to zero.
  // No candidates at all.
   // Do nothing, such that the both line will be kept the same.
   // Increase a counter, that no update for both.
}


// Function which  filters the found Hough-Lines in order to find only two lines anymore.
void findTwoNearLines()
{
  bool update_left = false;
  bool update_right = false;

  vector<Vec2f> lines_left;
  vector<Vec2f> lines_right;

  // Set the constraints needed to filter. These constraints shall reset them to default values after five loops without two lines.
  // Idea: Three cases. Find the most suitable reset parameters: Left curved, right curved, straight.
  if((update_counter_left > 5) && (update_counter_right < 5))
  {
    theta_left_rad = -2.4081;
    rho_left = -229.39;
    update_counter_left = 0;
  }
  else if((update_counter_left < 5) && (update_counter_right > 5))
  {
    theta_right_rad = 2.12559;
    rho_right = -200.435;
    update_counter_right = 0;
  }
  else if((update_counter_left > 5) && (update_counter_right > 5))
  {
    theta_left_rad = -2.4081;
    theta_right_rad = 2.12559;
    rho_left = -229.39;
    rho_right = -200.435;
    update_counter_left = 0;
    update_counter_right = 0;
  }
  if(lines.size() > 0)
  {
    // Assign all lines to lines_left and lines_right. In the same step also filter out all obvious bad matches (to big jumps).
    for(int i = 0; i<lines.size(); i++)
    {
      float top_crossing_x = lines[i][0]*cos(lines[i][1]) - sin(lines[i][1])*((0 - lines[i][0]*sin(lines[i][1]))/(cos(lines[i][1])));
      float bottom_crossing_x = lines[i][0]*cos(lines[i][1]) - sin(lines[i][1])*((src_roi.rows - lines[i][0]*sin(lines[i][1]))/(cos(lines[i][1])));
      if((bottom_crossing_x < src_roi.cols/2.0) && (bottom_crossing_x > - 320))
      {
        //float rel_error_theta_left = std::abs(1.0 - lines[i][1]/theta_left_rad);
        float delta_theta_left = std::abs(theta_left_rad - lines[i][1]);
        if(lines[i][1] < 1.3)    //(lines[i][1] < 1.3)(delta_theta_left<5)
        {
          lines_left.push_back(lines[i]);
        }
      }
      if((bottom_crossing_x > src_roi.cols/2.0) && (bottom_crossing_x < 960))
      {
        float rel_error_theta_right = std::abs(1.0 - lines[i][1]/theta_right_rad);
        float delta_theta_right = std::abs(theta_right_rad - lines[i][1]);
        if(rel_error_theta_right < 0.5)  // (delta_theta_right<0.2)
        {
          lines_right.push_back(lines[i]);
        }
      }
    }

    std::cout<<"Number of left lines: "<<lines_left.size()<<std::endl;
    std::cout<<"Number of right lines: "<<lines_right.size()<<std::endl;

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
  std::cout<<"Calculated theta: "<<theta<<std::endl;

  // New method to calculate rho.
  float rho_nom = x_a + y_a*((x_a - x_b)/(y_b - y_a));
  float rho_denom = cos(theta) - sin(theta)*(x_b - x_a)/(y_b - y_a);
  float rho = rho_nom/rho_denom;

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

  if(draw_left == true)
  {
      line(image_to_draw, left_top_dst, left_bottom_dst, Scalar(0, 255, 0), 3, CV_AA);
  }
  if(draw_right == true)
  {
    line(image_to_draw, right_top_dst, right_bottom_dst, Scalar(0, 255, 0), 3, CV_AA);
  }
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
void drawLinesToImage(Mat &image, vector<Vec2f> &lines_to_draw)
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
		line(image, pt1, pt2, Scalar(255, 0, 0), 3, CV_AA);
	}
}

// ROBIN'S FUNCTIONS
// Function to show an Image in a new window.
void showImage(Mat show, string name)
{
	//Open new window and show the image. 
	namedWindow(name, CV_WINDOW_AUTOSIZE );
	imshow(name, show);
  waitKey(1);
}

// Does the Hough-Transform and draws the lines.
void houghTransform(Mat contours, Mat &draw_to, vector<Vec2f> &lines_hT, int threshold)
{
  //Hough transform. Parameter to be determined.
  HoughLines(contours, lines_hT, 1, CV_PI/180, threshold, 0, 0);  // war 90.
  //Iterate through all the lines and draw the line. 
  for(int i = 0; i < lines_hT.size(); i++)
  {
    float rho = lines_hT[i][0];
    float theta = lines_hT[i][1];
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

// Candidate getter functions.
vector<Vec2f> GrayProperty (Mat src_GP)
{ 
  vector<Vec2f> lines_GP;
  Mat contours(src_GP.rows, src_GP.cols, CV_8UC1);
  //Find gray areas using the function FindGray.
  Mat gray = FindGray(src_GP);
  //showImage(gray, "FindGray");
  Canny(gray, contours, 180, 180);
  //Sobel(gray, contours, CV_16S, 1, 0, 3);
  showImage(contours, "konturen");
  houghTransform(contours, src_GP, lines_GP, 30);
  return lines_GP;
}
vector<Vec2f> InRange (Mat src_IR)
{
	//Calculate a mask by using the openCV-function inRange.
  Mat mask(src_IR.rows, src_IR.cols, CV_8UC1);
	inRange(src_IR, Scalar(40, 40, 40),Scalar(150, 150, 150), mask);
	Mat gray(src_IR.rows, src_IR.cols, CV_8UC1);
	Mat contours(src_IR.rows, src_IR.cols, CV_8UC1);
	Mat conv(src_IR.rows, src_IR.cols, CV_8UC1, 255);
	//Apply the mask to the source image. 
	bitwise_and(src_IR, src_IR, conv, mask);
	Canny(mask, contours, 120, 150);
	vector<Vec2f> lines_IR;
	houghTransform(contours, src_IR ,lines_IR, 80);
	return lines_IR;
}
vector<Vec2f> CompareGray (Mat src_CG)
{
  vector<Vec2f> lines_CG;
  Mat src_copy_CG = src_CG.clone();
  //showImage(showChannel(src_CG, true, true, true), "RGB");
  // showImage(RoadThreshold(src_CG), "RoadTreshold");
  Mat color_contour = src_copy_CG.clone();
  //Calculate Canny-image by using the function RoadThreshold;
  Canny(RoadThreshold(src_copy_CG),  color_contour, 30, 50);
  //showImage(color_contour, "FarbCanny");
  Mat show_color_Hough=src_copy_CG.clone();
  houghTransform(color_contour, show_color_Hough, lines_CG, 30);
  // showImage(show_color_Hough, "ColorHoug");
  return lines_CG;
}
Mat showChannel(Mat RGB, bool B, bool G, bool R)
{
	//This function is able to spit the color-channels and merge them again, if they are needed.
	Mat channel[3];
	Mat result = Mat::zeros(RGB.rows, RGB.cols, CV_8UC3);
	split(RGB, channel);
	if (B==false)
		{
		channel[0]=Mat::zeros(RGB.rows, RGB.cols, CV_8UC1);
		}
	if (G==false)
		{
		channel[1]=Mat::zeros(RGB.rows, RGB.cols, CV_8UC1);
		}
	if (R==false)
		{
		channel[2]=Mat::zeros(RGB.rows, RGB.cols, CV_8UC1);
		}
	merge(channel, 3, result);
	return result;
}
Mat RoadThreshold(Mat src_RT)
{
  //Calculate the intensity of areas by using the function IntensityOfArea.
  Vec3b Intensity1 = IntensityOfArea(src_RT, 360, 250, 100, 75);
  Vec3b Intensity2 = IntensityOfArea(src_RT, 180, 250, 100, 75);
  float B_average = (Intensity1[0]+Intensity2[0])/2;
  float G_average = (Intensity1[1]+Intensity2[1])/2;
  float R_average = (Intensity1[2]+Intensity2[2])/2;
  //Define black vector.
  Vec3b black;
  black[0] = 0;
  black[1] = 0;
  black[2] = 0;
  //Define white vector.
  Vec3b white;
  black[0]=255;
  black[1]=255;
  black[2]=255;
  //Iterate through the whole image.
  for(int y=0;y<src_RT.rows;y++)
  {
    for(int x=0;x<src_RT.cols;x++)
      {
      //Calculate the deviation of the intensity. 
      Vec3b color = src_RT.at<Vec3b>(Point(x,y));
      float B_delta_sq = ((color[0]-B_average)/B_average)*((color[0]-B_average)/B_average);
      float G_delta_sq = ((color[1]-G_average)/G_average)*((color[1]-G_average)/G_average);
      float R_delta_sq = ((color[2]-R_average)/R_average)*((color[2]-R_average)/R_average);
      //Threshold the image.
      if ( (B_delta_sq < 0.08) && (G_delta_sq<0.08) && (R_delta_sq<0.08))
      {
        src_RT.at<Vec3b>(Point(x,y)) = black;
      }
      else
      {
        src_RT.at<Vec3b>(Point(x,y)) = white;
      }
    }
  }
  Mat src_RT_U(src_RT.rows, src_RT.cols, CV_8U);
  Mat src_RT_U_filtered(src_RT.rows, src_RT.cols, CV_8U);
  cvtColor(src_RT, src_RT_U, cv::COLOR_RGB2GRAY);
  medianBlur(src_RT_U, src_RT_U_filtered, 15);  
  // showImage(src_RT_U_filtered, "RoadThreshold");
  return src_RT_U_filtered;
}
Vec3b IntensityOfArea(Mat &src_IOA, int x_gray, int y_gray, int width_gray, int height_gray)
{
  //This function calculate the averaged intensity of alle color-channels of a fixed region of a image.
  Rect region_gray = Rect(x_gray, y_gray, width_gray, height_gray);
  rectangle(src_IOA, region_gray, Scalar(255, 255, 255), 10);
  //showImage(src_IOA, "infunction ");
  Mat src_gray = src(region_gray);
  float B_average=0;
  float G_average=0;
  float R_average=0;
  for(int y=0;y<src_gray.rows;y++)
  {
    for(int x=0;x<src_gray.cols;x++)
      {
        Vec3b color = src_gray.at<Vec3b>(Point(x,y));
    B_average=B_average+color[0];
    G_average=G_average+color[1];
    R_average=R_average+color[2];
    }
  }
  B_average=B_average/(src_gray.rows*src_gray.cols);
  G_average=G_average/(src_gray.rows*src_gray.cols);
  R_average=R_average/(src_gray.rows*src_gray.cols);
  Vec3b intensity;
  intensity[0]=B_average;
  intensity[1]=G_average;
  intensity[2]=R_average;
  return intensity;
}
Mat FindGray(Mat src_FG)
{
  Mat blur = src_FG.clone();
  //medianBlur(src, blur, 7);
  Mat result_FG(src_FG.rows, src_FG.cols, CV_8UC1);
  int sum;
  for(int y = 0;y < src_FG.rows; y++)
  {
    for(int x = 0; x<src_FG.cols; x++)
    {
        Vec3b color = src_FG.at<Vec3b>(Point(x,y));
          if( (color[0]>220) || (color[1]>220) || (color[2]>220))
            {
              sum = 255;
            }
         else
            {
              int bg = color[0]-color[1];
              int br = color[0]-color[2];
              int gr = color[1]-color[2];
              sum = (abs(bg) + abs(br) + abs(gr));
            }
        result_FG.at<uchar>(Point(x,y)) = sum;
    }
  }
  threshold(result_FG, blur, 35, 255, THRESH_BINARY);
  medianBlur(blur, blur, 15);
  //showImage(blur, "thresholded image");
  return blur;
}

vector<Vec2f> HoughClassic (Mat src_HC)
{
  // Filter the image.
  Mat src_HC_roi_filtered = src_HC.clone();
  medianBlur(src_HC, src_HC_roi_filtered, 5);

  Mat contours = src_HC_roi_filtered.clone();
  //Run Canny. Parameter to be determined.
  Canny(src_HC_roi_filtered, contours, 30, 50);
  showImage(contours, "canny image");
  Mat draw_detected_hough = src_HC.clone();
  vector<Vec2f> lines_HC;
  //Do HoughTransform.
  houghTransform(contours, draw_detected_hough, lines_HC, 80);
  return lines_HC;
}