/* This is the final executable for running lane tracking:
1. Initialise everything: LD-object, IPM-object, RANSAC-object,... 
2. Subscribe to webcam.
3. Using LD-Class, get many lines.
4. Filter out to only find two lines.
5. Find points around line and run RANSAC.
6. Using IPM, transform found polynomial from RANSAC.
*/
#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "../../arc_lane_tracking_tools/include/inverse_perspective_mapping/inverse_perspective_mapping.hpp"
#include "../../arc_lane_tracking_tools/include/line_detection/line_detection.hpp"

using namespace std;
using namespace cv;

// CONSTANTS:

// Camera properties.
float camera_height_m = 1.3;
float pitch_angle_deg = 90.0;
float focal_length_px;
int input_width_px = 640;
int input_height_px = 480;
// Line Detection properties.
Point2f roi_left_top(0, 250); 
Point2f roi_right_bottom(640, 420);

// VARIABLES:
IPM ipm_object;
LineDetector ld_object;
Mat source;
bool flipping = true;

// Callback function for subscribing to webcam.
void webcamCallback(const sensor_msgs::Image::ConstPtr& incoming_image);

int main(int argc, char *argv[])
{

	// Set parameters of LD and IPM objects.
  	ipm_object.IPM::setParam(camera_height_m, pitch_angle_deg, focal_length_px, input_width_px, input_height_px);
 	  ld_object.LineDetector::setParams(roi_left_top, roi_right_bottom);

  	// Initialize ROS Node.
  	ros::init(argc, argv, "Lane_Detector");
  	ros::NodeHandle n;

  	// Declare callback function to get Webcam image.
 	ros::Subscriber sub = n.subscribe("/usb_cam/image_raw", 10, webcamCallback);

 	// Run the callback functions.
 	ros::spin();

	return 0;
}

void webcamCallback(const sensor_msgs::Image::ConstPtr& incoming_image)
{
  // NOCH NICHT GEMACHT: Clear things up for new iteration.
	// Fetch image.
	// Pointer to copy the converted sensor message to.
	cv_bridge::CvImagePtr cv_ptr;
	// Save incoming image.
    cv_ptr = cv_bridge::toCvCopy(incoming_image, sensor_msgs::image_encodings::BGR8);
  	// Check for valid image.
  	if (!(cv_ptr->image).data)
  	{
      std::cout<<"Got bad image from webcam!"<<std::endl;
  	}
  	// Flip (if needed) and save the image to global variable.
  	if(flipping)
  	{
  		cv::flip(cv_ptr->image, source, -1);
  	}
  	else
  	{
  		source = cv_ptr->image.clone();
  	}

    // Give it to LD to get two lines.
    ld_object.LineDetector::setImage(source);
    ld_object.LineDetector::doLineDetection();
    // Get the coordinates (LB, LT, RB, RT) of the two lines.
    vector<Point2f> line_points = ld_object.LineDetector::getLineCoordinates();

    // Transform the line coordinates using the IPM object.
    vector<Point2f> left_line_world;
    vector<Point2f> right_line_world;
    left_line_world.push_back(ipm_object.IPM::image2Local(line_points[0]));
    left_line_world.push_back(ipm_object.IPM::image2Local(line_points[1]));
    right_line_world.push_back(ipm_object.IPM::image2Local(line_points[2]));
    right_line_world.push_back(ipm_object.IPM::image2Local(line_points[3]));

    // Visualise the lines.
    visualiseTopView(left_line_world, right_line_world);
}

void visualiseTopView(vector<Point2f> left_line_world, vector<Point2f> right_line_world)
{
  int scale = 5;
  Mat visualise;
  visualise = imread("/home/nikhilesh/catkin_ws/src/arc_lane_following_autopilot/topview.jpg");
  //Point top left.
  Point2f draw_top_left((450 - (left_line_world[1].y * 30)), (650 - (left_line_world[1].x * 30)));
  cout<<"Grösse vom Bild "<<visualise.cols<<" "<< visualise.rows<<endl;
  //Point bottom left.
  Point2f draw_bottom_left((450 - (left_line_world[0].y * 30)), (650 - (left_line_world[0].x * 30)));
  //Point top right.
  Point2f draw_top_right((450 - (right_line_world[1].y * 30)), (650 - (right_line_world[1].x * 30)));
  //Point bottom right. 
  Point2f draw_bottom_right((450 - (right_line_world[0].y * 30)), (650 - (right_line_world[0].x * 30)));
  //Calculations for the left line.
  float draw_w_left = ((draw_top_left.y - draw_bottom_left.y)/(draw_top_left.x - draw_bottom_left.x));
  float draw_u_left = ((draw_top_left.y - draw_w_left*draw_top_left.x));
  float draw_y_left_desired_top = 400;
  float draw_y_left_desired_bottom = 580;
  float draw_x_left_desired_bottom = (draw_y_left_desired_bottom-draw_u_left)/draw_w_left;
  float draw_x_left_desired_top = (draw_y_left_desired_top - draw_u_left)/draw_w_left;
  Point2f draw_top_left_desired(draw_x_left_desired_top, draw_y_left_desired_top);
  Point2f draw_bottom_left_desired(draw_x_left_desired_bottom, draw_y_left_desired_bottom);
  //Draw left line.
  if(draw_left)
  {
      line(visualise, draw_bottom_left_desired, draw_top_left_desired, Scalar(255, 255, 255), 3);
  }
  //Calculations for the right line.
  float draw_w_right = ((draw_top_right.y-draw_bottom_right.y)/(draw_top_right.x-draw_bottom_right.x));
  float draw_u_right = ((draw_top_right.y - draw_w_right*draw_top_right.x));
  float draw_y_right_desired_top = 400;
  float draw_y_right_desired_bottom = 580;
  float draw_x_right_desired_bottom = (draw_y_right_desired_bottom-draw_u_right)/draw_w_right;
  float draw_x_right_desired_top = (draw_y_right_desired_top-draw_u_right)/draw_w_right;
  Point2f draw_top_right_desired(draw_x_right_desired_top, draw_y_right_desired_top);
  Point2f draw_bottom_right_desired(draw_x_right_desired_bottom, draw_y_right_desired_bottom);
  //Draw right line.
  if(draw_right)
    {
      line(visualise, draw_bottom_right_desired, draw_top_right_desired, Scalar(255, 255, 255), 3);
    }
  putText(visualise, "Relative Lateral Error", Point(320, 290), CV_FONT_HERSHEY_DUPLEX, 1, Scalar(0, 255, 0), 2);
  //Draw the middle lines. 
  if (draw_left && draw_right)
  {
    line(visualise, draw_bottom_right_desired, draw_bottom_left_desired, Scalar(0, 255, 0), 2);
    //arrowedLine(visualise, Point(450, 590), Point(450, 490), Scalar(0, 255, 0), 2);
    //Write the relative error. 
    float draw_distance = draw_x_right_desired_bottom - draw_x_left_desired_bottom;
    float draw_rel_error_right = (draw_x_right_desired_bottom-450)/draw_distance;
    float draw_rel_error_left = (450-draw_x_left_desired_bottom)/draw_distance;
    ostringstream ss_left;
    ss_left << "left: " <<draw_rel_error_left << " %";
    string draw_text_left(ss_left.str());
    putText(visualise, draw_text_left, Point(320, 330), CV_FONT_HERSHEY_DUPLEX, 1, Scalar(0, 255, 0), 1);
    ostringstream ss_right;
    ss_right << "right:" <<draw_rel_error_right << " %";
    string draw_text_right(ss_right.str());
    putText(visualise, draw_text_right, Point(320, 360), CV_FONT_HERSHEY_DUPLEX, 1, Scalar(0, 255, 0), 1);
  }
  //Draw the coordinate system;
  int draw_origin_cs_x = 500+30;
  int draw_origin_cs_y = 733-56;
  //arrowedLine(visualise, Point(draw_origin_cs_x, draw_origin_cs_y), Point(draw_origin_cs_x, draw_origin_cs_y-15), Scalar(0,0, 255), 2);
  //arrowedLine(visualise, Point(draw_origin_cs_x,draw_origin_cs_y), Point(draw_origin_cs_x-15, draw_origin_cs_y), Scalar(0, 0, 255), 2);
  putText(visualise, "x", Point(draw_origin_cs_x+5, draw_origin_cs_y-13), CV_FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 2);
  putText(visualise, "y", Point(draw_origin_cs_x-15, draw_origin_cs_y+16), CV_FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 2);
  imshow("visalisierung", visualise);
}