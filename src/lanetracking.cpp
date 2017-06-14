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
#include "std_msgs/String.h"
#include "../../arc_lane_tracking_tools/include/inverse_perspective_mapping/inverse_perspective_mapping.hpp"
#include "../../arc_lane_tracking_tools/include/line_detection/line_detection.hpp"
#include "../../arc_lane_tracking_tools/include/ransac_fitting/ransac_fitting.hpp"

using namespace std;
using namespace cv;

// Global variables: IPM, LD, RANSAC, fetched image,...
IPM ipm_object;
LineDetector ld_object;
Ransac ransac_object;
Mat source;
vector<float> left_polynomial;
vector<float> right_polynomial;

// Callback function for subscribing to webcam.
void webcamCallback(const sensor_msgs::Image::ConstPtr& incoming_image);

int main(int argc, char *argv[])
{
	// NOCH NICHT GEMACHT: Set parameters of LD, IPM, RANSAC objects.
  	ipm_object.IPM::setParam(float camera_height_m, float pitch_angle_deg, float focal_length_px, int input_width_px, int input_height_px);
 	ld_object.LineDetector::setParams(Point2f roi_left_top, Point2f roi_right_bottom, Vec2f default_left, Vec2f default_right);
 	ransac_object.Ransac::setRansacParams(float max_inlier_distance, int max_num_of_iterations, int min_size_consensus);

  	// Initialize ROS Node.
  	ros::init(argc, argv, "lane_tracker");
  	ros::NodeHandle n;

  	// Declare callback function to get Webcam image.
 	ros::Subscriber sub = n.subscribe("/usb_cam/image_raw", 10, webcamCallback);

 	// Run that shit.
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
  	//cv::flip(cv_ptr->image, src, -1);
  	source = cv_ptr->image.clone();

	// NOCH NICHT GEMACHT: Give it to LD to get lines.
	ld_object.LineDetector::setImage(source);
	ld_object.LineDetector::doLineDetection();

	// NOCH NICHT GEMACHT: Find points around two lines.
	vector<Point2f> left_points = ld_object.LineDetector::pointsAroundLeftLineOrig();
	vector<Point2f> right_points = ld_object.LineDetector::pointsAroundRightLineOrig();

	// Give the point cloud to RANSAC object to fit polynomial.
	ransac_object.Ransac::setRansacDataSet(left_points);
	left_polynomial = ransac_object.Ransac::getRansacCoeff();
	ransac_object.Ransac::setRansacDataSet(right_points);
	right_polynomial = ransac_object.Ransac::getRansacCoeff();
	// NOCH NICHT GEMACHT: Sample the left and right polynomials to find four sample points on each polynomial.	
	vector<Point2f> sample_of_fit_left_image;
	vector<Point2f> sample_of_fit_right_image;
	// Transform found polynomial, using IPM.
	vector<Point2f> sample_of_fit_left_world;
	vector<Point2f> sample_of_fit_right_world;
	for(int i = 0; i < sample_of_fit_left_world.size(); i++)
	{
		sample_of_fit_left_world.push_back(ipm_object.IPM::image2Local(sample_of_fit_left_image[i]));
		sample_of_fit_right_world.push_back(ipm_object.IPM::image2Local(sample_of_fit_right_image[i]));
	}
	// NOCH NICHT GEMACHT: Return the polynomial coefficients of the left and right polynomial in the world frame.
}
