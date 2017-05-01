#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/String.h"
#include "../include/inverse_perspective_mapping/inverse_perspective_mapping.hpp"
using namespace cv;
using namespace std;
//Robins functions.
void showImage(Mat show, string name)
{
	//Function to shwo an Image in a new window.
	namedWindow(name, CV_WINDOW_AUTOSIZE );
	imshow(name , show );
  	waitKey(1);
}
void saveImage(const sensor_msgs::Image::ConstPtr& incoming_image, Mat &src)
{	//Saves incoming images.
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr=cv_bridge::toCvCopy(incoming_image, sensor_msgs::image_encodings::BGR8);
	src=cv_ptr->image;
}
void houghTransform(Mat contours, Mat &src)
{
	//Does the Hough-Transform and draw the lines.
	vector<Vec2f> lines;
 	HoughLines(contours, lines, 1, CV_PI/180, 90, 0, 0); //Parameter to determine.
	for(size_t i=0; i<lines.size(); i++)
	{
		float rho=lines[i][0];
		float theta=lines[i][1];
		cout << i << "   " <<theta <<endl;
		Point pt1;
		Point pt2;
		double a=cos(theta);
		double b=sin(theta);
		double x0=a*rho;
		double y0=b*rho;
		pt1.x = cvRound(x0 + 1000*(-b));
		pt1.y = cvRound(y0 + 1000*(a));
		pt2.x = cvRound(x0 - 1000*(-b));
		pt2.y = cvRound(y0 - 1000*(a));
		line(src, pt1, pt2, Scalar(0, 255, 0), 3, CV_AA);
	}
}
// Callback function.
void ipmCallback(const sensor_msgs::Image::ConstPtr& incoming_image)
{
	//Save the incoming image in an Mat-element and flip it.
	Mat src_flipped;
	Mat src;
	saveImage(incoming_image, src_flipped);
	flip(src_flipped, src, 0);
	//Tryout median filter.
	Mat median;
	medianBlur(src, median, 15); //Parameter to determine.
	showImage(median, "median");
	//Tryout gaussian filter.
	Mat gaussian;
	Size size(23, 23);
	GaussianBlur(src, gaussian, size, 15, 3, 0); //Several parameters to determine.
	showImage(gaussian, "gaussian");
	//Minimise the image to ROI and show it.
	Mat src_ROI=src;
	int x=0;
	int y=220;
	int w=src_flipped.cols;
	int h=src_flipped.rows-y;
	Rect region=Rect(x, y, w, h);
	Mat ROI=median(region); //Gaussian oder median to determine.
	showImage(ROI, "ROI");
	rectangle(src_ROI, region, Scalar( 0, 255, 0 ) , 3, CV_AA);
	showImage(src_ROI, "Ausschnitt");
	//Canny-Edge-Detection.
	Mat contoursInv=ROI;
	Canny(ROI, contoursInv, 30, 50); //Parameter to determine.
	Mat contours=ROI;
	threshold(contoursInv,contours,128,255,THRESH_BINARY_INV);
	showImage(contours, "Konturen");
	Mat show=ROI;
	//Hough-Transform.
	houghTransform(contoursInv, show);
	showImage(show, "Linien");
}
int main(int argc, char* argv[])
{
  // Initialize the node.
  ros::init(argc, argv, "test_robin_node");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/usb_cam/image_raw", 10, ipmCallback);
  ros::spin();
  return 0;
}
