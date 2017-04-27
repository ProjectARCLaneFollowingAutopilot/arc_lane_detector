#include <cv.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"


using namespace cv;
using namespace std;

// Global variables.
// Variable to save the incoming image from the webcam globally (maybe not needed).
Mat src_img;
Mat src_img_copy;

int main(int argc, char* argv[])
{
  src_img = cv::imread("/home/nikku/Desktop/Speed.jpg");

	if (!src_img.data)
	{
    cout<<"Failed to load image!"<<endl;
		return -1;
	}

  src_img_copy = src_img.clone();

  for(int y=0;y<src_img.rows;y++)
      {
          for(int x=0;x<src_img.cols;x++)
          {
          Vec3b color = src_img.at<Vec3b>(Point(x,y));

              color[0] = color[0]*1.5;
              color[1] = color[1]*0.5;
              color[2] = color[2]*1.5;
              //cout << "Pixel >200 :" << x << "," << y << endl;
              src_img.at<Vec3b>(Point(x,y)) = color;
          }
      }
  imshow("Original", src_img_copy);
  imshow("Fun", src_img);
  waitKey(0);





/*

  // GOOD COMBINATION 1
  cv::cvtColor(src_img, src_img_gray, CV_BGR2GRAY );
  //threshold(src_img_gray, src_img_gray, 120, 255, THRESH_BINARY);
  medianBlur(src_img_gray, src_img_gray, 5);

  src_img_gray = src_img_gray(Rect(0,240,639,239));

  Canny(src_img_gray, src_img_gray, 40, 130);

  vector<Vec2f> lines;
  // Run hough-line transform on dst_canny to detect lines.
  cv::HoughLines(src_img_gray, lines, 1, CV_PI/180, 150, 0, 0 );
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


  imshow("Canny", src_img_gray);
  waitKey(0);
  cv::imwrite( "/home/nikku/Desktop/Speed_black_and_white_medianblur_canny.jpg", src_img_gray);

*/
  return 0;
}

/* This program will be used to test different imamgeprocessing techniques in.
For example: Filter, color channels, edge detections, gradient operators,...?
*/

/* Used code from:
http://stackoverflow.com/questions/23001512/c-and-opencv-get-and-set-pixel-color-to-mat
*/
