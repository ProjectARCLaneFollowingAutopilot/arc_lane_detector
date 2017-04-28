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

  return 0;
}
