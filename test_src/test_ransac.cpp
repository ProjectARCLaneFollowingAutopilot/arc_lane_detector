
#include <fstream>
#include <iostream>
#include <cv.h>
#include <vector>
#include "opencv2/highgui/highgui.hpp"
#include "../../arc_lane_tracking_tools/include/ransac_fitting/ransac_fitting.hpp"

using namespace cv;

int main(int argc, char *argv[])
{

	Ransac test_kurve;

	// TEST.
	// Read in the generated data set.
	vector<Point2f> datenset;
	std::ifstream read_in;
	read_in.open("/home/nikhilesh/DataTextFiles/RobinAlgo/c_oben.txt");
	while(read_in)
	{
		float x;
		float y_noise;
		read_in>>x>>y_noise;
		y_noise = -y_noise;
		Point2f temp_point(x, y_noise);
		datenset.push_back(temp_point);
	}
	read_in.close();

	// Assign the data set.
	test_kurve.Ransac::setRansacDataSet(datenset);
	// Set the RANSAC parameters.
	test_kurve.Ransac::setRansacParams(15, 50, 4);	// 5
	// Get the coefficients.
	// t for measuring the performance of the algorithm.
	double t = (double)getTickCount();

	vector<float> found_coeffs = test_kurve.Ransac::getRansacCoeff();
	std::cout<<"a: "<<found_coeffs[0]<<std::endl;
	std::cout<<"b: "<<found_coeffs[1]<<std::endl;
	std::cout<<"c: "<<found_coeffs[2]<<std::endl;
	std::cout<<"d: "<<found_coeffs[3]<<std::endl;

	std::ofstream write_out;
	write_out.open("/home/nikhilesh/DataTextFiles/coeff_lsq.txt");
  	write_out <<found_coeffs[0]<<"\n";
  	write_out <<found_coeffs[1]<<"\n";
  	write_out <<found_coeffs[2]<<"\n";
  	write_out <<found_coeffs[3]<<"\n";
  	write_out.close();

	t = ((double)getTickCount() - t)/getTickFrequency();

	std::cout<<"Time passed in seconds: "<<t<<std::endl;
	
	return 0;
}