
#include <fstream>
#include <iostream>
#include <cv.h>
#include <vector>
#include "opencv2/highgui/highgui.hpp"
#include "../../arc_lane_tracking_tools/include/ransac_fitting/ransac_fitting.hpp"

using namespace cv;

int main()
{

	Ransac test_kurve;

	// TEST.
	// Read in the generated data set.
	vector<Point2f> datenset;
	std::ifstream read_in;
	read_in.open("/home/nikhilesh/DataTextFiles/data.txt");
	while(read_in)
	{
		float x;
		float y_noise;
		read_in>>x>>y_noise;
		Point2f temp_point(x, y_noise);
		datenset.push_back(temp_point);
	}
	read_in.close();

	// Assign the data set.
	test_kurve.Ransac::setRansacDataSet(datenset);
	// Set the RANSAC parameters.
	test_kurve.Ransac::setRansacParams(0.5, 20, 3);
	// Get the coefficients.
	vector<float> found_coeffs = test_kurve.Ransac::getRansacCoeff();
	std::cout<<"a: "<<found_coeffs[0]<<std::endl;
	std::cout<<"b: "<<found_coeffs[1]<<std::endl;
	std::cout<<"c: "<<found_coeffs[2]<<std::endl;
	
	
	return 0;
}