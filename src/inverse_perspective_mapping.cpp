#include "../include/inverse_perspective_mapping/inverse_perspective_mapping.hpp"

// Public member methods.
// Default constructor.
IPM::IPM()
{
  std::cout<<"IPM object created!"<<std::endl;
}

// Default destructor.
IPM::~IPM()
{
  std::cout<<"IPM object destroyed!"<<std::endl;
}

// Method to set parameters, such as extrinsic and intrinsic camera parameters and then calculates and stores the projection matrix.
void IPM::setParam(float camera_height_m, float pitch_angle_deg, float focal_length_px, int input_width_px, int input_height_px)
{
  this->camera_height_m_ = camera_height_m;
  this->pitch_angle_deg_ = pitch_angle_deg;
  this->focal_length_px_ = focal_length_px;
  this->input_width_px_ = input_width_px;
  this->input_height_px_ = input_height_px;

  this->setTransformationMatrix();
}

// Method to set a new input image.
void IPM::getImage(cv::Mat src)
{
  this->input_img_ = src;
}

// Method which does IPM and returns undistorted, projected image.
cv::Mat IPM::invPerspectiveMapping()
{
  // Run cv::perspectiveProjection to get transformed image.
}

// Private member methods.

// Method which prompts the user to calibrate the transformation matrix by assigning four pixel each in an image.
void IPM::setTransformationMatrix()
{
  // Prompt user input.
  // Show image (perspective) with four corners on a rectangle.
  // Store the clicked on pixels.
  // Show image of same dimensions as input image.
  // User shall click on two points (diagonal), from which rectangle will be found and pixels stored.
  // From this get the transformation matrix and then store it.
}
