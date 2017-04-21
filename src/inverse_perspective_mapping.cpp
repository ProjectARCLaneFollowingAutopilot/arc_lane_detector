#include "../include/inverse_perspective_mapping/inverse_perspective_mapping.hpp"

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

// Method to set parameters, such as extrinsic and intrinsic camera parameters and then adjusts the connection_matrix_.
void IPM::setParam(float camera_height_m, float pitch_angle_deg, int focal_length_px, int input_width_px, int input_height_px)
{
  this->camera_height_m_ = camera_height_m;
  this->pitch_angle_deg_ = pitch_angle_deg;
  this->focal_length_px_ = focal_length_px;
  this->input_width_px_ = input_width_px;
  this->input_height_px_ = input_height_px;
  this->num_px_ = (this->input_height_px_)*(this->input_width_px_);

  //delete[] connection_vector_;
  //connection_vector_ = new connection_struct[this->num_px_];

}

// Method to set a new input image.
void IPM::getImage(cv::Mat src)
{
  this->input_img_ = src;
}

// Method which does IPM and returns undistorted, projected image.
cv::Mat IPM::invPerspectiveMapping()
{

}
