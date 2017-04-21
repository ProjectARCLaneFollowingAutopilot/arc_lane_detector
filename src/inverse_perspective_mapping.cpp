#include "../include/inverse_perspective_mapping/inverse_perspective_mapping.hpp"

// Default constructor.
IPM::IPM()
{
  std::cout<<"IPM object created!"<<std::endl;
  connection_vector_ = new connection_struct[2];
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

  delete[] connection_vector_;
  connection_vector_ = new connection_struct[this->num_px_];

}

// Method to set a new input image.
void IPM::getImage(cv::Mat src)
{
  this->input_img_ = src;
}

// Method which does IPM and returns undistorted, projected image.
cv::Mat IPM::invPerspectiveMapping()
{
  // Iterate trough all pixel points of the input image to generate the connection_vector_.
  int counter = 0;
  for(int i_rows = 0; i_rows<input_height_px_; i_rows++)
  {
    for(int i_cols; i_cols<input_width_px_; i_cols++)
    {
      // Set Index in connection_vector_.
      // Set RGB values of input image in connection_vector_.
      this->connection_vector_[counter].blue = (int)(this->input_img_).at<cv::Vec3b>(i_rows,i_cols)[0];
      this->connection_vector_[counter].green = (int)(this->input_img_).at<cv::Vec3b>(i_rows,i_cols)[1];
      this->connection_vector_[counter].red = (int)(this->input_img_).at<cv::Vec3b>(i_rows,i_cols)[2];
      // Set pixel position of corresponding input image in connection_vector_.
      this->connection_vector_[counter].input_u = i_cols;
      this->connection_vector_[counter].input_v = i_rows;
      // Find lambda from equation 7.
      // Find and set x_ground, y_ground, z_ground=0 in connection_vector_.
    }

  }

  // Discretize the points from connection_vector_ onto a new image.

}
