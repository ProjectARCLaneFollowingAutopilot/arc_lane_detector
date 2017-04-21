#pragma once
/* DESCRIPTION
 This class can be used to create an object, which will take an input image (perspectively distorted)
and return a non-distorted image projected on the ground plane.
In order to work, the position and orientation of the camera w.r.t the ground plane has to be known.
*/
#include <cv.h>
#include "opencv2/highgui/highgui.hpp"
#include <iostream>

struct connection_struct
{
  int input_u;
  int input_v;
  int red;
  int green;
  int blue;
  float x_ground;
  float y_ground;
  float z_ground;
  int index;
};

class IPM
{
  public:

  // PUBLIC MEMBER METHODS.
  // Default constructor.
  IPM();
  // Default destructor.
  ~IPM();
  // Method to set parameters, such as extrinsic and intrinsic camera parameters and then adjusts the connection_matrix_.
  void setParam(float camera_height_m, float pitch_angle_deg, int focal_length_px, int input_width_px, int input_height_px);
  // Method to set a new input image.
  // Method which does IPM and returns undistorted, projected image.

  // PUBLIC MEMBER VARIABLES.


  private:

  // Private member methods.

  // Private member variables.
  // Input image, which is perspectively distorted.
  cv::Mat input_img_;
  // Output image, which is undistorted and projected on the ground plane.
  cv::Mat output_img_;
  // Height of camera above World-Origin/Ground-Plane (in meter).
  float camera_height_m_;
  // Pitch angle of camera w.r.t vertical link Camera<->World (in degree).
  float pitch_angle_deg_;
  // Focal length of the camera (in pixels), assume that fx=fy.
  int focal_length_px_;
  // Width of input image (in pixels).
  int input_width_px_;
  // Height of input image (in pixels).
  int input_height_px_;
  // Number of pixels in input image.
  int num_px_;
  // Pointer to Connection Matrix which has the same dimensions as input image (for example 640 480), and stores structs.
  connection_struct* connection_vector_;

};
