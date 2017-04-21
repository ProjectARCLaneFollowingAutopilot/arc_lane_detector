#pragma once
/* DESCRIPTION
 This class can be used to create an object, which will take an input image (perspectively distorted)
and return a non-distorted image projected on the ground plane.
In order to work, the position and orientation of the camera w.r.t the ground plane has to be known.
*/

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
};

class IPM
{
  public:

  // PUBLIC MEMBER METHODS.
  // Default constructor.
  IPM();
  // Default destructor.
  ~IPM();
  // Method to set parameters, such as extrinsic and intrinsic camera parameters.
  // Method to set a new input image.
  // Method which does IPM and returns undistorted, projected image.

  // PUBLIC MEMBER VARIABLES.

  connection_struct lolol;


  private:

  // Private member methods.

  // Private member variables.
  // Input image, which is perspectively distorted.
  // Output image, which is undistorted and projected on the ground plane.
  // Height of camera above World-Origin/Ground-Plane (in meter).
  // Pitch angle of camera w.r.t vertical link Camera<->World (in degree).
  // Focal length of the camera (in pixels), assume that fx=fy.
  // Width of input image (in pixels).
  // Height of input image (in pixels).
  // Connection Matrix which has the same dimensions as input image (for example 640 480), and stores structs.




};

/* Optionally the following methods/variables can be implemented at a later time:
// HELPER (NON ESSENTIAL).
// Constructor which takes an image and camera parameters (extrinsic and intrinsic) as input.
// Method which returns input image.
// Method which returns output image.
// Method which displays input image.
// Method which displays ipm'd image.

// Variable which define ROI-pixels.


*/
