#include "../include/inverse_perspective_mapping/inverse_perspective_mapping.hpp"

// GENERAL FUNCTIONS.
// Callback function for setMouseCallback and returns the point clicked on.
void getClickedPixel(int event, int x, int y, int flags, void *ptr)
{
  if(event == cv::EVENT_LBUTTONDOWN)
  {
    std::cout<<"CLICK"<<std::endl;
    cv::Point2f *p = (cv::Point2f*)ptr;
    p->x = x;
    p->y = y;
  }
}

// PUBLIC MEMBER METHODS.
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

// Method to set a new input image.
void IPM::getImage(cv::Mat src)
{
  this->input_img_ = src;
  this->output_img_ = (this->input_img_).clone();
}

// Method to set parameters, such as extrinsic and intrinsic camera parameters and then finds the homography matrix.
void IPM::setParam(float camera_height_m, float pitch_angle_deg, float focal_length_px, int input_width_px, int input_height_px)
{
  // Set intrinsic and extrinsic camera parameters.
  this->camera_height_m_ = camera_height_m;
  this->pitch_angle_deg_ = pitch_angle_deg;
  this->focal_length_px_ = focal_length_px;
  this->input_width_px_ = input_width_px;
  this->input_height_px_ = input_height_px;

  std::cout<<"Width in Pixel "<<input_width_px<<std::endl;
  std::cout<<"Height in Pixel "<<input_height_px<<std::endl;

  // The user has to choose four points in the input image which definately are on the ground plane.
  this->setCtrlPts();

  // Calculate the homography matrix.
  //this->setTransformationMatrix();
  this->setTransformationMatrix(1);
}

// Method to prompt the user to set input control points.
void IPM::setCtrlPts()
{
  // Let user select the input points.
  cv::Point2f p;
  cv::namedWindow("Display", CV_WINDOW_AUTOSIZE);

  // Prompt user to select four points in perspectively distorted input image of the groundplane.
  std::cout<<"Selecting Input Points on ground plane"<<std::endl;
  for(int i=0; i<4; i++)
  {
    std::cout<<"Point: "<<i+1<<" out of "<<4<<std::endl;
    std::cout<<"You have now 5 sec to click on your point"<<std::endl;
    cv::imshow("Display", input_img_);
    cv::setMouseCallback("Display", getClickedPixel, &p);
    cv::waitKey(5000);
    std::cout<<"Saved pixels: "<<std::endl;
    std::cout<<p<<std::endl;
    this->src_points_[i] = p;
    std::cout<<"Input Points saved!"<<std::endl;
  }
  // setMouseCallback(windowname,NULL);
}

// Method which does IPM and returns undistorted, projected image.
cv::Mat IPM::invPerspectiveMapping()
{
  // Run cv::perspectiveProjection to get transformed image.
  cv::warpPerspective(this->input_img_, this->output_img_, this->perspective_transform_, (this->output_img_).size());

  // Display both images.
  //cv::namedWindow("Input", CV_WINDOW_AUTOSIZE);
  //cv::namedWindow("Output", CV_WINDOW_AUTOSIZE);
  //cv::imshow("Input", this->input_img_);
  //cv::imshow("Output", this->output_img_);
  //cv::waitKey(50);

  return this->output_img_;
}

// PRIVATE MEMBER METHODS.
// Method which prompts the user to calibrate the transformation matrix by assigning four pixel each in an image.
void IPM::setTransformationMatrix()
{
  cv::Point2f p;
  cv::namedWindow("Display", CV_WINDOW_AUTOSIZE);

  // Prompt user to select four points in perspectively distorted input image.
  std::cout<<"Selecting Input Points"<<std::endl;
  for(int i=0; i<4; i++)
  {
    std::cout<<"Point: "<<i+1<<" out of "<<4<<std::endl;
    std::cout<<"You have now 5 sec to click on your point"<<std::endl;
    cv::imshow("Display", input_img_);
    cv::setMouseCallback("Display", getClickedPixel, &p);
    // cv::setMouseCallback("Display", NULL, NULL);
    cv::waitKey(5000);
    std::cout<<"Saved pixels: "<<std::endl;
    std::cout<<p<<std::endl;
    this->src_points_[i] = p;
  }

  std::cout<<"Input Points saved!"<<std::endl;

  cv::Point2f diagonal_elements[2];

  // Prompt user to select two points to generate a rectangle (in destination image).
  std::cout<<"Selecting Output Points"<<std::endl;
  for(int i=0; i<2; i++)
  {
    std::cout<<"Point: "<<i+1<<" out of "<<2<<std::endl;
    std::cout<<"You have now 5 sec to click on your point"<<std::endl;
    cv::imshow("Display", output_img_);
    cv::setMouseCallback("Display", getClickedPixel, &p);
    cv::waitKey(5000);
    std::cout<<"Saved pixels: "<<std::endl;
    std::cout<<p<<std::endl;
    diagonal_elements[i] = p;
  }

  // Assign the destination points from the two diagonal points, the user selected.
  // Use a square template, which is easier to calibrate.
  this->dst_points_[0] = diagonal_elements[0];
  (this->dst_points_[1]).x = diagonal_elements[1].x;
  (this->dst_points_[1]).y = diagonal_elements[0].x;
  (this->dst_points_[2]).x = diagonal_elements[0].x;
  (this->dst_points_[2]).y = diagonal_elements[1].y;
  this->dst_points_[3] = diagonal_elements[1];

  std::cout<<"Output Points saved!"<<std::endl;

  // Knowing four points in each image, calculate the transformation matrix.
  this->perspective_transform_ = cv::getPerspectiveTransform(this->src_points_, this->dst_points_);
}

// Method which uses four predefined points on the input image, uses equation (6) to project and then gets and sets the transformation matrix.
void IPM::setTransformationMatrix(bool some_variable)
{
  std::cout<<"New method was called"<<std::endl;
  // Variable which stores the actual cartesian coordinates of the input points' projection on the ground plane. In world coordinates!
  cv::Point2f dst_points_cartesian[4];
  float x_ground = 0.0;
  float y_ground = 0.0;
  float alpha_rad = ((this->pitch_angle_deg_)/180.0)*PI;
  // Project each pixel point, the user chose onto the ground plane (in the world frame).
  for(int i = 0; i<4; i++)
  {
    // Get pixel coordinates centred at the image centre and convention as used when deriving the formulas (u to the right, v to the top).
    float u = (this->src_points_[i]).x - (this->input_width_px_)/2.0;
    float v =  (-1.0)*((this->src_points_[i]).y - (this->input_height_px_)/2.0);

    // Find lambda from equation (7).
    float lambda = (this->camera_height_m_)/((this->focal_length_px_)*cos(alpha_rad) -  v*sin(alpha_rad));
    // Project pixel from src_points_ to x,y (in world frame) onto ground plane (z = 0) using equation (6) and save to dst_points_cartesian.
    x_ground = 0.0 + lambda*(v*cos(alpha_rad) + (this->focal_length_px_)*sin(alpha_rad));
    y_ground = 0.0 - lambda*u;
    dst_points_cartesian[i].x = x_ground;
    dst_points_cartesian[i].y = y_ground;
  }

  // Find x_min, x_max, y_min, y_max in dst_points_cartesian-->Boundaries of image equivalent to image dimensions in px.
  float x_min_cart = 1000.0;
  float x_max_cart = -1000.0;
  float y_min_cart = 1000.0;
  float y_max_cart = -1000.0;
  for(int i = 0; i<4; i++)
  {
    if(dst_points_cartesian[i].x < x_min_cart)
    {
      x_min_cart = dst_points_cartesian[i].x;
    }
    if(dst_points_cartesian[i].x > x_max_cart)
    {
      x_max_cart = dst_points_cartesian[i].x;
    }
    if(dst_points_cartesian[i].y < y_min_cart)
    {
      y_min_cart = dst_points_cartesian[i].y;
    }
    if(dst_points_cartesian[i].y > y_max_cart)
    {
      y_max_cart = dst_points_cartesian[i].y;
    }
  }

  // Get the resolution of the output image (pixel/cm). Idea is, that the calibration points were chosen, such that they form
  // a quadrilateral which is around the ROI (street). Then a rectangle is fitted using the extremas of the quadrilateral.
  float y_res = (this->input_width_px_)/(2*(y_max_cart - y_min_cart));              //this->input_width_px_/(y_max_cart - y_min_cart);
  float x_res = this->input_height_px_/(2*(x_max_cart - x_min_cart));              //this->input_height_px_/(x_max_cart - x_min_cart);

  // World coordinates of the image origin (top left pixel).
  float x_origin = x_min_cart + 480.0/x_res;
  float y_origin = 320.0/y_res;

  // Find to which pixel the points from dst_points_cartesian correspond to and assign them to dst_points_.
  for(int i = 0; i<4; i++)
  {
    this->dst_points_[i].x = trunc(y_res*(std::abs(y_origin - dst_points_cartesian[i].y)));      //trunc(y_res*(std::abs(y_max_cart - dst_points_cartesian[i].y)));
    this->dst_points_[i].y = trunc(x_res*(std::abs(x_origin - dst_points_cartesian[i].x)));                 //trunc(x_res*(std::abs(x_max_cart - dst_points_cartesian[i].x)));
  }

  // Knowing four points in each image, calculate the transformation matrix (image plane --> image plane).
  this->perspective_transform_ = cv::getPerspectiveTransform(this->src_points_, this->dst_points_);
}


/* Quellenangabe
Die Idee IPM zu verwenden kam zuallererst vom Paper: Real Time Detection of Lane Markers.

Das Paper Low-level Image Processing
for Lane Detection and Tracking hat mich auf die Idee gebracht Kalibrierungspunkte auf der Strasse zu wählen (war schon klar),
aber diese nicht als Berandung für das IPM'ed Bild zu wählen sondern, dass diese recht wenig Platz einnehmen im Zielbild.
Dadurch sieht man auf dem IPM'd Bild sehr viel von der Umgebung, wenn auch verzerrt.
*/
