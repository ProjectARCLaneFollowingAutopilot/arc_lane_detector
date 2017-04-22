#include "../include/inverse_perspective_mapping/inverse_perspective_mapping.hpp"

// Non class methods
// Callback function to detect if a pixel was clicked and to return the clicked on pixel.
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
  this->output_img_ = (this->input_img_).clone();

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
  std::cout<<"IPM got called"<<std::endl;
  cv::warpPerspective(this->input_img_, this->output_img_, this->perspective_transform_, (this->output_img_).size());

  if(! input_img_.data )
  {
    std::cout <<  "Could not open or find the input image" << std::endl ;
  }

  if(! input_img_.data )
  {
    std::cout <<  "Could not open or find the output image" << std::endl ;
  }

  std::cout<<"Image was warped"<<std::endl;
  cv::namedWindow("Input", CV_WINDOW_AUTOSIZE);
  cv::namedWindow("Output", CV_WINDOW_AUTOSIZE);
  std::cout<<"Windows were created"<<std::endl;
  cv::imshow("Input", this->input_img_);
  cv::imshow("Output", this->output_img_);
  cv::waitKey(0);
}

// Private member methods.

// Method which prompts the user to calibrate the transformation matrix by assigning four pixel each in an image.
void IPM::setTransformationMatrix()
{
  // Prompt user input.
  // Open a window.
  // Show image (perspective) with four corners on a rectangle.
  cv::Point2f p;
  cv::namedWindow("Display", CV_WINDOW_AUTOSIZE);

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
  this->dst_points_[0] = diagonal_elements[0];
  (this->dst_points_[1]).x = diagonal_elements[1].x;
  (this->dst_points_[1]).y = diagonal_elements[0].x;
  (this->dst_points_[2]).x = diagonal_elements[0].x;
  (this->dst_points_[2]).x = diagonal_elements[1].y;
  this->dst_points_[3] = diagonal_elements[1];

  // From this get the transformation matrix and then store it.
  this->perspective_transform_ = cv::getPerspectiveTransform(this->src_points_, this->dst_points_);
  std::cout<<perspective_transform_<<std::endl;

}
