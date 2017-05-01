// LOAD IMAGE FROM PATH.
src_img = cv::imread("/home/nikku/Desktop/Speed.jpg");

// WRITE IMAGE TO PATH.
cv::imwrite("/home/nikku/Desktop/IPM/after_ipm.jpg", after_ipm);

// CALLBACK FUNCTION WHEN USING WEBCAM WITH ROS.
void imageCallback(const sensor_msgs::Image::ConstPtr& incoming_image)
{
  // Create a pointer, where the incoming ros-image gets assigned to.
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(incoming_image, sensor_msgs::image_encodings::BGR8);
}

// CHECK FOR VALID IMAGE.
if (!(src_img).data)
{
  std::cout<<"Failed to load image"<<std::endl;
}

// FLIP IMAGE UPSIDE DOWN AND SAVE TO NEW IMAGE.
cv::flip(src_img, dst_img, 0);

// CONVERT TO GRAYSCALE IMAGE.
cv::cvtColor(src_img, src_img_gray, CV_BGR2GRAY );

// APPLY A MEDIANBLUR FILTER.
cv::medianBlur(src_img, dst_img, 5);

// APPLY GAUSSIANBLUR FILTER.
Size kernel_size(5, 5);
cv::GaussianBlur(src_img, filtered_image, kernel_size, 0.6, 0.6);

// CROP IMAGE.
cropped_image = src_img(Rect(0,170,639,239));

// THRESHOLD AN IMAGE.
threshold(src_img, dst_img, 128, 255, THRESH_BINARY_INV);

// CANNY OPERATOR.
cv::Canny(src_img_gray, edge_image, 40, 130);

// DO HOUGH LINE TRANSFORM AND DRAW LINES.
vector<Vec2f> lines;
cv::HoughLines(src_img_gray, lines, 1, CV_PI/180, 110, 0, 0 );
cv::cvtColor(src_img_gray, src_img_gray, CV_GRAY2BGR);
for( size_t i = 0; i < lines.size(); i++ )
{
  float rho = lines[i][0], theta = lines[i][1];
  Point pt1, pt2;
  double a = cos(theta), b = sin(theta);
  double x0 = a*rho, y0 = b*rho;
  pt1.x = cvRound(x0 + 1000*(-b));
  pt1.y = cvRound(y0 + 1000*(a));
  pt2.x = cvRound(x0 - 1000*(-b));
  pt2.y = cvRound(y0 - 1000*(a));
  line( src_img_gray, pt1, pt2, Scalar(0,255,0), 3, CV_AA);
}

// ITERATE THROUGH IMAGE TO MANIPULATE BGR-PIXEL VALUES.
for(int y=0;y<src_img.rows;y++)
{
  for(int x=0;x<src_img.cols;x++)
  {
    Vec3b color = src_img.at<Vec3b>(Point(x,y));
    color[0] = color[0]*2.0;
    color[1] = color[1]*2.0;
    color[2] = color[2]*2.0;
    src_img.at<Vec3b>(Point(x,y)) = color;
  }
}

/// GIVE IPM OBJECT NEW IMAGE.
test_object.IPM::getImage(src_img);

// SET PARAMETERS OF IPM OBJECT.
test_object.IPM::setParam(camera_height, pitch_angle, focal_length_px, src_img.cols, src_img.rows);

// APPLY IPM ON IPM OBJECT AND GET IPM'D IMAGE.
after_ipm = test_object.IPM::invPerspectiveMapping();

// CREATE ROSNODE AND SUBSCRIBE TO WEBCAM.
ros::init(argc, argv, "node_name");
ros::NodeHandle n;
ros::Subscriber sub = n.subscribe("/usb_cam/image_raw", 10, ipmCallback);
ros::spin();

// SOLVE A LINEAR SYSTEM OF EQUATIONS.
Eigen::Matrix3f A;
Eigen::Vector3f b;
A << 1,2,3,  4,5,6,  7,8,10;
b << 3, 3, 4;
cout << "Here is the matrix A:\n" << A << endl;
cout << "Here is the vector b:\n" << b << endl;
Eigen::Vector3f x = A.colPivHouseholderQr().solve(b);
cout << "The solution is:\n" << x << endl;
