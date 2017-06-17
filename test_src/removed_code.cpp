  /* Filtering out two lines.
  // Based on update_counter_left and update_counter_right, decide if: Extrapolate left from right, extrapolate right from left or take previous parameters.
  if((update_counter_left > 0) && (update_counter_right == 0))
  {
    // Extrapolate left line, using right line and lateral distance from last frame.
    // 1. Get two points in the imag which define the RIGHT Line.
    // Image coordinates as defined in manuscript.
    Point right_far_img;
    Point right_near_img;

    float y_top_roi = 0.0;
    float y_bottom_roi = src_roi.rows;

    right_far_img.x = rho_right*cos(theta_right_rad) - sin(theta_right_rad)*((y_top_roi - rho_right*sin(theta_right_rad))/(cos(theta_right_rad)));
    right_near_img.x = rho_right*cos(theta_right_rad) - sin(theta_right_rad)*((y_bottom_roi - rho_right*sin(theta_right_rad))/(cos(theta_right_rad)));
    right_far_img.y = 250.0;
    right_near_img.y = 420.0;

    // 2. Using IPM-matrices, transform the two image points to world points in the local vehicle frame.
    // World coordinates as defined in manuscript.
    Point right_far_world;
    Point right_near_world;
    // 2a. Calculate the lambda from equation (7) to see, where the ray crosses the ground/street-plane.
    float lambda_right_far = camera_height/(focal_length*cos(camera_angle) - (src.rows/2.0 - right_far_img.y)*sin(camera_angle));
    float lambda_right_near = camera_height/(focal_length*cos(camera_angle) - (src.rows/2.0 - right_near_img.y)*sin(camera_angle));
    // 2b. Knowing lambda, calculate the projections of the two points on the groundplane.
    right_far_world.x = lambda_right_far*(cos(camera_angle)*(src.rows/2.0 - right_far_img.y) + focal_length*sin(camera_angle));
    right_far_world.y = lambda_right_far*(-1.0)*(right_far_img.x - src.cols/2.0);
    right_near_world.x = lambda_right_near*(cos(camera_angle)*(src.rows/2.0 - right_near_img.y) + focal_length*sin(camera_angle));;
    right_near_world.y = lambda_right_near*(-1.0)*(right_near_img.x - src.cols/2.0);

    // 3. Find the directional vector of those two world points, find the left-facing normal vector (in world coordinates).
    Point2f dir_right;
    Point2f norm_right;

    dir_right.x = right_far_world.x - right_near_world.x;
    dir_right.y = right_far_world.y - right_near_world.y;

    norm_right.x = -dir_right.y/sqrt(pow(dir_right.x, 2) + pow(dir_right.y, 2));
    norm_right.y = dir_right.x/sqrt(pow(dir_right.x, 2) + pow(dir_right.y, 2));
    // 4. Using the normal vector, the parallel_distance and one point of the right line, find a point on the left line.
    // World coordinates as defined in manuscript.
    Point left_near_world;

    left_near_world.x = right_near_world.x + parallel_distance*norm_right.x;
    left_near_world.y = right_near_world.y + parallel_distance*norm_right.y;
    // 5. Find another point on the left line.
    Point left_far_world;

    left_far_world.x = right_far_world.x + parallel_distance*norm_right.x;
    left_far_world.y = right_far_world.y + parallel_distance*norm_right.y;

    std::cout<<"Left Far: "<<left_far_world<<std::endl;
    std::cout<<"Right Far: "<<right_far_world<<std::endl;
    std::cout<<"Left Near: "<<left_near_world<<std::endl;
    std::cout<<"Right Near: "<<right_near_world<<std::endl;
    // 6. Transform the two left line's point to the image, using PM.
    // Transform to camera coordinate system.
    Point3f left_far_camera;
    Point3f left_near_camera;

    left_far_camera.x = 0*(left_far_world.x) -1.0*(left_far_world.y) + 0*(-camera_height);
    left_far_camera.y = cos(camera_angle)*(left_far_world.x) + 0*(left_far_world.y) + sin(camera_angle)*(-camera_height);
    left_far_camera.z = -sin(camera_angle)*(left_far_world.x) + 0*(left_far_world.y) + cos(camera_angle)*(-camera_height);

    left_near_camera.x = 0*(left_near_world.x) -1.0*(left_near_world.y) + 0*(-camera_height);
    left_near_camera.y = cos(camera_angle)*(left_near_world.x) + 0*(left_near_world.y) + sin(camera_angle)*(-camera_height);
    left_near_camera.z = -sin(camera_angle)*(left_near_world.x) + 0*(left_near_world.y) + cos(camera_angle)*(-camera_height);
    // Transform to pixels. In image coordinates.
    float lambda_px_left_far = -focal_length/(left_far_camera.z);
    float lambda_px_left_near = -focal_length/(left_near_camera.z);

    cv::Point2f left_far;
    cv::Point2f left_near;

    left_far.x = lambda_px_left_far*left_far_camera.x + src.cols/2.0;
    left_far.y = src.cols/2.0 - lambda_px_left_far*left_far_camera.y;
    left_near.x = lambda_px_left_near*left_near_camera.x + src.cols/2.0;
    left_near.y = src.cols/2.0 - lambda_px_left_near*left_near_camera.y;
    // 7. Convert to rho, theta for the cropped image.
    // Get rho and theta.
    Eigen::Vector2f params_left = getRhoAndTheta(left_near.x, left_far.x, left_near.y, left_far.y);

    theta_left_rad = params_left[0];
    rho_left = params_left[1];

    std::cout<<"Extrapolated left from right."<<std::endl;
    // Only for now.
    update_counter_left = 0;
  }
  else if((update_counter_left == 0) && (update_counter_right > 0))
  {
    // Extrapolate right line, using left line and lateral distance from last frame.
    // 1. Get two points in the image which define the LEFT Line.
    // 2. Using IPM-matrices, transform the two image points to world points in the local vehicle frame.
    // 3. Find the directional vector of those two world points, find the right-facing normal vector.
    // 4. Using the normal vector, the parallel_distance and one point of the left line, find a point on the right line.
    // 5. Find another point on the right line, using the left line's directional vector.
    // 6. Transform the two right line's point to the image, using PM.
    // 7. Convert to rho_right, theta_right_rad and save them to global variables.

    // FOR NOW: JUST LEAVE IT TO THE OLD VALUES OR THE ONES WHICH WERE ACTUALLY FOUND.
  }
  else if((update_counter_left > 0) && (update_counter_right > 0))
  {
    // Just take the previous parameters. --> Do nothing, do not change the parameters.
  }

  // Final step: Calculate parallel_distance,...
  parallel_distance = 3;
  */



     /* Necessary???
    // Prompt user to select two lines on the cropped input image.
    setCtrlPts(src_roi);
    Transform and save x, y to rho, theta.
    Eigen::Vector2f polar_parameters;
    // Left line.
    polar_parameters = getRhoAndTheta(init_points[0].x, init_points[1].x, init_points[0].y, init_points[1].y);
    theta_left_rad = polar_parameters[0];
    rho_left = polar_parameters[1];
    // Right line;
    polar_parameters = getRhoAndTheta(init_points[2].x, init_points[3].x, init_points[2].y, init_points[3].y);
    theta_right_rad = polar_parameters[0];
    rho_right = polar_parameters[1];
    */