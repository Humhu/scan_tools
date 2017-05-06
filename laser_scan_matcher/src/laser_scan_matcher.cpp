/*
 * Copyright (c) 2011, Ivan Dryanovski, William Morris
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the CCNY Robotics Lab nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*  This package uses Canonical Scan Matcher [1], written by
 *  Andrea Censi
 *
 *  [1] A. Censi, "An ICP variant using a point-to-line metric"
 *  Proceedings of the IEEE International Conference
 *  on Robotics and Automation (ICRA), 2008
 */

#include <laser_scan_matcher/laser_scan_matcher.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/assign.hpp>

#include <argus_utils/geometry/PoseSE2.h>
#include <argus_utils/geometry/GeometryUtils.h>

namespace scan_tools
{

LaserScanMatcher::LaserScanMatcher(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh),
  nh_private_(nh_private),
  initialized_(false),
  received_imu_(false),
  received_odom_(false),
  received_vel_(false),
  extrinsics_(nh)
{
  ROS_INFO("Starting LaserScanMatcher");

  // **** init parameters

  initParams();

  // **** state variables

  input_.laser[0] = 0.0;
  input_.laser[1] = 0.0;
  input_.laser[2] = 0.0;

  // InitializeAndRead output_ vectors as Null for error-checking
  output_.cov_x_m = 0;
  output_.dx_dy1_m = 0;
  output_.dx_dy2_m = 0;

  // **** publishers

  if (publish_pose_)
  {
    pose_publisher_  = nh_.advertise<geometry_msgs::Pose2D>(
      "pose2D", 5);
  }

  if (publish_pose_stamped_)
  {
    pose_stamped_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(
      "pose_stamped", 5);
  }

  if (publish_pose_with_covariance_)
  {
    pose_with_covariance_publisher_  = nh_.advertise<geometry_msgs::PoseWithCovariance>(
      "pose_with_covariance", 5);
  }

  if (publish_pose_with_covariance_stamped_)
  {
    pose_with_covariance_stamped_publisher_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
      "pose_with_covariance_stamped", 5);
  }

  // *** subscribers

  if (use_cloud_input_)
  {
    cloud_subscriber_ = nh_.subscribe(
      "cloud", 1, &LaserScanMatcher::cloudCallback, this);
  }
  else
  {
    scan_subscriber_ = nh_.subscribe(
      "scan", 1, &LaserScanMatcher::scanCallback, this);
  }

  if (use_imu_)
  {
    imu_subscriber_ = nh_.subscribe(
      "imu/data", 1, &LaserScanMatcher::imuCallback, this);
  }
  if (use_odom_)
  {
    odom_subscriber_ = nh_.subscribe(
      "odom", 1, &LaserScanMatcher::odomCallback, this);
  }
  if (use_vel_)
  {
    if (stamped_vel_)
      vel_subscriber_ = nh_.subscribe(
        "vel", 1, &LaserScanMatcher::velStmpCallback, this);
    else
      vel_subscriber_ = nh_.subscribe(
        "vel", 1, &LaserScanMatcher::velCallback, this);
  }
}

LaserScanMatcher::~LaserScanMatcher()
{
  ROS_INFO("Destroying LaserScanMatcher");
}

void LaserScanMatcher::initParams()
{
  if (!nh_private_.getParam ("base_frame", base_frame_))
    base_frame_ = "base_link";
  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = "world";

  // **** input type - laser scan, or point clouds?
  // if false, will subscribe to LaserScan msgs on /scan.
  // if true, will subscribe to PointCloud2 msgs on /cloud

  if (!nh_private_.getParam ("use_cloud_input", use_cloud_input_))
    use_cloud_input_= false;

  if (use_cloud_input_)
  {
    cloud_range_min_.InitializeAndRead(nh_private_,
                                0.1,
                                "cloud_range_min",
                                "Minimum point distance");
    cloud_range_max_.InitializeAndRead(nh_private_,
                                50.0,
                                "cloud_range_max",
                                "Maximum point distance");
    cloud_res_.InitializeAndRead(nh_private_,
                          0.05,
                          "cloud_res",
                          "Cloud resolution");
  }

  // **** keyframe params: when to generate the keyframe scan
  // if either is set to 0, reduces to frame-to-frame matching

  kf_dist_linear_.InitializeAndRead(nh_private_,
                             0.1,
                             "kf_dist_linear",
                             "Keyframe linear threshold");
  kf_dist_angular_.InitializeAndRead(nh_private_,
                              10.0 * (M_PI / 180.0),
                              "kf_dist_angular",
                              "Keyframe angular threshold");

  // **** What predictions are available to speed up the ICP?
  // 1) imu - [theta] from imu yaw angle - /imu topic
  // 2) odom - [x, y, theta] from wheel odometry - /odom topic
  // 3) vel - [x, y, theta] from velocity predictor - see alpha-beta predictors - /vel topic
  // If more than one is enabled, priority is imu > odom > vel

  if (!nh_private_.getParam ("use_imu", use_imu_))
    use_imu_ = true;
  if (!nh_private_.getParam ("use_odom", use_odom_))
    use_odom_ = true;
  if (!nh_private_.getParam ("use_vel", use_vel_))
    use_vel_ = false;

  // **** Are velocity input messages stamped?
  // if false, will subscribe to Twist msgs on /vel
  // if true, will subscribe to TwistStamped msgs on /vel
  if (!nh_private_.getParam ("stamped_vel", stamped_vel_))
    stamped_vel_ = false;

  // **** How to publish the output?
  // tf (fixed_frame->base_frame),
  // pose message (pose of base frame in the fixed frame)

  if (!nh_private_.getParam ("publish_tf", publish_tf_))
    publish_tf_ = true;
  if (!nh_private_.getParam ("publish_pose", publish_pose_))
    publish_pose_ = true;
  if (!nh_private_.getParam ("publish_pose_stamped", publish_pose_stamped_))
    publish_pose_stamped_ = false;
  if (!nh_private_.getParam ("publish_pose_with_covariance", publish_pose_with_covariance_))
    publish_pose_with_covariance_ = false;
  if (!nh_private_.getParam ("publish_pose_with_covariance_stamped", publish_pose_with_covariance_stamped_))
    publish_pose_with_covariance_stamped_ = false;

  if (!nh_private_.getParam("position_covariance", position_covariance_))
  {
    position_covariance_.resize(3);
    std::fill(position_covariance_.begin(), position_covariance_.end(), 1e-9);
  }

  if (!nh_private_.getParam("orientation_covariance", orientation_covariance_))
  {
    orientation_covariance_.resize(3);
    std::fill(orientation_covariance_.begin(), orientation_covariance_.end(), 1e-9);
  }

  // **** CSM parameters - comments copied from algos.h (by Andrea Censi)
  // Maximum angular displacement between scans
  max_angular_correction_.InitializeAndRead(nh_private_,
                               45.0,
                               "max_angular_correction",
                               "Maximum angular displacement between scans (deg)");
  max_angular_correction_.AddCheck<argus::GreaterThan>(0.0);

  // Maximum translation between scans (m)
  max_linear_correction_.InitializeAndRead(nh_private_,
                              0.50,
                              "max_linear_correction",
                              "Maximum translation between scans (m)");
  max_linear_correction_.AddCheck<argus::GreaterThan>(0.0);

  // Maximum ICP cycle iterations
  max_iterations_.InitializeAndRead(nh_private_,
                             10,
                             "max_iterations",
                             "Maximum ICP cycle iterations");
  max_iterations_.AddCheck<argus::IntegerValued>();

  // A threshold for stopping (m)
  log_epsilon_xy_.InitializeAndRead(nh_private_,
                             -6,
                             "log_epsilon_xy",
                             "Log threshold for stopping (m)");

  // A threshold for stopping (rad)
  log_epsilon_theta_.InitializeAndRead(nh_private_,
                                -6,
                                "log_epsilon_theta",
                                "Log threshold for stopping (rad)");

  // Maximum distance for a correspondence to be valid
  max_correspond_dist_.InitializeAndRead(nh_private_,
                                  0.3,
                                  "max_correspond_dist",
                                  "Maximum distance for a correspondence to be valid");
  max_correspond_dist_.AddCheck<argus::GreaterThan>(0.0);

  // Noise in the scan (m)
  log_sigma_.InitializeAndRead(nh_private_,
                    -2,
                    "log_sigma",
                    "Log noise in the scan (m)");

  // Use smart tricks for finding correspondences.
  use_corr_tricks_.InitializeAndRead(nh_private_,
                              true,
                              "use_correspond_tricks",
                              "Use smart tricks for finding correspondences");

  // Restart: Restart if error is over threshold
  restart_.InitializeAndRead(nh_private_,
                      false,
                      "restart_if_err_over",
                      "Restart if error is over threshold");

  // Restart: Threshold for restarting
  log_restart_thresh_err_.InitializeAndRead(nh_private_,
                                     -2,
                                     "log_restart_thresh_err",
                                     "Log mean error threshold for restarting");

  // Restart: displacement for restarting. (m)
  restart_dt_.InitializeAndRead(nh_private_,
                         1.0,
                         "restart_dt",
                         "Displacement for restarting (m)");
  restart_dt_.AddCheck<argus::GreaterThan>(0);

  // Restart: displacement for restarting. (rad)
  restart_dtheta_.InitializeAndRead(nh_private_,
                             0.1,
                             "restart_dtheta",
                             "Displacement for restarting (rad)");
  restart_dtheta_.AddCheck<argus::GreaterThan>(0);

  // Max distance for staying in the same clustering
  clustering_thresh_.InitializeAndRead(nh_private_,
                                   0.25,
                                   "clustering_threshold",
                                   "Max distance for staying in the same clustering");
  clustering_thresh_.AddCheck<argus::GreaterThan>(0);

  // Number of neighbour rays used to estimate the orientation
  orientation_neighbourhood_.InitializeAndRead(nh_private_,
                                        20,
                                        "orientation_neighbourhood",
                                        "Number of neighbour rays used to estimate the orientation");
  orientation_neighbourhood_.AddCheck<argus::IntegerValued>();

  // If 0, it's vanilla ICP
  use_point_to_line_dist_.InitializeAndRead(nh_private_,
                                     true,
                                     "use_point_to_line_distance",
                                     "Whether to use line-based ICP or vanilla ICP");

  // Discard correspondences based on the angles
  do_alpha_test_.InitializeAndRead(nh_private_,
                            false,
                            "do_alpha_test",
                            "Discard correspondences based on the angles");

  // Discard correspondences based on the angles - threshold angle, in degrees
  alpha_test_thresh_.InitializeAndRead(nh_private_,
                                20.0,
                                "alpha_test_thresh",
                                "Correspondence angle threshold (deg)");
  alpha_test_thresh_.AddCheck<argus::GreaterThan>(0);

  // Percentage of correspondences to consider: if 0.9,
  // always discard the top 10% of correspondences with more error
  outliers_maxPerc_.InitializeAndRead(nh_private_,
                              0.90,
                              "outliers_max_ratio",
                              "Ratio of correspondences to consider (outlier ratio inverse)");
  outliers_maxPerc_.AddCheck<argus::GreaterThanOrEqual>(0);
  outliers_maxPerc_.AddCheck<argus::LessThanOrEqual>(1);  

  // Parameters describing a simple adaptive algorithm for discarding.
  //  1) Order the errors.
  //  2) Choose the percentile according to outliers_adaptive_order.
  //     (if it is 0.7, get the 70% percentile)
  //  3) Define an adaptive threshold multiplying outliers_adaptive_mult
  //     with the value of the error at the chosen percentile.
  //  4) Discard correspondences over the threshold.
  //  This is useful to be conservative; yet remove the biggest errors.
  outliers_adaptive_order_.InitializeAndRead(nh_private_,
                                      0.7,
                                      "outliers_adaptive_order",
                                      "Ratio (percentile) for base adaptive error threshold");
  outliers_adaptive_order_.AddCheck<argus::GreaterThanOrEqual>(0);
  outliers_adaptive_order_.AddCheck<argus::LessThanOrEqual>(1);

  outliers_adaptive_mult_.InitializeAndRead(nh_private_,
                                     2.0,
                                     "outliers_adaptive_mult",
                                     "Multiplier on base adaptive error threshold");
  outliers_adaptive_mult_.AddCheck<argus::GreaterThan>(0);

  // If you already have a guess of the solution, you can compute the polar angle
  // of the points of one scan in the new position. If the polar angle is not a monotone
  // function of the readings index, it means that the surface is not visible in the
  // next position. If it is not visible, then we don't use it for matching.
  do_visibility_test_.InitializeAndRead(nh_private_,
                                 false,
                                 "do_visibility_test",
                                 "Whether to enable the angle-based surface visibility test");

  // no two points in laser_sens can have the same corr.
  do_remove_doubles_.InitializeAndRead(nh_private_,
                                true,
                                "do_remove_doubles",
                                "Whether to remove points with the same correspondence");

  // If 1, computes the covariance of ICP using the method http://purl.org/censi/2006/icpcov
  argus::GetParam(nh_private_, "do_compute_covariance", do_compute_covariance_, false);

  // Checks that find_correspondences_tricks gives the right answer
  argus::GetParam(nh_private_, "debug_verify_tricks", debug_verify_tricks_, false);

  // If 1, the field 'true_alpha' (or 'alpha') in the first scan is used to compute the
  // incidence beta, and the factor (1/cos^2(beta)) used to weight the correspondence.");
  use_ml_weights_.InitializeAndRead(nh_private_,
                             false,
                             "use_ml_weights",
                             "Whether to use the first scan incidence beta to weight the correspondences");

  // If 1, the field 'readings_sigma' in the second scan is used to weight the
  // correspondence by 1/sigma^2
  use_sigma_weights_.InitializeAndRead(nh_private_,
                                false,
                                "use_sigma_weights",
                                "Whether to use the second scan sigma to weight the correspondence");
}

void LaserScanMatcher::updateScanMatchParams(sm_params& params)
{
  params.max_angular_correction_deg = max_angular_correction_;
  params.max_linear_correction = max_linear_correction_;
  params.epsilon_xy = std::pow(10, log_epsilon_xy_);
  params.epsilon_theta = std::pow(10, log_epsilon_theta_);
  params.max_iterations = max_iterations_;
  params.max_correspondence_dist = max_correspond_dist_;
  params.use_corr_tricks = use_corr_tricks_;
  params.restart = restart_;
  params.restart_threshold_mean_error = std::pow(10, log_restart_thresh_err_);
  params.restart_dt = restart_dt_;
  params.restart_dtheta = restart_dtheta_;
  params.outliers_maxPerc = outliers_maxPerc_;
  params.outliers_adaptive_order = outliers_adaptive_order_;
  params.outliers_adaptive_mult = outliers_adaptive_mult_;
  params.outliers_remove_doubles = do_remove_doubles_;
  params.clustering_threshold = clustering_thresh_;
  params.orientation_neighbourhood = orientation_neighbourhood_;
  params.do_alpha_test = do_alpha_test_;
  params.do_alpha_test_thresholdDeg = alpha_test_thresh_;
  params.do_visibility_test = do_visibility_test_;
  params.use_point_to_line_distance = use_point_to_line_dist_;
  params.use_ml_weights = use_ml_weights_;
  params.use_sigma_weights = use_sigma_weights_;
  params.do_compute_covariance = do_compute_covariance_;
  params.debug_verify_tricks = debug_verify_tricks_;
  params.sigma = std::pow(10, log_sigma_);
  if(use_cloud_input_)
  {
    params.min_reading = cloud_range_min_;
    params.max_reading = cloud_range_max_;
  }
}

void LaserScanMatcher::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
  boost::mutex::scoped_lock(mutex_);
  latest_imu_msg_ = *imu_msg;
  if (!received_imu_)
  {
    last_used_imu_msg_ = *imu_msg;
    received_imu_ = true;
  }
}

void LaserScanMatcher::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  boost::mutex::scoped_lock(mutex_);
  latest_odom_msg_ = *odom_msg;
  if (!received_odom_)
  {
    last_used_odom_msg_ = *odom_msg;
    received_odom_ = true;
  }
}

void LaserScanMatcher::velCallback(const geometry_msgs::Twist::ConstPtr& twist_msg)
{
  boost::mutex::scoped_lock(mutex_);
  latest_vel_msg_ = *twist_msg;

  received_vel_ = true;
}

void LaserScanMatcher::velStmpCallback(const geometry_msgs::TwistStamped::ConstPtr& twist_msg)
{
  boost::mutex::scoped_lock(mutex_);
  latest_vel_msg_ = twist_msg->twist;

  received_vel_ = true;
}

void LaserScanMatcher::cloudCallback (const PointCloudT::ConstPtr& cloud)
{
  // **** if first scan, cache the tf from base to the scanner

  std_msgs::Header cloud_header = pcl_conversions::fromPCL(cloud->header);

  if (!initialized_)
  {
    // cache the static tf from base to laser
    if (!getBaseToLaserTf(cloud_header.frame_id))
    {
      ROS_WARN("Skipping scan");
      return;
    }

    PointCloudToLDP(cloud, prev_ldp_scan_);
    last_icp_time_ = cloud_header.stamp;
    initialized_ = true;
  }

  LDP curr_ldp_scan;
  PointCloudToLDP(cloud, curr_ldp_scan);
  processScan(curr_ldp_scan, cloud_header.stamp);
}

void LaserScanMatcher::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  // **** if first scan, cache the tf from base to the scanner

  if (!initialized_)
  {
    createCache(scan_msg);    // caches the sin and cos of all angles

    // cache the static tf from base to laser
    if (!getBaseToLaserTf(scan_msg->header.frame_id))
    {
      ROS_WARN("Skipping scan");
      return;
    }

    laserScanToLDP(scan_msg, prev_ldp_scan_);
    last_icp_time_ = scan_msg->header.stamp;
    initialized_ = true;
  }

  LDP curr_ldp_scan;
  laserScanToLDP(scan_msg, curr_ldp_scan);
  processScan(curr_ldp_scan, scan_msg->header.stamp);
}

void LaserScanMatcher::processScan(LDP& curr_ldp_scan, const ros::Time& time)
{
  ros::WallTime start = ros::WallTime::now();

  // CSM is used in the following way:
  // The scans are always in the laser frame
  // The reference scan (prevLDPcan_) has a pose of [0, 0, 0]
  // The new scan (currLDPScan) has a pose equal to the movement
  // of the laser in the laser frame since the last scan
  // The computed correction is then propagated using the tf machinery

  prev_ldp_scan_->odometry[0] = 0.0;
  prev_ldp_scan_->odometry[1] = 0.0;
  prev_ldp_scan_->odometry[2] = 0.0;

  prev_ldp_scan_->estimate[0] = 0.0;
  prev_ldp_scan_->estimate[1] = 0.0;
  prev_ldp_scan_->estimate[2] = 0.0;

  prev_ldp_scan_->true_pose[0] = 0.0;
  prev_ldp_scan_->true_pose[1] = 0.0;
  prev_ldp_scan_->true_pose[2] = 0.0;

  input_.laser_ref  = prev_ldp_scan_;
  input_.laser_sens = curr_ldp_scan;

  // **** estimated change since last scan

  double dt = (time - last_icp_time_).toSec();
  double pr_ch_x, pr_ch_y, pr_ch_a;
  getPrediction(pr_ch_x, pr_ch_y, pr_ch_a, dt);

  // the predicted change of the laser's position, in the fixed frame

  argus::PoseSE2 pr_ch2(pr_ch_x, pr_ch_y, pr_ch_a);
  argus::PoseSE3 pr_ch = argus::PoseSE3::FromSE2(pr_ch2);
//   tf::Transform pr_ch;
//   createTfFromXYTheta(pr_ch_x, pr_ch_y, pr_ch_a, pr_ch);

  // account for the change since the last kf, in the fixed frame

  pr_ch = pr_ch * (f2b_ * f2b_kf_.Inverse());

  // the predicted change of the laser's position, in the laser frame

  //tf::Transform pr_ch_l;
  argus::PoseSE3 pr_ch_l;
  pr_ch_l = laser_to_base_ * f2b_.Inverse() * pr_ch * f2b_ * base_to_laser_ ;

  argus::PoseSE2 pr_ch_l2 = argus::PoseSE2::FromSE3(pr_ch_l);
  input_.first_guess[0] = pr_ch_l2.GetTranslation().x(); //getOrigin().getX();
  input_.first_guess[1] = pr_ch_l2.GetTranslation().y();//getOrigin().getY();
  input_.first_guess[2] = pr_ch_l2.GetRotation().angle(); //tf::getYaw(pr_ch_l.getRotation());

  // If they are non-Null, free covariance gsl matrices to avoid leaking memory
  if (output_.cov_x_m)
  {
    gsl_matrix_free(output_.cov_x_m);
    output_.cov_x_m = 0;
  }
  if (output_.dx_dy1_m)
  {
    gsl_matrix_free(output_.dx_dy1_m);
    output_.dx_dy1_m = 0;
  }
  if (output_.dx_dy2_m)
  {
    gsl_matrix_free(output_.dx_dy2_m);
    output_.dx_dy2_m = 0;
  }

  // *** scan match - using point to line icp from CSM
  updateScanMatchParams(input_);
  sm_icp(&input_, &output_);
  
//tf::Transform corr_ch;
  argus::PoseSE3 corr_ch;

  if (output_.valid)
  {

    // the correction of the laser's position, in the laser frame
    //tf::Transform corr_ch_l;
    // createTfFromXYTheta(output_.x[0], output_.x[1], output_.x[2], corr_ch_l);
	argus::PoseSE2 corr_ch_l2(output_.x[0], output_.x[1], output_.x[2]);
	argus::PoseSE3 corr_ch_l = argus::PoseSE3::FromSE2(corr_ch_l2);

    // the correction of the base's position, in the base frame
    corr_ch = base_to_laser_ * corr_ch_l * laser_to_base_;

    // update the pose in the world frame
    f2b_ = f2b_kf_ * corr_ch;

    // **** publish

	argus::PoseSE2 f2b_2 = argus::PoseSE2::FromSE3(f2b_);

    if (publish_pose_)
    {
      // unstamped Pose2D message
      geometry_msgs::Pose2D::Ptr pose_msg;
      pose_msg = boost::make_shared<geometry_msgs::Pose2D>();
      pose_msg->x = f2b_2.GetTranslation().x(); //f2b_.getOrigin().getX();
      pose_msg->y = f2b_2.GetTranslation().y(); //f2b_.getOrigin().getY();
      pose_msg->theta = f2b_2.GetRotation().angle(); //tf::getYaw(f2b_.getRotation());
      pose_publisher_.publish(pose_msg);
    }
    if (publish_pose_stamped_)
    {
      // stamped Pose message
      geometry_msgs::PoseStamped::Ptr pose_stamped_msg;
      pose_stamped_msg = boost::make_shared<geometry_msgs::PoseStamped>();

      pose_stamped_msg->header.stamp    = time;
      pose_stamped_msg->header.frame_id = fixed_frame_;
	  pose_stamped_msg->pose = PoseToMsg(f2b_);
    //   tf::poseTFToMsg(f2b_, pose_stamped_msg->pose);

      pose_stamped_publisher_.publish(pose_stamped_msg);
    }
    if (publish_pose_with_covariance_)
    {
      // unstamped PoseWithCovariance message
      geometry_msgs::PoseWithCovariance::Ptr pose_with_covariance_msg;
      pose_with_covariance_msg = boost::make_shared<geometry_msgs::PoseWithCovariance>();
	  pose_with_covariance_msg->pose = PoseToMsg(f2b_);
	//   tf::poseTFToMsg(f2b_, pose_with_covariance_msg->pose);

      if (input_.do_compute_covariance)
      {
        pose_with_covariance_msg->covariance = boost::assign::list_of
          (gsl_matrix_get(output_.cov_x_m, 0, 0)) (0)  (0)  (0)  (0)  (0)
          (0)  (gsl_matrix_get(output_.cov_x_m, 0, 1)) (0)  (0)  (0)  (0)
          (0)  (0)  (static_cast<double>(position_covariance_[2])) (0)  (0)  (0)
          (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[0])) (0)  (0)
          (0)  (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[1])) (0)
          (0)  (0)  (0)  (0)  (0)  (gsl_matrix_get(output_.cov_x_m, 0, 2));
      }
      else
      {
        pose_with_covariance_msg->covariance = boost::assign::list_of
          (static_cast<double>(position_covariance_[0])) (0)  (0)  (0)  (0)  (0)
          (0)  (static_cast<double>(position_covariance_[1])) (0)  (0)  (0)  (0)
          (0)  (0)  (static_cast<double>(position_covariance_[2])) (0)  (0)  (0)
          (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[0])) (0)  (0)
          (0)  (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[1])) (0)
          (0)  (0)  (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[2]));
      }

      pose_with_covariance_publisher_.publish(pose_with_covariance_msg);
    }
    if (publish_pose_with_covariance_stamped_)
    {
      // stamped Pose message
      geometry_msgs::PoseWithCovarianceStamped::Ptr pose_with_covariance_stamped_msg;
      pose_with_covariance_stamped_msg = boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>();

      pose_with_covariance_stamped_msg->header.stamp    = time;
      pose_with_covariance_stamped_msg->header.frame_id = fixed_frame_;
	  pose_with_covariance_stamped_msg->pose.pose = argus::PoseToMsg(f2b_);
    //   tf::poseTFToMsg(f2b_, pose_with_covariance_stamped_msg->pose.pose);

      if (input_.do_compute_covariance)
      {
        pose_with_covariance_stamped_msg->pose.covariance = boost::assign::list_of
          (gsl_matrix_get(output_.cov_x_m, 0, 0)) (0)  (0)  (0)  (0)  (0)
          (0)  (gsl_matrix_get(output_.cov_x_m, 0, 1)) (0)  (0)  (0)  (0)
          (0)  (0)  (static_cast<double>(position_covariance_[2])) (0)  (0)  (0)
          (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[0])) (0)  (0)
          (0)  (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[1])) (0)
          (0)  (0)  (0)  (0)  (0)  (gsl_matrix_get(output_.cov_x_m, 0, 2));
      }
      else
      {
        pose_with_covariance_stamped_msg->pose.covariance = boost::assign::list_of
          (static_cast<double>(position_covariance_[0])) (0)  (0)  (0)  (0)  (0)
          (0)  (static_cast<double>(position_covariance_[1])) (0)  (0)  (0)  (0)
          (0)  (0)  (static_cast<double>(position_covariance_[2])) (0)  (0)  (0)
          (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[0])) (0)  (0)
          (0)  (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[1])) (0)
          (0)  (0)  (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[2]));
      }

      pose_with_covariance_stamped_publisher_.publish(pose_with_covariance_stamped_msg);
    }

    if (publish_tf_)
    {
		extrinsics_.SetExtrinsics(base_frame_, fixed_frame_, time, f2b_);
	//   geometry_msgs::TransformStamped transform_msg;
	//   msg.header.stamp = time;
	//   msg.header.frame_id = fixed_frame_;
	//   msg.child_frame_id = base_frame_;

    //   tf::StampedTransform transform_msg (f2b_, time, fixed_frame_, base_frame_);
    //   tf_broadcaster_.sendTransform(transform_msg);
    }
  }
  else
  {
	  corr_ch = argus::PoseSE3();
    // corr_ch.setIdentity();
    ROS_WARN("Error in scan matching");
  }

  // **** swap old and new
  // NOTE Added this check for invalid output to make overall tracking
  // more robust at the cost of more drift
  if (!output_.valid || newKeyframeNeeded(corr_ch))
  {
    // generate a keyframe
    ld_free(prev_ldp_scan_);
    prev_ldp_scan_ = curr_ldp_scan;
    f2b_kf_ = f2b_;
  }
  else
  {
    ld_free(curr_ldp_scan);
  }

  last_icp_time_ = time;

  // **** statistics

  double dur = (ros::WallTime::now() - start).toSec() * 1e3;
  ROS_DEBUG("Scan matcher total duration: %.1f ms", dur);
}

bool LaserScanMatcher::newKeyframeNeeded(const argus::PoseSE3& d)
{
  argus::PoseSE2 d2 = argus::PoseSE2::FromSE3(d);

  if (fabs(d2.GetRotation().angle()) > kf_dist_angular_) return true;

  double x = d2.GetTranslation().x(); //d.getOrigin().getX();
  double y = d2.GetTranslation().x(); //d.getOrigin().getY();
  if ((x*x + y*y) > (kf_dist_linear_ * kf_dist_linear_)) return true;

  return false;
}

void LaserScanMatcher::PointCloudToLDP(const PointCloudT::ConstPtr& cloud,
                                             LDP& ldp)
{
  double max_d2 = cloud_res_ * cloud_res_;

  PointCloudT cloud_f;

  cloud_f.points.push_back(cloud->points[0]);

  for (unsigned int i = 1; i < cloud->points.size(); ++i)
  {
    const PointT& pa = cloud_f.points[cloud_f.points.size() - 1];
    const PointT& pb = cloud->points[i];

    double dx = pa.x - pb.x;
    double dy = pa.y - pb.y;
    double d2 = dx*dx + dy*dy;

    if (d2 > max_d2)
    {
      cloud_f.points.push_back(pb);
    }
  }

  unsigned int n = cloud_f.points.size();

  ldp = ld_alloc_new(n);

  for (unsigned int i = 0; i < n; i++)
  {
    // calculate position in laser frame
    if (is_nan(cloud_f.points[i].x) || is_nan(cloud_f.points[i].y))
    {
      ROS_WARN("Laser Scan Matcher: Cloud input contains NaN values. \
                Please use a filtered cloud input.");
    }
    else
    {
      double r = sqrt(cloud_f.points[i].x * cloud_f.points[i].x +
                      cloud_f.points[i].y * cloud_f.points[i].y);

      if (r > cloud_range_min_ && r < cloud_range_max_)
      {
        ldp->valid[i] = 1;
        ldp->readings[i] = r;
      }
      else
      {
        ldp->valid[i] = 0;
        ldp->readings[i] = -1;  // for invalid range
      }
    }

    ldp->theta[i] = atan2(cloud_f.points[i].y, cloud_f.points[i].x);
    ldp->cluster[i]  = -1;
  }

  ldp->min_theta = ldp->theta[0];
  ldp->max_theta = ldp->theta[n-1];

  ldp->odometry[0] = 0.0;
  ldp->odometry[1] = 0.0;
  ldp->odometry[2] = 0.0;

  ldp->true_pose[0] = 0.0;
  ldp->true_pose[1] = 0.0;
  ldp->true_pose[2] = 0.0;
}

void LaserScanMatcher::laserScanToLDP(const sensor_msgs::LaserScan::ConstPtr& scan_msg,
                                            LDP& ldp)
{
  unsigned int n = scan_msg->ranges.size();
  ldp = ld_alloc_new(n);

  for (unsigned int i = 0; i < n; i++)
  {
    // calculate position in laser frame

    double r = scan_msg->ranges[i];

    if (r > scan_msg->range_min && r < scan_msg->range_max)
    {
      // fill in laser scan data

      ldp->valid[i] = 1;
      ldp->readings[i] = r;
    }
    else
    {
      ldp->valid[i] = 0;
      ldp->readings[i] = -1;  // for invalid range
    }

    ldp->theta[i]    = scan_msg->angle_min + i * scan_msg->angle_increment;

    ldp->cluster[i]  = -1;
  }

  ldp->min_theta = ldp->theta[0];
  ldp->max_theta = ldp->theta[n-1];

  ldp->odometry[0] = 0.0;
  ldp->odometry[1] = 0.0;
  ldp->odometry[2] = 0.0;

  ldp->true_pose[0] = 0.0;
  ldp->true_pose[1] = 0.0;
  ldp->true_pose[2] = 0.0;
}

void LaserScanMatcher::createCache (const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  a_cos_.clear();
  a_sin_.clear();

  for (unsigned int i = 0; i < scan_msg->ranges.size(); ++i)
  {
    double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
    a_cos_.push_back(cos(angle));
    a_sin_.push_back(sin(angle));
  }

  input_.min_reading = scan_msg->range_min;
  input_.max_reading = scan_msg->range_max;
}

bool LaserScanMatcher::getBaseToLaserTf (const std::string& frame_id)
{
  ros::Time t = ros::Time::now();

  try
  {
	base_to_laser_ = extrinsics_.GetExtrinsics(base_frame_,
                                               frame_id,
											   t);
	laser_to_base_ = base_to_laser_.Inverse();
	return true;
  }
  catch(argus::ExtrinsicsException)
  {
	ROS_WARN_STREAM( "Could not get extrinsics of " << frame_id << " to "
	                 << base_frame_ );
	return false;
  }

//   tf::StampedTransform base_to_laser_tf;
//   try
//   {
//     tf_listener_.waitForTransform(
//       base_frame_, frame_id, t, ros::Duration(1.0));
//     tf_listener_.lookupTransform (
//       base_frame_, frame_id, t, base_to_laser_tf);
//   }
//   catch (tf::TransformException ex)
//   {
//     ROS_WARN("Could not get initial transform from base to laser frame, %s", ex.what());
//     return false;
//   }
//   base_to_laser_ = base_to_laser_tf;
//   laser_to_base_ = base_to_laser_.inverse();

//   return true;
}

// returns the predicted change in pose (in fixed frame)
// since the last time we did icp
void LaserScanMatcher::getPrediction(double& pr_ch_x, double& pr_ch_y,
                                     double& pr_ch_a, double dt)
{
  boost::mutex::scoped_lock(mutex_);

  // **** base case - no input available, use zero-motion model
  pr_ch_x = 0.0;
  pr_ch_y = 0.0;
  pr_ch_a = 0.0;

  // **** use velocity (for example from ab-filter)
  if (use_vel_)
  {
    pr_ch_x = dt * latest_vel_msg_.linear.x;
    pr_ch_y = dt * latest_vel_msg_.linear.y;
    pr_ch_a = dt * latest_vel_msg_.angular.z;

    if      (pr_ch_a >= M_PI) pr_ch_a -= 2.0 * M_PI;
    else if (pr_ch_a < -M_PI) pr_ch_a += 2.0 * M_PI;
  }

  // **** use wheel odometry
  if (use_odom_ && received_odom_)
  {
	argus::PoseSE3 latestOdom = argus::MsgToPose(latest_odom_msg_.pose.pose);
	argus::PoseSE3 lastUsedOdom = argus::MsgToPose(last_used_odom_msg_.pose.pose);
	argus::PoseSE2 latestOdom2 = argus::PoseSE2::FromSE3(latestOdom);
	argus::PoseSE2 lastUsedOdom2 = argus::PoseSE2::FromSE3(lastUsedOdom);

    //pr_ch_x = latest_odom_msg_.pose.pose.position.x -
    //          last_used_odom_msg_.pose.pose.position.x;
	pr_ch_x = latestOdom2.GetTranslation().x() -
	          lastUsedOdom2.GetTranslation().x();

    // pr_ch_y = latest_odom_msg_.pose.pose.position.y -
    //           last_used_odom_msg_.pose.pose.position.y;
	pr_ch_y = latestOdom2.GetTranslation().y() -
	          lastUsedOdom2.GetTranslation().y();

    // pr_ch_a = tf::getYaw(latest_odom_msg_.pose.pose.orientation) -
    //           tf::getYaw(last_used_odom_msg_.pose.pose.orientation);
	pr_ch_a = latestOdom2.GetRotation().angle() -
	          lastUsedOdom2.GetRotation().angle();

    if      (pr_ch_a >= M_PI) pr_ch_a -= 2.0 * M_PI;
    else if (pr_ch_a < -M_PI) pr_ch_a += 2.0 * M_PI;

    last_used_odom_msg_ = latest_odom_msg_;
  }

  // **** use imu
  if (use_imu_ && received_imu_)
  {
	argus::QuaternionType latestImu = argus::MsgToQuaternion(latest_imu_msg_.orientation);
	argus::QuaternionType lastUsedImu = argus::MsgToQuaternion(last_used_imu_msg_.orientation);
	argus::EulerAngles latestEuler = argus::QuaternionToEuler(latestImu);
	argus::EulerAngles lastUsedEuler = argus::QuaternionToEuler(lastUsedImu);

	pr_ch_a = latestEuler.yaw - lastUsedEuler.yaw;
    // pr_ch_a = tf::getYaw(latest_imu_msg_.orientation) -
            //   tf::getYaw(last_used_imu_msg_.orientation);

    if      (pr_ch_a >= M_PI) pr_ch_a -= 2.0 * M_PI;
    else if (pr_ch_a < -M_PI) pr_ch_a += 2.0 * M_PI;

    last_used_imu_msg_ = latest_imu_msg_;
  }
}

// void LaserScanMatcher::createTfFromXYTheta(
//   double x, double y, double theta, tf::Transform& t)
// {
//   t.setOrigin(tf::Vector3(x, y, 0.0));
//   tf::Quaternion q;
//   q.setRPY(0.0, 0.0, theta);
//   t.setRotation(q);
// }

} // namespace scan_tools
