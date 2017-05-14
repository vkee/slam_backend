#include "localization.h"

namespace slam {

Localization::Localization():robot_pose_counter_ (0), landmark_obs_counter_(0)
{
}

Localization::Localization(double isam2_relinearize_thresh, double isam2_relinearize_skip, 
    double prior_trans_stddev, double prior_rot_stddev, double odom_trans_stddev, double odom_rot_stddev,
    double land_obs_trans_stddev, double land_obs_rot_stddev): 
      isam2_relinearize_thresh_(isam2_relinearize_thresh), isam2_relinearize_skip_(isam2_relinearize_skip), 
      prior_trans_stddev_(prior_trans_stddev), prior_rot_stddev_(prior_rot_stddev), 
      odom_trans_stddev_(odom_trans_stddev), odom_rot_stddev_(odom_rot_stddev), 
      land_obs_trans_stddev_(land_obs_trans_stddev), land_obs_rot_stddev_(land_obs_rot_stddev),
      robot_pose_counter_ (0), landmark_obs_counter_(0)
{
  // Initialize iSAM2
  initialize_isam2();

  // Initialize the noise models
  initialize_noise_models();

  // Initialize the factor graph
  initialize_factor_graph();
}

Localization::~Localization()
{
}

// Initializes localization
void Localization::init_localization(double isam2_relinearize_thresh, double isam2_relinearize_skip, 
    double prior_trans_stddev, double prior_rot_stddev, double odom_trans_stddev, double odom_rot_stddev,
    double land_obs_trans_stddev, double land_obs_rot_stddev)
{
  isam2_relinearize_thresh_ = isam2_relinearize_thresh;
  isam2_relinearize_skip_ = isam2_relinearize_skip;
  prior_trans_stddev_ = prior_trans_stddev;
  prior_rot_stddev_ = prior_rot_stddev;
  odom_trans_stddev_ = odom_trans_stddev;
  odom_rot_stddev_ = odom_rot_stddev_;
  land_obs_trans_stddev_ = land_obs_trans_stddev;
  land_obs_rot_stddev_ = land_obs_rot_stddev;      

  // Initialize iSAM2
  initialize_isam2();

  // Initialize the noise models
  initialize_noise_models();

  // Initialize the factor graph
  initialize_factor_graph();
}

// Initialize iSAM2 with parameters
void Localization::initialize_isam2()
{
  gtsam::ISAM2Params parameters;
  parameters.relinearizeThreshold = isam2_relinearize_thresh_;
  parameters.relinearizeSkip = isam2_relinearize_skip_;
  isam2_ = gtsam::ISAM2(parameters);
}

// Initialize the factor graph
void Localization::initialize_factor_graph()
{
  // Add a prior on pose x1 at the origin. A prior factor consists of a mean and a noise model (covariance matrix)
  gtsam::Pose2 prior_pose; // prior mean is at origin

  current_robot_sym_ = gtsam::Symbol('x', robot_pose_counter_);

  factor_graph_.add(gtsam::PriorFactor<gtsam::Pose2>(current_robot_sym_, gtsam::Pose2(), prior_pose_noise_)); // add directly to graph
  init_est_.insert(current_robot_sym_, gtsam::Pose2());
}

// Initialize the noise models
void Localization::initialize_noise_models()
{
  gtsam::Vector prior_sigmas(3);
  prior_sigmas << prior_trans_stddev_, prior_trans_stddev_, prior_rot_stddev_;
  prior_pose_noise_ = gtsam::noiseModel::Diagonal::Sigmas(prior_sigmas);

  gtsam::Vector odom_sigmas(3);
  odom_sigmas << odom_trans_stddev_, odom_trans_stddev_, odom_rot_stddev_;
  odom_noise_ = gtsam::noiseModel::Diagonal::Sigmas(odom_sigmas);

  gtsam::Vector obs_sigmas(3);
  obs_sigmas << land_obs_trans_stddev_, land_obs_trans_stddev_, land_obs_rot_stddev_;
  land_obs_noise_ = gtsam::noiseModel::Diagonal::Sigmas(obs_sigmas);
}

// Adds an odometry measurement to iSAM2 and returns the current estimated state
Localization::Pose2D Localization::add_odom_measurement(double x, double y, double theta)
{
  robot_pose_counter_ += 1;
  gtsam::Symbol next_robot_sym = gtsam::Symbol('x', robot_pose_counter_);

  std::cout << "current_robot_sym_: " << current_robot_sym_ << std::endl;
  std::cout << "next_robot_sym: " << next_robot_sym << std::endl;

  // Update the factor graph with the transformation (need to increment the counter too)
  gtsam::Pose2 robot_odometry(x, y, theta);
  factor_graph_.add(gtsam::BetweenFactor<gtsam::Pose2> (current_robot_sym_, next_robot_sym, robot_odometry, odom_noise_));

  current_robot_sym_ = next_robot_sym;

  // Generating the initial estimates
  if (robot_pose_counter_ == 1)
  {
    // Estimate is just odom b/c prior is origin
    init_est_.insert(current_robot_sym_, robot_odometry);
  }

  else 
  {
    // Estimate is robot previous estimated pose concatenated with odom measurement
    Eigen::Matrix3d global_T_robot_prev = est_robot_pose_.matrix().cast<double>();
    Eigen::MatrixXd global_T_robot_now = global_T_robot_prev * robot_odometry.matrix().cast<double>();

    gtsam::Pose2 est(global_T_robot_now);
    init_est_.insert(current_robot_sym_, est);
  }

  // Update the robot pose wrt to the last optimized robot pose
  last_opt_robot_T_robot_now_ = last_opt_robot_T_robot_now_ * robot_odometry;

  // Update the current estimated robot pose (last optimized pose * last_opt_pose_T_curr_robot_pose
  est_robot_pose_ = opt_robot_pose_ * last_opt_robot_T_robot_now_;

  Localization::Pose2D pose2D;
  pose2D.x = est_robot_pose_.x();
  pose2D.y = est_robot_pose_.y();
  pose2D.theta = est_robot_pose_.theta();

  return pose2D;
}

// Adds a landmark measurement to iSAM2
void Localization::add_landmark_measurement(int landmark_id, double x, double y, double theta)
{
  gtsam::Symbol landmark_sym;
  bool new_landmark;

  // Check if the landmark has been observed before
  if (landmark_id_map_.count(landmark_id) == 1)
  {
    // Get the landmark symbol
    landmark_sym = landmark_id_map_.at(landmark_id);
    new_landmark = false;
  }

  // Case where the landmark has not been observed before
  else
  {
    landmark_obs_counter_ += 1;

    // Creating the new landmark symbol and putting it in the dictionary
    gtsam::Symbol next_landmark_sym = gtsam::Symbol('l', landmark_obs_counter_);
    landmark_id_map_.at(landmark_id) = next_landmark_sym;

    landmark_sym = next_landmark_sym;
    new_landmark = true;
  }

  // Construct the landmark measurement
  gtsam::Pose2 land_meas(x, y, theta);
  // Add the measurement to the factor graph from the current robot pose symbol
  factor_graph_.add(gtsam::BetweenFactor<gtsam::Pose2> (current_robot_sym_, landmark_sym, land_meas, land_obs_noise_));

  // If the landmark has not been unobserved before, need to add initial estimate
  if (new_landmark)
  {
    // Estimate is robot estimated pose concatenated with landmark measurement
    Eigen::Matrix3d global_T_robot = est_robot_pose_.matrix().cast<double>();
    Eigen::MatrixXd global_T_landmark = global_T_robot * land_meas.matrix().cast<double>();

    gtsam::Pose2 est(global_T_landmark);
    init_est_.insert(landmark_sym, est);
  }
}

// Optimizes the factor graph
void Localization::optimize_factor_graph()
{
  // Update iSAM with the new factors
  isam2_.update(factor_graph_, init_est_);
  // // TODO: may want to call update for more accuracy but more time

  // Get the current iSAM2 estimate
  est_state_ = isam2_.calculateEstimate();
  // Update the current estimated robot pose
  est_robot_pose_ = est_state_.at<gtsam::Pose2>(current_robot_sym_);
  opt_robot_pose_ = est_robot_pose_;
  // Identity since the optimized pose is the current robot pose
  last_opt_robot_T_robot_now_ = gtsam::Pose2();

  // Clear the factor graph and values for the next iteration
  factor_graph_.resize(0);
  init_est_.clear();
}

// Returns the estimated robot pose
Localization::Pose2D Localization::get_est_robot_pose()
{
  Localization::Pose2D pose2D;
  pose2D.x = est_robot_pose_.x();
  pose2D.y = est_robot_pose_.y();
  pose2D.theta = est_robot_pose_.theta();

  return pose2D;
}
}  // namespace slam