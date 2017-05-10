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
  gtsam::Pose2 prior_pose(); // prior mean is at origin

  current_robot_sym_ = gtsam::Symbol('x', robot_pose_counter_);

  factor_graph_.add(gtsam::PriorFactor<gtsam::Pose2>(current_robot_sym_, gtsam::Pose2(), prior_pose_noise_)); // add directly to graph
  init_est_.insert(current_robot_sym_, gtsam::Pose2());

  // TODO: need to figure out how to deal with multiple objects - probably will need a dictionary mapping the segnet labeled with their respective symbols
  // will need to wait on what the status with landmarks is before adding this

  // Creating the landmark symbol
  landmark_sym_ = gtsam::Symbol('l', 1);
}

// Initialize the noise models
void Localization::initialize_noise_models()
{
  gtsam::Vector prior_sigmas(6);
  prior_sigmas << prior_trans_stddev_, prior_trans_stddev_, prior_trans_stddev_, prior_rot_stddev_, prior_rot_stddev_, prior_rot_stddev_;
  prior_pose_noise_ = gtsam::noiseModel::Diagonal::Sigmas(prior_sigmas);

  gtsam::Vector odom_sigmas(6);
  odom_sigmas << odom_trans_stddev_, odom_trans_stddev_, odom_trans_stddev_, odom_rot_stddev_, odom_rot_stddev_, odom_rot_stddev_;
  odom_noise_ = gtsam::noiseModel::Diagonal::Sigmas(odom_sigmas);

  // TODO: probably want to have one for good measurements and one for not so good measurements
  gtsam::Vector obs_sigmas(6);
  obs_sigmas << land_obs_trans_stddev_, land_obs_trans_stddev_, land_obs_trans_stddev_, land_obs_rot_stddev_, land_obs_rot_stddev_, land_obs_rot_stddev_;
  land_obs_noise_ = gtsam::noiseModel::Diagonal::Sigmas(obs_sigmas);
}

// Adds an odometry measurement to iSAM2 and returns the current estimated state
Eigen::Matrix3f Localization::add_odom_measurement(Eigen::Matrix3f odom_measurement)
{
  robot_pose_counter_ += 1;
  gtsam::Symbol next_robot_sym = gtsam::Symbol('x', robot_pose_counter_);

  Eigen::MatrixXd odom_meas = odom_measurement.cast<double>();

  // Update the factor graph with the transformation (need to increment the counter too)
  gtsam::Pose2 robot_odometry(odom_meas);
  factor_graph_.add(gtsam::BetweenFactor<gtsam::Pose2> (current_robot_sym_, next_robot_sym, robot_odometry, odom_noise_));

  current_robot_sym_ = next_robot_sym;

  // Generating the initial estimates
  if (robot_pose_counter_ == 1)
  {
    init_est_.insert(current_robot_sym_, robot_odometry);
  }

  else 
  {
    Eigen::Matrix3f global_T_robot_prev = est_robot_pose_.matrix().cast<float>();
    Eigen::Matrix3f global_T_robot_now = global_T_robot_prev * odom_measurement;

    Eigen::MatrixXd global_T_robot_now_est = global_T_robot_now.cast<double>();

    gtsam::Pose2 est(global_T_robot_now_est);
    init_est_.insert(current_robot_sym_, est);
  }
  // Optimize the factor graph
  optimize_factor_graph();

  // Update the current estimated robot pose
  est_robot_pose_ = est_state_.at<gtsam::Pose2>(current_robot_sym_);

  return est_robot_pose_.matrix().cast<float>();
}

// Adds a landmark measurement to iSAM2 and returns the current estimated state of the landmark
Eigen::Matrix3f Localization::add_landmark_measurement(Eigen::Matrix3f landmark_measurement)
{
  Eigen::MatrixXd landmark_meas = landmark_measurement.cast<double>();
  gtsam::Pose2 measurement(landmark_meas);
  factor_graph_.add(gtsam::BetweenFactor<gtsam::Pose2> (current_robot_sym_, landmark_sym_, measurement, land_obs_noise_));

  landmark_obs_counter_ += 1;

  // TODO: if this is the first time that the landmark is visited, then need to put in the initial estimate
  // (but if it is not, then DO NOT put in the initial estimates)

  if (landmark_obs_counter_ == 1)
  {
    // TODO: need to add note to the estimate or just make it the origin b/c this is the exact pose
    init_est_.insert(landmark_sym_, measurement);
  }
  // TODO: with multiple landmark observations will need to figure out the data associations

  // Optimize the factor graph
  optimize_factor_graph();

  // Update the current estimated landmark pose
  est_landmark_pose_ = est_state_.at<gtsam::Pose2>(landmark_sym_);
  // Update the current estimated robot pose
  est_robot_pose_ = est_state_.at<gtsam::Pose2>(current_robot_sym_);

  return est_landmark_pose_.matrix().cast<float>();
}

// Optimizes the factor graph
void Localization::optimize_factor_graph()
{
  // Update iSAM with the new factors
  isam2_.update(factor_graph_, init_est_);
  // // TODO: may want to call update for more accuracy but more time

  // Get the current iSAM2 estimate
  est_state_ = isam2_.calculateEstimate();

  // Clear the factor graph and values for the next iteration
  factor_graph_.resize(0);
  init_est_.clear();
}

// Returns the estimated robot pose
Eigen::Matrix3f Localization::get_est_robot_pose()
{
  return est_robot_pose_.matrix().cast<float>();
}

// Returns the estimated landmark pose
Eigen::Matrix3f Localization::get_est_landmark_pose()
{
  return est_landmark_pose_.matrix().cast<float>();
}

}  // namespace slam