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

  // gtsam::Vector obs_sigmas(3);
  // obs_sigmas << land_obs_trans_stddev_, land_obs_trans_stddev_, land_obs_rot_stddev_;
  // land_obs_noise_ = gtsam::noiseModel::Diagonal::Sigmas(obs_sigmas);

  // Because now doing range bearing measurements
  gtsam::Vector obs_sigmas(2);
  obs_sigmas << land_obs_rot_stddev_, land_obs_trans_stddev_;
  land_obs_noise_ = gtsam::noiseModel::Diagonal::Sigmas(obs_sigmas);
}

// Adds an odometry measurement to iSAM2 and returns the current estimated state
Localization::Pose2D Localization::add_odom_measurement(double odom_x, double odom_y, double odom_theta,
  double global_x, double global_y, double global_theta)
{
  robot_pose_counter_ += 1;
  gtsam::Symbol next_robot_sym = gtsam::Symbol('x', robot_pose_counter_);

  // Update the factor graph with the transformation (need to increment the counter too)
  gtsam::Pose2 robot_odometry(odom_x, odom_y, odom_theta);
  factor_graph_.add(gtsam::BetweenFactor<gtsam::Pose2> (current_robot_sym_, next_robot_sym, robot_odometry, odom_noise_));

  current_robot_sym_ = next_robot_sym;

  // Generating the initial estimates
  if (robot_pose_counter_ == 1)
  {
    // Estimate is just odom b/c prior is origin
    init_est_.insert(current_robot_sym_, robot_odometry);
    est_robot_pose_ = robot_odometry;
  }

  else 
  {
    // Estimate is global pose

    gtsam::Pose2 est(global_x, global_y, global_theta);
    init_est_.insert(current_robot_sym_, est);
    est_robot_pose_ = est;
  }
}

// Adds/stores a landmark measurement to iSAM2 and returns whether the factor graph can be optimized
bool Localization::add_landmark_measurement(int landmark_id, double land_rel_x, double land_rel_y, double land_rel_theta, 
  double land_glob_x, double land_glob_y, double land_glob_theta)
{
  // Check if the landmark has been observed before
  if (landmark_id_map_.count(landmark_id) == 1)
  {
    // Get the landmark info
    Localization::LandmarkInfo landmark_info = landmark_id_map_.at(landmark_id);

    int num_obs = landmark_info.num_obs;
    gtsam::Symbol land_sym = landmark_info.land_sym;

    // If the landmark was observed only once, put the previous landmark measurement into the factor graph and add the initial estimate of the landmark
    if (num_obs == 1)
    {
      // Add the previous landmark measurement to the factor graph from the robot pose symbol
      // factor_graph_.add(gtsam::BetweenFactor<gtsam::Pose2> (landmark_info.robot_pose_sym, land_sym, landmark_info.robot_T_land, land_obs_noise_));
      
      // Add the previous landmark measurement to the factor graph from the robot pose symbol
      double x = landmark_info.robot_T_land.x();
      double y = landmark_info.robot_T_land.y();
      double range = sqrt(pow(x, 2) + pow(y, 2));
      factor_graph_.add(gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2>(
        landmark_info.robot_pose_sym, land_sym, landmark_info.robot_T_land.theta(), range, land_obs_noise_));
      
      // Add the initial estimate
      // init_est_.insert(land_sym, landmark_info.world_T_land);
      init_est_.insert(land_sym, gtsam::Point2(landmark_info.world_T_land.x(), landmark_info.world_T_land.y()));
    }

    // Construct the current landmark measurement
    // gtsam::Pose2 curr_land_meas(land_rel_x, land_rel_y, land_rel_theta);
    // Add the current measurement
    // factor_graph_.add(gtsam::BetweenFactor<gtsam::Pose2> (current_robot_sym_, land_sym, curr_land_meas, land_obs_noise_));
    double range = sqrt(pow(land_rel_x, 2) + pow(land_rel_y, 2));
    factor_graph_.add(gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2>(
      current_robot_sym_, land_sym, land_rel_theta, range, land_obs_noise_));

    // Update the dictionary (only the num_obs needs to be updated)
    // Increase the number of times the landmark was observed
    landmark_info.num_obs = landmark_info.num_obs + 1;

    // Update the dictionary entry
    landmark_id_map_[landmark_id] = landmark_info;

    // Can optimize the factor graph
    return true;
  }

  // Case where the landmark has not been observed before
  else
  {
    landmark_obs_counter_ += 1;

    // Creating the new landmark symbol and putting it in the dictionary
    gtsam::Symbol next_landmark_sym = gtsam::Symbol('l', landmark_obs_counter_);

    // Create the landmark entry 
    Localization::LandmarkInfo landmark_info;

    landmark_info.land_sym = next_landmark_sym;
    landmark_info.num_obs = 1;
    landmark_info.robot_pose_sym = current_robot_sym_;
    // Construct the current landmark measurement
    gtsam::Pose2 curr_land_meas(land_rel_x, land_rel_y, land_rel_theta);
    landmark_info.robot_T_land = curr_land_meas;
    // Construct the global landmark measurement
    gtsam::Pose2 glob_land_meas(land_glob_x, land_glob_y, land_glob_theta);
    landmark_info.world_T_land = glob_land_meas;

    landmark_id_map_[landmark_id] = landmark_info;
    return false;
  }
}

// Optimizes the factor graph
void Localization::optimize_factor_graph()
{
  // Update iSAM with the new factors
  isam2_.update(factor_graph_, init_est_);
  isam2_.update();
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