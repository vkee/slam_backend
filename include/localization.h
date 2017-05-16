#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <iostream>
#include <sstream>
#include <vector>
#include <unordered_map>

// GTSAM Stuff
#include <gtsam/geometry/Pose2.h>

// Each variable in the system (poses and landmarks) must be identified with a unique key.
// We can either use simple integer keys (1, 2, 3, ...) or symbols (X1, X2, L1).
// Here we will use Symbols
#include <gtsam/inference/Symbol.h>

// We want to use iSAM2 to solve the structure-from-motion problem incrementally, so
// include iSAM2 here
#include <gtsam/nonlinear/ISAM2.h>

// iSAM2 requires as input a set set of new factors to be added stored in a factor graph,
// and initial guesses for any new variables used in the added factors
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

// Once the optimized values have been calculated, we can also calculate the marginal covariance
// of desired variables
#include <gtsam/nonlinear/Marginals.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// Also, we will initialize the robot at some location using a Prior factor.
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/BearingRangeFactor.h>

namespace slam {

class Localization
{
	public:
    // Constructors
    Localization();
    Localization(double isam2_relinearize_thresh, double isam2_relinearize_skip, double prior_trans_stddev, 
      double prior_rot_stddev, double odom_trans_stddev, double odom_rot_stddev, double land_obs_trans_stddev, 
      double land_obs_rot_stddev);
    // Destructor
    ~Localization();

    // 2D Pose Struct
    struct Pose2D {
      double x;
      double y;
      double theta;
    };

    // Landmark Info for HashMap
    struct LandmarkInfo {
      // Symbol of the landmark
      gtsam::Symbol land_sym;
      // Number of times the landmark is observed
      int num_obs;
      // Symbol of the robot pose at obs time
      gtsam::Symbol robot_pose_sym;
      // Measured transform from the robot (at obs time) to landmark
      gtsam::Pose2 robot_T_land;
      // Estimated pose of landmark in world
      gtsam::Pose2 world_T_land;
    };

    // Initializes localization
    void init_localization(double isam2_relinearize_thresh, double isam2_relinearize_skip, 
      double prior_trans_stddev, double prior_rot_stddev, double odom_trans_stddev, double odom_rot_stddev,
      double land_obs_trans_stddev, double land_obs_rot_stddev);

    // // Adds an odometry measurement to iSAM2 and returns the current estimated state
    // Localization::Pose2D add_odom_measurement(double x, double y, double theta);

    // Adds an odometry measurement to iSAM2 and returns the current estimated state
    Localization::Pose2D add_odom_measurement(double odom_x, double odom_y, double odom_theta, 
        double global_x, double global_y, double global_theta);

    // Adds/stores a landmark measurement to iSAM2 and returns whether the factor graph can be optimized
    bool add_landmark_measurement(int landmark_id, double land_rel_x, double land_rel_y, double land_rel_theta, 
      double land_glob_x, double land_glob_y, double land_glob_theta);

    // Optimizes the factor graph
    void optimize_factor_graph();

    // Returns the estimated robot pose
    Localization::Pose2D get_est_robot_pose();

	private:
    // Initialize iSAM2 with parameters
    void initialize_isam2();

    // Initialize the factor graph
    void initialize_factor_graph();

    // Initialize the noise models
    void initialize_noise_models();

    // Map of the landmark ids to LandmarkInfo
    std::unordered_map<int, Localization::LandmarkInfo> landmark_id_map_;

    std::string landmark_name_;
    std::string robot_name_;
    std::string est_robot_name_;
    std::string est_landmark_name_;

    gtsam::Symbol current_robot_sym_;

    // Counters for the robot pose and landmarks
    int robot_pose_counter_;
    int landmark_obs_counter_;

    // Only relinearize variables whose linear delta magnitude is greater than this threshold (default: 0.1). 
    double isam2_relinearize_thresh_;
    // Only relinearize any variables every relinearizeSkip calls to ISAM2::update (default: 10) 
    int isam2_relinearize_skip_;

    // Standard deviation of the translation and rotation components of the robot pose prior
    double prior_trans_stddev_;
    double prior_rot_stddev_;
    // Standard deviation of the translation and rotation components of the odometry measurements
    double odom_trans_stddev_;
    double odom_rot_stddev_;
    // Standard deviation of the translation and rotation components of the landmark observations
    double land_obs_trans_stddev_;
    double land_obs_rot_stddev_;

    // isam2 solver
    gtsam::ISAM2 isam2_;

    // The iSAM2 estimated state
    gtsam::Values est_state_;
    // The estimated pose of the robot
    gtsam::Pose2 est_robot_pose_;
    // The estimated pose of the landmark
    gtsam::Pose2 est_landmark_pose_;

    // The pose of the robot at the last time an optimization was done
    gtsam::Pose2 opt_robot_pose_;

    // Transformation from the last optimized pose to the current optimized pose
    gtsam::Pose2 last_opt_robot_T_robot_now_;

    // Create a Factor Graph and Values to hold the new data
    gtsam::NonlinearFactorGraph factor_graph_;
    gtsam::Values init_est_;

    // Noise model of the prior pose noise
    gtsam::noiseModel::Diagonal::shared_ptr prior_pose_noise_;
    // Noise model of the robot odometry
    gtsam::noiseModel::Diagonal::shared_ptr odom_noise_;
    // Noise model of the landmark observations
    gtsam::noiseModel::Diagonal::shared_ptr land_obs_noise_;
};

}  // namespace slam

#endif // LOCALIZATION_H