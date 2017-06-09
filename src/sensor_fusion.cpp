#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <Eigen/Dense>
#include <queue>

#include "sensor_fusion.h"

SensorFusion::SensorFusion() : initialized_(false)
{
}

SensorFusion::~SensorFusion()
{
}

void SensorFusion::initialize(const Eigen::VectorXd &initial_x,
                              const Eigen::MatrixXd &initial_covariance,
                              const Eigen::MatrixXd &initial_state_noise_r,
                              const Eigen::MatrixXd &initial_range_noise_q)
{
  x_ = initial_x;
  covariance_ = initial_covariance;

  state_noise_r_ = initial_state_noise_r;
  range_noise_q_ = initial_range_noise_q;

  initialized_ = true;
}

void SensorFusion::ekfPredict(const Eigen::VectorXd &u, const double &delta_t)
{
  if (!initialized_)
  {
    std::cerr << "Not initialized!!! " << std::endl;
    return;
  }

//  std::cout << "ekfPredict " << std::endl;
  Eigen::VectorXd orientation = u.head(3);
  Eigen::VectorXd a = u.tail(3);

  double sp = std::sin(x_(StatePitch));
  double cp = std::cos(x_(StatePitch));

  double sr = std::sin(x_(StateRoll));
  double cr = std::cos(x_(StateRoll));

  double sy = std::sin(x_(StateYaw));
  double cy = std::cos(x_(StateYaw));

  Eigen::MatrixXd jacobian_g = Eigen::MatrixXd::Zero(x_.rows(), x_.rows());
  jacobian_g(StateX, StateX) = 1;
  jacobian_g(StateX, StateVx) = delta_t;
  jacobian_g(StateY, StateY) = 1;
  jacobian_g(StateY, StateVy) = delta_t;
  jacobian_g(StateZ, StateZ) = 1;
  jacobian_g(StateZ, StateVz) = delta_t;
  jacobian_g(StateVx, StateVx) = 1;
  jacobian_g(StateVx, StateRoll) = delta_t * (cy * sp * cr * a(1) + sy * sr * a(1) - cy * sp * sr * a(2) + sy * cr * a(2));
  jacobian_g(StateVx, StatePitch) = delta_t * (-cy * sp * a(0) + cy * cp * sr * a(1) + cy * cp * cr * a(2));
  jacobian_g(StateVx, StateYaw) = delta_t * (-sy * cp * a(0) - sy * sp * sr * a(1) - cy * cr * a(1) - sy * sp * cr * a(2) + cy * sr * a(2));
  jacobian_g(StateVy, StateVy) = 1;
  jacobian_g(StateVy, StateRoll) = delta_t * (sy * sp * cr * a(1) - cy * sr * a(1) - sy * sp * sr * a(2) - cy * cr * a(2));
  jacobian_g(StateVy, StatePitch) = delta_t * (-sy * sp * a(0) + sy * cp * sr * a(1) + sy * cp * cr * a(2));
  jacobian_g(StateVy, StateYaw) = delta_t * (cy * cp * a(0) + cy * sp * sr * a(1) - sy * cr * a(1) + cy * sp * cr * a(2) + sy * sr * a(2));
  jacobian_g(StateVz, StateVz) = 1;
  jacobian_g(StateVz, StateRoll) = delta_t * (cp * cr * a(1) - cp * sr * a(2));
  jacobian_g(StateVz, StatePitch) = delta_t * (-cp * a(0) - sp * sr * a(1) - sp * cr * a(2));
  jacobian_g(StateVz, StateYaw) = 0;

  Eigen::MatrixXd rotation_matrix(3, 3);
  rotation_matrix(0, 0) = cy * cp;
  rotation_matrix(0, 1) = cy * sp * sr - sy * cr;
  rotation_matrix(0, 2) = cy * sp * cr + sy * sr;

  rotation_matrix(1, 0) = sy * cp;
  rotation_matrix(1, 1) = sy * sp * sr + cy * cr;
  rotation_matrix(1, 2) = sy * sp * cr - cy * sr;

  rotation_matrix(2, 0) = -sp;
  rotation_matrix(2, 1) = cp * sr;
  rotation_matrix(2, 2) = cp * cr;

  // position
  x_.head(3) += x_.segment(3, 3) * delta_t;
  // linear velocity
  x_.segment(3, 3) += rotation_matrix * a * delta_t;
  // orientation, directly accquired from imu messages
  x_.segment(6, 3) = orientation;

  covariance_ = jacobian_g * covariance_ * jacobian_g.transpose() + state_noise_r_;
}

void SensorFusion::ekfCorrect(const double &measurement)
{
  if (!initialized_)
  {
    std::cerr << "Not initialized!!! " << std::endl;
    return;
  }

//  std::cout << "ekfCorrect " << std::endl;
  double cp = std::cos(x_(StatePitch));
  double cr = std::cos(x_(StateRoll));
  double sp = std::sin(x_(StatePitch));
  double sr = std::sin(x_(StateRoll));

  Eigen::MatrixXd jacobian_h = Eigen::MatrixXd::Zero(1, x_.rows());
  jacobian_h(MeasurementRange, StateZ) = 1 / (cp * cr);
  jacobian_h(MeasurementRange, StateRoll) = -x_(StateZ) * sr / (cp * cr * cr);
  jacobian_h(MeasurementRange, StatePitch) = -x_(StateZ) * sp / (cp * cp * cr);
  Eigen::MatrixXd kalman_gain = covariance_ * jacobian_h.transpose()*(jacobian_h * covariance_ * jacobian_h.transpose() + range_noise_q_).inverse();

  double measurement_predict = x_(StateZ) / (cp * cr);
  x_ += kalman_gain * (measurement - measurement_predict);
  covariance_ = (Eigen::MatrixXd::Identity(kalman_gain.rows(), kalman_gain.rows()) - kalman_gain * jacobian_h) * covariance_;
}
