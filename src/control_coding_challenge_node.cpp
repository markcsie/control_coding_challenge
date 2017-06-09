#include <queue>

#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "sensor_fusion.h"
#include "control_coding_challenge/NoiseMatrix.h"

class ControlCodingChallenge
{
public:
  ControlCodingChallenge(SensorFusion &sensor_fusion);
  virtual ~ControlCodingChallenge();

  void groundRangeCallback(const sensor_msgs::Range &msg);
  void imuCallback(const sensor_msgs::Imu &msg);
  void rNoiseCallback(const control_coding_challenge::NoiseMatrix &msg);
  void qNoiseCallback(const control_coding_challenge::NoiseMatrix &msg);
  void callback(const sensor_msgs::RangeConstPtr &ground_msg, const sensor_msgs::ImuConstPtr &imu_msg);

protected:
  bool time_initialized_;
  SensorFusion &sensor_fusion_;
  ros::Time prev_stamp_;
  Eigen::VectorXd u_;
  std::queue<sensor_msgs::Imu> imu_queue_;
};

ControlCodingChallenge::ControlCodingChallenge(SensorFusion &sensor_fusion) : sensor_fusion_(sensor_fusion), time_initialized_(false)
{
}

ControlCodingChallenge::~ControlCodingChallenge()
{
}

void ControlCodingChallenge::rNoiseCallback(const control_coding_challenge::NoiseMatrix &msg)
{
  if (msg.matrix.size() != SensorFusion::StateDim * SensorFusion::StateDim)
  {
    std::cerr << "wrong message for rNoise" << std::endl;
    return;
  }

  Eigen::MatrixXd r_noise = Eigen::MatrixXd::Zero(SensorFusion::StateDim, SensorFusion::StateDim);
  for (size_t row = 0; row < SensorFusion::StateDim; row++)
  {
    for (size_t col = 0; col < SensorFusion::StateDim; col++)
    {
      r_noise(row, col) = msg.matrix[row * SensorFusion::StateDim + col];
    }
  }

  sensor_fusion_.setStateNoiseR(r_noise);
}

void ControlCodingChallenge::qNoiseCallback(const control_coding_challenge::NoiseMatrix &msg)
{
  if (msg.matrix.size() != SensorFusion::MeasurementDim * SensorFusion::MeasurementDim)
  {
    std::cerr << "wrong message for qNoise" << std::endl;
    return;
  }

  Eigen::MatrixXd q_noise = Eigen::MatrixXd::Zero(SensorFusion::MeasurementDim, SensorFusion::MeasurementDim);
  for (size_t row = 0; row < SensorFusion::MeasurementDim; row++)
  {
    for (size_t col = 0; col < SensorFusion::MeasurementDim; col++)
    {
      q_noise(row, col) = msg.matrix[row * SensorFusion::MeasurementDim + col];
    }
  }
  sensor_fusion_.setRangeNoiseQ(q_noise);
}

void ControlCodingChallenge::imuCallback(const sensor_msgs::Imu &msg)
{
  // cache imu messages for later use
  imu_queue_.push(msg);
}

void ControlCodingChallenge::groundRangeCallback(const sensor_msgs::Range &range_msg)
{
  if (!time_initialized_)
  {
    prev_stamp_ = range_msg.header.stamp;
    u_ = Eigen::VectorXd::Zero(SensorFusion::ControlDim);
    time_initialized_ = true;
  }
  else
  {
    // motion update
    while (!imu_queue_.empty() && imu_queue_.front().header.stamp <= range_msg.header.stamp)
    {
      ros::Duration delta_t = imu_queue_.front().header.stamp - prev_stamp_;

      // NOTE: The orientation of x_t is from u_t not u_{t-1}
      double roll;
      double pitch;
      double yaw;
      tf2::Quaternion q(imu_queue_.front().orientation.x, imu_queue_.front().orientation.y, imu_queue_.front().orientation.z, imu_queue_.front().orientation.w);
      tf2::Matrix3x3 mat(q);
      mat.getEulerYPR(yaw, pitch, roll);
      u_(SensorFusion::ControlRoll) = roll;
      u_(SensorFusion::ControlPitch) = pitch;
      u_(SensorFusion::ControlYaw) = yaw;

      sensor_fusion_.ekfPredict(u_, delta_t.toSec());

      // NOTE: The velocity of x_t is calculated from a_{t-1} not a_t
      u_(SensorFusion::ControlAx) = imu_queue_.front().linear_acceleration.x;
      u_(SensorFusion::ControlAy) = imu_queue_.front().linear_acceleration.y;
      u_(SensorFusion::ControlAz) = imu_queue_.front().linear_acceleration.z;

      prev_stamp_ = imu_queue_.front().header.stamp;

      imu_queue_.pop();
    }
    // Predict again to sychronize the time
    ros::Duration delta_t = range_msg.header.stamp - prev_stamp_;
    if (!delta_t.isZero())
    {
      sensor_fusion_.ekfPredict(u_, delta_t.toSec());
    }

    // measurement update
    sensor_fusion_.ekfCorrect(range_msg.range);
  }

  std::cout << "range from sensor " << range_msg.range << std::endl;
  std::cout << "filtered_height_ " << (sensor_fusion_.getX())(SensorFusion::StateZ) << std::endl;
}



int main(int argc, char **argv)
{
  SensorFusion sensor_fusion;
  // initial state??
  Eigen::VectorXd initial_x = Eigen::VectorXd::Zero(SensorFusion::StateDim);
  // initial covariance??
  Eigen::MatrixXd initial_covariance = Eigen::MatrixXd::Identity(initial_x.rows(), initial_x.rows());
  // parameters
  Eigen::MatrixXd initial_state_noise_r_ = Eigen::MatrixXd::Identity(initial_x.rows(), initial_x.rows());
  Eigen::MatrixXd initial_range_noise_q_ = Eigen::MatrixXd::Identity(SensorFusion::MeasurementDim, SensorFusion::MeasurementDim);
  sensor_fusion.initialize(initial_x, initial_covariance, initial_state_noise_r_, initial_range_noise_q_);

  ControlCodingChallenge control(sensor_fusion);

  ros::init(argc, argv, "control_coding_challenge");
  ros::NodeHandle node;
  ros::Subscriber imu_sub = node.subscribe("/ifm_sys/imu/data", 100, &ControlCodingChallenge::imuCallback, &control);
  ros::Subscriber ground_sub = node.subscribe("/ifm_sys/distance/ground", 100, &ControlCodingChallenge::groundRangeCallback, &control);

  ros::Subscriber r_noise_sub = node.subscribe("/ifm_sys/sensor_fusion/r_noise", 100, &ControlCodingChallenge::rNoiseCallback, &control);
  ros::Subscriber q_noise_sub = node.subscribe("/ifm_sys/sensor_fusion/q_noise", 100, &ControlCodingChallenge::qNoiseCallback, &control);

  ros::spin();

  return EXIT_SUCCESS;
}
