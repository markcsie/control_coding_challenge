#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>

#include <Eigen/Dense>
#include <queue>


class SensorFusion
{
public:
  SensorFusion(const double &r, const double &q);
  SensorFusion(const SensorFusion& other);
  virtual ~SensorFusion();

  ros::Publisher dataPublisher;
  void groundRangeCallback(const sensor_msgs::Range &msg);
  void imuCallback(const sensor_msgs::Imu &msg);
  void rNoiseCallback(const std_msgs::Float64 &msg);
  void qNoiseCallback(const std_msgs::Float64 &msg);
  void callback(const sensor_msgs::RangeConstPtr &ground_msg, const sensor_msgs::ImuConstPtr &imu_msg);

protected:
  enum StateMembers // state with respect to the global frame
  {
    StateX,
    StateY,
    StateZ,
    StateVx,
    StateVy,
    StateVz,
    StateRoll,
    StatePitch,
    StateYaw,
  };
  bool initialized_;
  ros::Time prev_stamp_;
  double r_;
  double q_;
  double prev_linear_acceleration_z_;
  double filtered_z_;
  double variance_z_;
  std::queue<sensor_msgs::Imu> imu_queue_;

  Eigen::MatrixXd imu_covariance_r_;
  Eigen::MatrixXd range_variance_q_;
  Eigen::MatrixXd covariance_sigma_; // state covariance
  Eigen::VectorXd ekfPredict(const Eigen::VectorXd &x, const Eigen::VectorXd &u, const double &delta_t);
  Eigen::VectorXd ekfCorrect(const Eigen::VectorXd &x, const double &z);

};

SensorFusion::SensorFusion(const double &r, const double &q) : initialized_(false), r_(r), q_(q)
{
}

SensorFusion::SensorFusion(const SensorFusion& other)
{
}

SensorFusion::~SensorFusion()
{
}

//void SensorFusion::callback(const sensor_msgs::RangeConstPtr &ground_msg, const sensor_msgs::ImuConstPtr &imu_msg)
//{
//  //  std::cout << "ground " << ground_msg->header.stamp << std::endl;
//  //  std::cout << "imu " << imu_msg->header.stamp << std::endl;

//  //  std::cout << "ground " << ground_msg->range << std::endl;
//}

Eigen::VectorXd SensorFusion::ekfPredict(const Eigen::VectorXd &x, const Eigen::VectorXd &u, const double &delta_t)
{
  double roll = x(StateRoll);
  double pitch = x(StatePitch);
  double yaw = x(StateYaw);

  double sp = std::sin(pitch);
  double cp = std::cos(pitch);

  double sr = std::sin(roll);
  double cr = std::cos(roll);

  double sy = std::sin(yaw);
  double cy = std::cos(yaw);
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

  Eigen::VectorXd x_predict = x;
  // position
  x_predict.head(3) += u.segment(6, 3) * delta_t;
  // linear velocity
  Eigen::VectorXd a = u.segment(3, 3);
  x_predict.segment(3, 3) += rotation_matrix * 0.5 * a * delta_t * delta_t;
  // orientation, directly accquired from imu messages
  x_predict.segment(6, 3) = u.head(3);


  Eigen::MatrixXd jacobian_g;
  covariance_sigma_ = jacobian_g * covariance_sigma_ * jacobian_g.transpose() + imu_covariance_r_;

  return x_predict;
}

Eigen::VectorXd SensorFusion::ekfCorrect(const Eigen::VectorXd &x, const double &z)
{
  Eigen::MatrixXd jacobian_h;
  Eigen::MatrixXd kalman_gain = covariance_sigma_ * jacobian_h.transpose()*(jacobian_h * covariance_sigma_ * jacobian_h.transpose() + range_variance_q_).inverse();

  Eigen::VectorXd x_correct = x;
  double z_predict;
  x_correct += kalman_gain * (z - z_predict);
  covariance_sigma_ = (Eigen::MatrixXd::Identity(kalman_gain.rows(), kalman_gain.rows()) - kalman_gain * jacobian_h) * covariance_sigma_;

  return x_correct;
}


void SensorFusion::imuCallback(const sensor_msgs::Imu &msg)
{
  // cache imu messages for later use
  imu_queue_.push(msg);
  std::cout << "msg.orientation " << msg.orientation << std::endl;
  std::cout << "msg.orientation_covariance[0] " << msg.orientation_covariance[0] << std::endl;
  std::cout << "msg.orientation_covariance[1] " << msg.orientation_covariance[1] << std::endl;

}

void SensorFusion::rNoiseCallback(const std_msgs::Float64 &msg)
{
  r_ = msg.data;
  std::cout << "r_ updated " << r_ << std::endl;
}

void SensorFusion::qNoiseCallback(const std_msgs::Float64 &msg)
{
  q_ = msg.data;
  std::cout << "q_ updated " << q_ << std::endl;
}

void SensorFusion::groundRangeCallback(const sensor_msgs::Range &msg)
{
  std::cout << "range frame_id " << msg.header.frame_id << std::endl;
  if (!initialized_)
  {
    prev_stamp_ = msg.header.stamp;
    filtered_z_ = msg.range;
    variance_z_ = 1;
    initialized_ = true;
    prev_linear_acceleration_z_ = 0; // assumption
  }
  else
  {
    // motion update
    while (!imu_queue_.empty() && imu_queue_.front().header.stamp < msg.header.stamp)
    {
      std::cout << "imu prev_stamp_ " << prev_stamp_ << std::endl;
      std::cout << "imu prev_linear_acceleration_z_.z " << prev_linear_acceleration_z_ << std::endl;

      // for those imu messages received before initilization, discard them
      if (imu_queue_.front().header.stamp >= prev_stamp_)
      {
        ros::Duration delta_t = imu_queue_.front().header.stamp - prev_stamp_;
//        std::cout <<  "delta_t.toSec() " << delta_t.toSec() << std::endl;

        filtered_z_ = filtered_z_ + std::pow(delta_t.toSec(), 2) * prev_linear_acceleration_z_ / 2;
        variance_z_ = variance_z_ + r_;

        prev_linear_acceleration_z_ = imu_queue_.front().linear_acceleration.z;
        prev_stamp_ = imu_queue_.front().header.stamp;
      }

      imu_queue_.pop();
    }

    // measurement update
    double kalman_gain = variance_z_ / (variance_z_ + q_);
    filtered_z_ = filtered_z_ + kalman_gain * (msg.range - filtered_z_);
    variance_z_ = (1 - kalman_gain) * variance_z_;
  }

  std::cout << "time " << msg.header.stamp << std::endl;
  std::cout << "ground " << msg.range << std::endl;
  std::cout << "filtered_z_ " << filtered_z_ << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "control_coding_challenge");
  ros::NodeHandle node;

  SensorFusion sensor_fusion(1, 1);

  //  message_filters::Subscriber<sensor_msgs::Range> ground_sub(node, "/ifm_sys/distance/ground", 1);
  //  message_filters::Subscriber<sensor_msgs::Imu> imu_sub(node, "/ifm_sys/imu/data", 1);
  //  message_filters::TimeSynchronizer<sensor_msgs::Range, sensor_msgs::Imu> sync(ground_sub, imu_sub, 10);
  //  sync.registerCallback(boost::bind(&SensorFusion::callback, &sensor_fusion, _1, _2));

  ros::Subscriber imu_sub = node.subscribe("/ifm_sys/imu/data", 100, &SensorFusion::imuCallback, &sensor_fusion);
  ros::Subscriber ground_sub = node.subscribe("/ifm_sys/distance/ground", 100, &SensorFusion::groundRangeCallback, &sensor_fusion);

  ros::Subscriber r_noise_sub = node.subscribe("/ifm_sys/sensor_fusion/r_noise", 100, &SensorFusion::rNoiseCallback, &sensor_fusion);
  ros::Subscriber q_noise_sub = node.subscribe("/ifm_sys/sensor_fusion/q_noise", 100, &SensorFusion::qNoiseCallback, &sensor_fusion);


  ros::spin();

  return EXIT_SUCCESS;
}
