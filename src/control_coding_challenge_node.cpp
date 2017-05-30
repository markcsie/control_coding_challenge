#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>

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
  void callback(const sensor_msgs::RangeConstPtr &ground_msg, const sensor_msgs::ImuConstPtr &imu_msg);

protected:
  bool initialized_;
  ros::Time prev_stamp_;
  double r_;
  double q_;
  double prev_linear_acceleration_z_;
  double filtered_z_;
  double variance_z_;
  std::queue<sensor_msgs::Imu> imu_queue_;
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

void SensorFusion::imuCallback(const sensor_msgs::Imu &msg)
{
  // cache imu messages for later use
  imu_queue_.push(msg);
  std::cout << "imu frame_id " << msg.header.frame_id << std::endl;
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

  ros::spin();

  return EXIT_SUCCESS;
}
