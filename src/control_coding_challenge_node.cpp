#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>

class SensorFusion
{
public:
  SensorFusion();
  SensorFusion(const SensorFusion& other);
  virtual ~SensorFusion();

  ros::Publisher dataPublisher;
  void groundRangeCallback(const sensor_msgs::Range &msg);
  void imuCallback(const sensor_msgs::Imu &msg);
  void callback(const sensor_msgs::Range &ground_msg, const sensor_msgs::Imu &imu_msg);

};

SensorFusion::SensorFusion()
{
}

SensorFusion::SensorFusion(const SensorFusion& other)
{
}

SensorFusion::~SensorFusion()
{
}

void SensorFusion::callback(const sensor_msgs::Range &ground_msg, const sensor_msgs::Imu &imu_msg)
{
  std::cout << "ground " << ground_msg.header.stamp << std::endl;
  std::cout << "ground " << ground_msg.range << std::endl;

  std::cout << "imu " << imu_msg.header.stamp << std::endl;
}


void SensorFusion::groundRangeCallback(const sensor_msgs::Range &msg)
{
  std::cout << "ground " << msg.header.stamp << std::endl;
  std::cout << "ground " << msg.range << std::endl;
}

void SensorFusion::imuCallback(const sensor_msgs::Imu &msg)
{
    std::cout << "imu " << msg.header.stamp << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "control_coding_challenge");
  ros::NodeHandle node;

  message_filters::Subscriber<sensor_msgs::Range> ground_sub(node, "/ifm_sys/distance/ground", 1);
  message_filters::Subscriber<sensor_msgs::Imu> imu_sub(node, "/ifm_sys/imu/data", 1);


  message_filters::TimeSynchronizer<sensor_msgs::Range, sensor_msgs::Imu> sync(ground_sub, imu_sub, 10);
  SensorFusion sensor_fusion;
  sync.registerCallback(boost::bind(&SensorFusion::callback, &sensor_fusion, _1, _2));
//  ros::Subscriber ground_sub = node.subscribe("/ifm_sys/distance/ground", 100, &SensorFusion::groundRangeCallback, &sensor_fusion);
//  ros::Subscriber imu_sub = node.subscribe("/ifm_sys/imu/data", 100, &SensorFusion::imuCallback, &sensor_fusion);

  while (ros::ok())
  {
      ros::spinOnce(); // check for incoming messages
  }

  return EXIT_SUCCESS;
}
