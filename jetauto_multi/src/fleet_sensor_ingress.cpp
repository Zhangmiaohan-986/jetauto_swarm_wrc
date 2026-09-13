#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_msgs/TFMessage.h>

#include <set>
#include <stdexcept>
#include <string>
#include <vector>

// One vehicle-side subscription per topic; consumers share the VM-side stream.
template <typename Message>
ros::Subscriber relay(ros::NodeHandle& nh, const std::string& robot,
                      const std::string& suffix)
{
  const std::string input = "/" + robot + "/" + suffix;
  const std::string output = suffix == "tf" ? "/fleet/tf" : "/fleet" + input;
  // This ingress must sit outside consumer remap groups: reject aliases that
  // could feed a relayed stream back into this or another ingress channel.
  if (nh.resolveName(input) != input || nh.resolveName(output) != output)
    throw std::runtime_error("sensor ingress endpoints must not be remapped: " + input);

  ros::AdvertiseOptions options;
  // TF merges distinct cars/children: queue=1 could overwrite another frame.
  options.init<Message>(output, suffix == "tf" ? 100 : 1);
  // roscpp otherwise rewrites header.seq during serialization. A relay must
  // preserve it as well as stamps, frames, payload and non-finite sensor values.
  options.has_header = false;
  const ros::Publisher publisher = nh.advertise(options);
  // A robot's EKF and state publisher send different TF children; retain a
  // small burst from both publishers instead of coalescing them as one sensor.
  return nh.subscribe<Message>(input, suffix == "tf" ? 10 : 1,
      [publisher](const boost::shared_ptr<const Message>& message) {
        publisher.publish(message);
      }, ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fleet_sensor_ingress");
  ros::NodeHandle nh, private_nh("~");
  bool per_robot_tf = false;
  private_nh.param("per_robot_tf", per_robot_tf, false);
  std::vector<std::string> robots;
  if (!private_nh.getParam("robots", robots) || robots.empty()) {
    ROS_FATAL("~robots must be a non-empty list of robot_N names");
    return 1;
  }
  std::set<std::string> unique;
  for (const auto& robot : robots) {
    if (robot.compare(0, 6, "robot_") != 0 || robot.size() <= 6 ||
        robot[6] == '0' || robot.find_first_not_of("0123456789", 6) != std::string::npos ||
        !unique.insert(robot).second) {
      ROS_FATAL_STREAM("invalid or duplicate robot name: " << robot);
      return 1;
    }
  }
  try {
    std::vector<ros::Subscriber> subscribers;
    for (const auto& robot : robots) {
      subscribers.push_back(relay<sensor_msgs::LaserScan>(nh, robot, "scan"));
      subscribers.push_back(relay<nav_msgs::Odometry>(nh, robot, "odom"));
      subscribers.push_back(relay<sensor_msgs::Imu>(nh, robot, "imu"));
      if (per_robot_tf)
        subscribers.push_back(relay<tf2_msgs::TFMessage>(nh, robot, "tf"));
    }
    // A single callback thread lets one large LaserScan delay every robot's
    // odom/IMU delivery.  Keep the relay stateless and let ROS dispatch the
    // independent vehicle streams concurrently.
    const int spinner_threads = 4;
    ROS_INFO("VM sensor ingress ready for %zu robots (scan, odom, imu; per_robot_tf=%s; spinner_threads=%d)",
             robots.size(), per_robot_tf ? "true" : "false", spinner_threads);
    ros::AsyncSpinner spinner(spinner_threads);
    spinner.start();
    ros::waitForShutdown();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM(error.what());
    return 1;
  }
  return 0;
}
