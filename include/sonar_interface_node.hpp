#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <std_msgs/msg/float32.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <algorithm>
#include <vector>
using namespace std;


class SonarInterfaceNode : public rclcpp::Node
{
public:



  /**
   * @brief constructor
   */
  explicit SonarInterfaceNode(const rclcpp::NodeOptions & options);

private:
  // Set up ROS.
  string port_;
  int baud_, x_num, y_num, smooth_, rate;
  serial::Serial serial_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_x, publisher_y;

};



