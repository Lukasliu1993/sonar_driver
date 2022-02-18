#include <sonar_interface_node.hpp>

SonarInterfaceNode::SonarInterfaceNode(const rclcpp::NodeOptions & node_options) 
: Node("sonar_interface", node_options)
{
  // read parameter for serial
  port_ = declare_parameter("port", "/dev/ttyUSB0");
  baud_ = declare_parameter("baud", 115200);
  x_num = declare_parameter("x_num", 2);// 前后方向的传感器数量
  y_num = declare_parameter("y_num", 2);// 左右方向的传感器数量
  rate = declare_parameter("rate", 10);
  smooth_ = declare_parameter("smooth", 4);// 这款传感器的数值稳定性还不够，面对特征较弱的障碍物时，大部分时间会返回真实测值，但偶尔还串入一些最大值（因为没有接收到回波），为了保险起见，传感器的每次测值取前smooth_次的最小值
  rclcpp::Rate loop_rate(rate);
  // set parameter for serial
  this->serial_.setPort(port_);
  this->serial_.setBaudrate(baud_);
  this->serial_.setTimeout(std::numeric_limits<uint32_t>::max(), 5000, 0, 5000, 0);
  this->serial_.open();

  // imu data publisher 
  publisher_x = create_publisher<std_msgs::msg::Float32>("front_back_warning", 1);// 前后方向的最小距离
  publisher_y = create_publisher<std_msgs::msg::Float32>("left_right_warning", 1);// 左右方向的最小距离

  
  vector<vector<float>> v_list;
  vector<float> v;
  v.push_back(5.675);
  for (int i = 0;i < x_num + y_num;i++){
    v_list.push_back(v);
  }

  uint8_t payload[3];
  while (rclcpp::ok()){
    float front_back_length = 5.675;
    for (int i = 0; i < x_num; i++){
      uint8_t recbuff[2];
      try{
        payload[0] = 224+i*2;
        payload[1] = 0x02;
        payload[2] = 0xb0;
        this->serial_.write(payload,sizeof(payload));
        this->serial_.read(recbuff,sizeof(recbuff));
        float sonar_range = ((recbuff[0] << 8)|(recbuff[1] & 0xff));
        sonar_range = sonar_range / 1000;
        if(v_list[i].size() == smooth_){
          v_list[i].erase(v_list[i].begin());
        }
        v_list[i].push_back(sonar_range);
        float sonar_range_ = *min_element(v_list[i].begin(), v_list[i].end());
        if (sonar_range_ < front_back_length){
          front_back_length = sonar_range_;
        }
      }
      catch(serial::IOException& e){
      }   
    }
    std_msgs::msg::Float32 warning_data1;
    warning_data1.data = front_back_length;
    publisher_x->publish(warning_data1);
    float left_right_length = 5.675;
    for (int i = x_num; i < x_num + y_num; i++){
      uint8_t recbuff[2];
      try{
        payload[0] = 224+i*2;
        payload[1] = 0x02;
        payload[2] = 0xb0;
        this->serial_.write(payload,sizeof(payload));
        this->serial_.read(recbuff,sizeof(recbuff));
        float sonar_range = ((recbuff[0] << 8)|(recbuff[1] & 0xff));
        sonar_range = sonar_range / 1000;
        if(v_list[i].size() == smooth_){
          v_list[i].erase(v_list[i].begin());
        }
        v_list[i].push_back(sonar_range);
        float sonar_range_ = *min_element(v_list[i].begin(), v_list[i].end());
        if (sonar_range_ < left_right_length){
          left_right_length = sonar_range_;
        }
      }
      catch(serial::IOException& e){
      }   
    } 
    std_msgs::msg::Float32 warning_data2;
    warning_data2.data = left_right_length;
    publisher_y->publish(warning_data2);  
    loop_rate.sleep();
  }
  this->serial_.close();
}


#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(SonarInterfaceNode)
