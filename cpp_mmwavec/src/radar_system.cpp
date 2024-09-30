#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "mmwave_class.cpp"
// #include "cpp_mmwavec/msg/DataFrame.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher(mmWave_Sensor* sensor): Node("minimal_publisher"), count_(0), sensor(sensor){
      this->sensor = sensor;
      publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>("radar_data", 10); // ROS2 topic is "radar_data"
      timer_ = this->create_wall_timer(
      1ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
        std_msgs::msg::Int16MultiArray pub_msg = std_msgs::msg::Int16MultiArray();
        if (this->sensor->is_capture_started()) {
          uint64_t size = this->sensor->data_array_ptr->frame_size;
          pub_msg.data.resize(size);
          for (uint64_t i = 0; i < size; ++i) {
              pub_msg.data[i] = sensor->data_array_ptr->data[i]; // Publish the data from the ring buffer to the ROS 2 topic "radar_data"
          }        
        }
          publisher_->publish(pub_msg);
      
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
    size_t count_;
    mmWave_Sensor* sensor;
};

void collect_data_thread_fn(mmWave_Sensor &sensor){
  while (true){
    if (sensor.is_capture_started()==1){
      sensor.collect_data();
    }
  }
}

int main(int argc, char * argv[])
{
  // Initialize a mmWave_Sensor object with the config name as an argument
  mmWave_Sensor my_radar=mmWave_Sensor(argv[1]);

  // Set capture to off as a precaution
  my_radar.toggle_capture(0);
  // Set up and configure the DCA1000EVM and the IWR1443BOOST
  int setup = my_radar.setupDCA_and_cfgIWR();
  // Check if successful
  if (setup==-1) return 0;

  // Arm the DCA
  my_radar.arm_dca();
  std::this_thread::sleep_for(std::chrono::milliseconds(2000)); 

  // Start the data collection thread
  std::thread collect(collect_data_thread_fn, std::ref(my_radar));

  // Create ROS2 node
  rclcpp::init(argc, argv);
  // Start capture
  my_radar.toggle_capture(1);
  // Spin (run) the ROS2 node for the radar
  rclcpp::spin(std::make_shared<MinimalPublisher>(&my_radar));

  // If process is aborted (i.e. Ctrl+C), turn radar off and close socket and serial communication, shut down ROS2
  my_radar.toggle_capture(0);
  my_radar.close_comms();
  rclcpp::shutdown();
  
  return 0;
}