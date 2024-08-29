#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <chrono> /*new*/

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <cv_bridge/cv_bridge.h>
using namespace std::chrono_literals;

class SubImg : public rclcpp::Node {
  private:
    cv::Mat image;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSubscription;
    rclcpp::TimerBase::SharedPtr timer_; /*new*/
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_; /*new*/
    const std::string packageName = "custom_package";
    const std::string imageFileRelative = "/image/techman_robot.jpg";
    bool isShowPic;
    void get_new_image_callback(sensor_msgs::msg::Image::SharedPtr msg);
    void show_image();
    void timer_callback(); /*new*/

  public:
    static int encoding_to_mat_type(const std::string & encoding);
    SubImg();
};

int SubImg::encoding_to_mat_type(const std::string & encoding){
  if (encoding == "mono8") {
    return CV_8UC1;
  } else if (encoding == "bgr8") {
    return CV_8UC3;
  } else if (encoding == "mono16") {
    return CV_16SC1;
  } else if (encoding == "rgba8") {
    return CV_8UC4;
  } else if (encoding == "bgra8") {
    return CV_8UC4;
  } else if (encoding == "32FC1") {
    return CV_32FC1;
  } else if (encoding == "rgb8") {
    return CV_8UC3;
  }else if (encoding =="8UC3"){
    return CV_8UC3;
  }
  else {
    std::cout<<"the unknow image type is "<<encoding<<std::endl;
    throw std::runtime_error("Unsupported encoding type");
  }
}

void SubImg::get_new_image_callback(sensor_msgs::msg::Image::SharedPtr msg){
  try{

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); // or the appropriate encoding
    
    cv::Mat frame = cv_ptr->image;
    /*
    cv::Mat frame(msg->height, msg->width, SubImg::encoding_to_mat_type(msg->encoding),
      const_cast<unsigned char *>(msg->data.data()), msg->step);
    if (msg->encoding == "rgb8") {
      cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
    }
    */
    std::cout << "Width : " << frame.size().width << std::endl;
    std::cout << "Height: " << frame.size().height << std::endl;
    frame.copyTo(this->image);
    std::cout<<"after set this->image = frame";
  }
  catch(std::runtime_error &exception){
    std::cout<<"there is a exception "<< exception.what()<< std::endl;
  }
}

void SubImg::show_image(){
  while(true){
    cv::imshow("showimage",this->image );
    cv::waitKey(30);


  }
}
/*new*/
void SubImg::timer_callback(){
    printf("1");
    /*
    auto message = sensor_msgs::msg::Image();
    message.data = this->image;
    RCLCPP_INFO(this->get_logger(), "Publishing"); 
    publisher_->publish(message);
    */
    sensor_msgs::msg::Image::SharedPtr message = std::make_shared<sensor_msgs::msg::Image>();
    message->header.stamp = this->get_clock()->now();
    message->header.frame_id = "frame_id";
    message->height = this->image.rows;
    message->width = this->image.cols;
    message->encoding = "bgr8"; // Set the appropriate encoding
    message->is_bigendian = 0;
    message->step = this->image.step;

    // Copy image data
    message->data.resize(this->image.total() * this->image.elemSize());
    std::memcpy(message->data.data(), this->image.data, message->data.size());

    RCLCPP_INFO(this->get_logger(), "Publishing");
    publisher_->publish(*message);
}
/*new*/

SubImg::SubImg() : Node("test_image_sub"){
  //image = cv::Mat();
  auto position = ament_index_cpp::get_package_share_directory(packageName);
  std::string fullIniImagePath = position + imageFileRelative;
  this->image = cv::imread(fullIniImagePath);
  isShowPic = true;
  
  imageSubscription = this->create_subscription<sensor_msgs::msg::Image>(
  "techman_image", 10, std::bind(&SubImg::get_new_image_callback, this, std::placeholders::_1));
  
  publisher_ = this->create_publisher<sensor_msgs::msg::Image>("imgtopic", 10); /*new*/

  timer_ = this->create_wall_timer( 1s, std::bind(&SubImg::timer_callback, this));

  std::thread(&SubImg::show_image, this).detach();
  //std::thread(&SubImg::timer_callback, this).detach(); /*new*/
}

int main(int argc, char *argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubImg>());
  std::cout<<"end spin"<<std::endl;
  rclcpp::shutdown();

  return 0;
}
