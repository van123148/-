// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
// W
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <bits/stdc++.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <cv_bridge/cv_bridge.h>

using std::placeholders::_1;

class ArmorDetectorNode : public rclcpp::Node
{
public:
  ArmorDetectorNode()
      : Node("minimal_subscriber")
  {
    pub_ = this->create_publisher<sensor_msgs::msg::Image>("image", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw", rclcpp::SensorDataQoS(),
        std::bind(&ArmorDetectorNode::imageCallback, this, std::placeholders::_1));
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
  {
    RCLCPP_INFO(this->get_logger(), "s");
    auto img = cv_bridge::toCvShare(img_msg, "rgb8")->image;
    now_time = clock();
    double fps = 1.0 / ((now_time - lst_time) / CLOCKS_PER_SEC);
    lst_time = now_time;
    std::string str = std::to_string(fps);
    putText(img, str, cv::Point(30, 30), cv::FONT_HERSHEY_COMPLEX, 1.0, cv::Scalar(12, 23, 200), 3, 8);
    imwrite("a.jpg",img);
    sensor_msgs::msg::Image msg_img;
    cv_bridge::CvImage cv_img;
    cv_img.encoding = "rgb8";
    cv_img.header.stamp = this->now();
    cv_img.image = img;
    cv_img.toImageMsg(msg_img);
    pub_->publish(msg_img);
  }
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  double now_time, lst_time;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmorDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
