#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <deque>

using namespace std::chrono_literals;

class CoordinatesSubscriber : public rclcpp::Node
{
public:
  CoordinatesSubscriber()
      : Node("coordinates_subscriber")
  {
    // 创建订阅者，只订阅 geometry_msgs/msg/PointStamped 类型的消息
    subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "target_point", 10,
        [this](const geometry_msgs::msg::PointStamped::SharedPtr msg)
        {
          this->callback(msg);
        });

    // 创建定时器，每秒处理一次队列
    timer_ = this->create_wall_timer(
        1s, [this]()
        { this->process_queue(); });
  }

private:
  void callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Callback function called");
    RCLCPP_INFO(this->get_logger(), "Received PointStamped message with x: %f, y: %f, z: %f",
                msg->point.x, msg->point.y, msg->point.z);

    // 将接收到的消息存储到队列中
    if (queue_.size() >= 10) // 如果队列已满
    {
      queue_.pop_front(); // 移除最旧的元素
    }
    queue_.push_back(*msg); // 添加新消息
  }

  void process_queue()
  {
    // 示例处理：打印队列中的元素个数
    RCLCPP_INFO(this->get_logger(), "Queue size: %zu", queue_.size());

    // 处理队列中的数据（可根据需求添加更多处理代码）
    for (const auto& msg : queue_)
    {
      RCLCPP_INFO(this->get_logger(), "Queue element x: %f, y: %f, z: %f",
                  msg.point.x, msg.point.y, msg.point.z);
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::deque<geometry_msgs::msg::PointStamped> queue_; // 用于存储坐标的双端队列
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CoordinatesSubscriber>());
  rclcpp::shutdown();
  return 0;
}

