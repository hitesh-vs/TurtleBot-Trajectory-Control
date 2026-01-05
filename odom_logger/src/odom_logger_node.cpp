#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <fstream>
#include <iomanip>
#include <cmath>

class OdomLogger : public rclcpp::Node
{
public:
  OdomLogger() : Node("odom_logger")
  {
    declare_parameter<std::string>("csv_path", "odom_log.csv");
    declare_parameter<bool>("write_header", true);

    csv_path_ = get_parameter("csv_path").as_string();
    write_header_ = get_parameter("write_header").as_bool();

    file_.open(csv_path_, std::ios::out);
    if (!file_.is_open())
    {
      RCLCPP_FATAL(get_logger(), "Failed to open CSV file: %s", csv_path_.c_str());
      rclcpp::shutdown();
      return;
    }

    if (write_header_)
    {
      file_ << "time_sec,x,y,yaw\n";
      file_.flush();
    }

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 50,
      std::bind(&OdomLogger::odomCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(), "Logging odometry to: %s", csv_path_.c_str());
  }

  ~OdomLogger()
  {
    if (file_.is_open())
    {
      file_.close();
      RCLCPP_INFO(get_logger(), "Odometry log saved.");
    }
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    double time_sec =
      msg->header.stamp.sec +
      msg->header.stamp.nanosec * 1e-9;

    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double yaw = quaternionToYaw(msg->pose.pose.orientation);

    file_ << std::fixed << std::setprecision(6)
          << time_sec << ","
          << x << ","
          << y << ","
          << yaw << "\n";

    file_.flush();  // ensure data is written even if node crashes
  }

  static double quaternionToYaw(const geometry_msgs::msg::Quaternion &q)
  {
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  std::string csv_path_;
  bool write_header_;

  std::ofstream file_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomLogger>());
  rclcpp::shutdown();
  return 0;
}
