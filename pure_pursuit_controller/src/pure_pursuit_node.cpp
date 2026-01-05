#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>

struct Waypoint
{
  double x;
  double y;
  double v;
  double t;
};

class PurePursuitController : public rclcpp::Node
{
public:
  PurePursuitController()
  : Node("pure_pursuit_controller"), current_index_(0)
  {
    declare_parameter<std::string>("csv_path", "waypoints_time.csv");
    declare_parameter<double>("lookahead_distance", 1.0);
    declare_parameter<double>("max_angular_velocity", 1.5);
    declare_parameter<double>("goal_tolerance", 0.3);

    csv_path_ = get_parameter("csv_path").as_string();
    lookahead_distance_ = get_parameter("lookahead_distance").as_double();
    max_angular_velocity_ = get_parameter("max_angular_velocity").as_double();
    goal_tolerance_ = get_parameter("goal_tolerance").as_double();

    load_waypoints(csv_path_);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&PurePursuitController::odomCallback, this, std::placeholders::_1)
    );

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    RCLCPP_INFO(get_logger(), "Loaded %lu waypoints", waypoints_.size());
    RCLCPP_INFO(get_logger(), "Pure Pursuit Controller started");
  }

private:
  // ------------------------------------------------------------
  void load_waypoints(const std::string &path) {
    std::ifstream file(path);
    if (!file.is_open())
    {
        RCLCPP_ERROR(get_logger(), "Failed to open CSV file: %s", path.c_str());
        return;
    }

    std::string line;
    int line_num = 0;

    while (std::getline(file, line))
    {
        line_num++;

        // Skip empty lines
        if (line.empty())
        continue;

        std::stringstream ss(line);
        std::string cell;
        std::vector<std::string> tokens;

        while (std::getline(ss, cell, ','))
        tokens.push_back(cell);

        // Expect exactly 4 columns
        if (tokens.size() != 4)
        {
        RCLCPP_WARN(
            get_logger(),
            "Skipping malformed line %d: %s",
            line_num, line.c_str()
        );
        continue;
        }

        try
        {
        Waypoint wp;
        wp.x = std::stod(tokens[0]);
        wp.y = std::stod(tokens[1]);
        wp.v = std::stod(tokens[2]);
        wp.t = std::stod(tokens[3]);
        waypoints_.push_back(wp);
        }
        catch (const std::exception &e)
        {
        RCLCPP_WARN(
            get_logger(),
            "Skipping non-numeric line %d: %s",
            line_num, line.c_str()
        );
        continue;
        }
    }

    if (waypoints_.empty())
    {
        RCLCPP_ERROR(get_logger(), "No valid waypoints loaded!");
    }
    }


  // ------------------------------------------------------------
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (current_index_ >= waypoints_.size())
    {
      stopRobot();
      return;
    }

    // --- Robot pose ---
    const auto &p = msg->pose.pose.position;
    double px = p.x;
    double py = p.y;

    double yaw = quaternionToYaw(msg->pose.pose.orientation);

    // --- Lookahead point ---
    auto target = findLookaheadPoint(px, py);
    if (!target.has_value())
    {
      stopRobot();
      return;
    }

    Waypoint wp = target.value();

    // --- Transform to robot frame ---
    double dx = wp.x - px;
    double dy = wp.y - py;

    double x_r =  std::cos(-yaw) * dx - std::sin(-yaw) * dy;
    double y_r =  std::sin(-yaw) * dx + std::cos(-yaw) * dy;

    // --- Pure Pursuit control ---
    double curvature = (2.0 * y_r) / (lookahead_distance_ * lookahead_distance_);
    double omega = wp.v * curvature;

    // Clamp angular velocity
    omega = std::clamp(omega, -max_angular_velocity_, max_angular_velocity_);

    // --- Publish command ---
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = wp.v;
    cmd.angular.z = omega;
    cmd_pub_->publish(cmd);
  }

  // ------------------------------------------------------------
  std::optional<Waypoint> findLookaheadPoint(double px, double py)
  {
    while (current_index_ < waypoints_.size())
    {
      const auto &wp = waypoints_[current_index_];
      double dist = std::hypot(wp.x - px, wp.y - py);

      if (dist >= lookahead_distance_)
        return wp;

      current_index_++;
    }
    return std::nullopt;
  }

  // ------------------------------------------------------------
  void stopRobot()
  {
    geometry_msgs::msg::Twist cmd;
    cmd_pub_->publish(cmd);
  }

  // ------------------------------------------------------------
  static double quaternionToYaw(const geometry_msgs::msg::Quaternion &q)
  {
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  // ---------------- MEMBERS ----------------
  std::string csv_path_;
  double lookahead_distance_;
  double max_angular_velocity_;
  double goal_tolerance_;

  size_t current_index_;
  std::vector<Waypoint> waypoints_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuitController>());
  rclcpp::shutdown();
  return 0;
}
