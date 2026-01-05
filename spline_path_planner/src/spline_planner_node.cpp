// spline_planner_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// --------- Structures ---------
struct SplineCoefficients {
  double t_start, t_end;
  double ax, bx, cx, dx;
  double ay, by, cy, dy;
  double az, bz, cz, dz;
};

struct Waypoint {
  double x, y, z;
};

// --------- Cubic Spline Class ---------
class CubicSpline {
public:
  CubicSpline() : n_(0) {}

  CubicSpline(const VectorXd &t, const VectorXd &y) : t_(t), n_(t.size() - 1) {
    VectorXd h(n_);
    for (int i = 0; i < n_; i++)
      h(i) = t_(i + 1) - t_(i);

    // Tridiagonal system
    MatrixXd A = MatrixXd::Zero(n_ + 1, n_ + 1);
    VectorXd b = VectorXd::Zero(n_ + 1);

    A(0,0) = 1.0; A(n_,n_) = 1.0;

    for (int i=1;i<n_;i++) {
      A(i,i-1)=h(i-1); A(i,i)=2*(h(i-1)+h(i)); A(i,i+1)=h(i);
      b(i) = 3*((y(i+1)-y(i))/h(i) - (y(i)-y(i-1))/h(i-1));
    }

    c_ = A.colPivHouseholderQr().solve(b);

    a_ = y.head(n_);
    b_ = VectorXd(n_);
    d_ = VectorXd(n_);

    for (int i=0;i<n_;i++) {
      b_(i) = (y(i+1)-y(i))/h(i) - h(i)*(2*c_(i)+c_(i+1))/3.0;
      d_(i) = (c_(i+1)-c_(i))/(3.0*h(i));
    }
  }

  int getNumSegments() const { return n_; }
  double getTStart(int i) const { return t_(i); }
  double getTEnd(int i) const { return t_(i+1); }
  double getA(int i) const { return a_(i); }
  double getB(int i) const { return b_(i); }
  double getC(int i) const { return c_(i); }
  double getD(int i) const { return d_(i); }

private:
  VectorXd t_, a_, b_, c_, d_;
  int n_;
};

// --------- Main Node ---------
class SplinePathPlanner : public rclcpp::Node {
public:
  SplinePathPlanner() : Node("spline_path_planner") {
    declare_parameter("waypoints_file", "waypoints.csv");
    waypoints_file_ = get_parameter("waypoints_file").as_string();

    // Publishers
    spline_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/spline_coefficients", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/spline_visualization", 10);

    if (!loadWaypoints()) {
      RCLCPP_FATAL(get_logger(), "Failed to load waypoints");
      return;
    }

    // Timer to publish spline & visualization at 1 Hz
    timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&SplinePathPlanner::generateAndPublishSpline, this)
    );

    RCLCPP_INFO(get_logger(), "Spline Path Planner running");
  }

private:
  // --------- Load CSV safely ---------
  bool loadWaypoints() {
    waypoints_.clear();
    std::ifstream file(waypoints_file_);
    if (!file.is_open()) {
      RCLCPP_ERROR(get_logger(), "Cannot open %s", waypoints_file_.c_str());
      return false;
    }

    std::string line;
    int line_number = 0;
    while (std::getline(file, line)) {
      line_number++;
      if (line.empty()) continue;

      std::stringstream ss(line);
      std::string token;
      std::vector<double> values;

      while (std::getline(ss, token, ',')) {
        try {
          token.erase(0, token.find_first_not_of(" \t\r\n"));
          token.erase(token.find_last_not_of(" \t\r\n")+1);
          values.push_back(std::stod(token));
        } catch (...) {
          values.clear();
          break; // skip bad line (header or invalid)
        }
      }

      if (values.size() >= 2) {
        waypoints_.push_back({values[0], values[1], values.size() >= 3 ? values[2] : 0.0});
      } else if (!values.empty()) {
        RCLCPP_WARN(get_logger(), "Skipping invalid line %d in %s",
                    line_number, waypoints_file_.c_str());
      }
    }

    RCLCPP_INFO(get_logger(), "Loaded %zu waypoints", waypoints_.size());
    return waypoints_.size() >= 2;
  }

  // --------- Generate spline coefficients ---------
  std::vector<SplineCoefficients> generateCubicSplineCoefficients() {
    int n = waypoints_.size();
    VectorXd s(n), x(n), y(n), z(n);

    s(0) = 0.0;
    for (int i=1;i<n;i++) {
      double dx=waypoints_[i].x-waypoints_[i-1].x;
      double dy=waypoints_[i].y-waypoints_[i-1].y;
      double dz=waypoints_[i].z-waypoints_[i-1].z;
      s(i)=s(i-1)+std::sqrt(dx*dx+dy*dy+dz*dz);
    }

    for (int i=0;i<n;i++) {
      x(i)=waypoints_[i].x; y(i)=waypoints_[i].y; z(i)=waypoints_[i].z;
    }

    CubicSpline sx(s,x), sy(s,y), sz(s,z);
    int num_segments = sx.getNumSegments();
    std::vector<SplineCoefficients> coeffs(num_segments);

    for (int i=0;i<num_segments;i++) {
      coeffs[i].t_start = sx.getTStart(i);
      coeffs[i].t_end = sx.getTEnd(i);
      coeffs[i].ax = sx.getA(i); coeffs[i].bx = sx.getB(i);
      coeffs[i].cx = sx.getC(i); coeffs[i].dx = sx.getD(i);
      coeffs[i].ay = sy.getA(i); coeffs[i].by = sy.getB(i);
      coeffs[i].cy = sy.getC(i); coeffs[i].dy = sy.getD(i);
      coeffs[i].az = sz.getA(i); coeffs[i].bz = sz.getB(i);
      coeffs[i].cz = sz.getC(i); coeffs[i].dz = sz.getD(i);
    }

    return coeffs;
  }

  // --------- Evaluate cubic polynomial ---------
  double evalCubic(double a, double b, double c, double d, double t) {
    return a + b*t + c*t*t + d*t*t*t;
  }

  // --------- Publish spline + visualization ---------
  void generateAndPublishSpline() {
    auto coefficients = generateCubicSplineCoefficients();

    // Publish Float64MultiArray
    std_msgs::msg::Float64MultiArray msg;
    int segments = coefficients.size();
    msg.layout.dim.resize(2);
    msg.layout.dim[0].label = "segments";
    msg.layout.dim[0].size = segments;
    msg.layout.dim[0].stride = segments*14;
    msg.layout.dim[1].label = "coefficients";
    msg.layout.dim[1].size = 14;
    msg.layout.dim[1].stride = 14;
    msg.data.reserve(segments*14);

    for (const auto &c : coefficients) {
      msg.data.insert(msg.data.end(), {
        c.t_start, c.t_end,
        c.ax, c.bx, c.cx, c.dx,
        c.ay, c.by, c.cy, c.dy,
        c.az, c.bz, c.cz, c.dz
      });
    }
    spline_pub_->publish(msg);

    // Publish visualization markers
    publishVisualization(coefficients);
  }

  // --------- Visualization in RViz ---------
  void publishVisualization(const std::vector<SplineCoefficients> &coeffs) {
    // Trajectory (LINE_STRIP)
    visualization_msgs::msg::Marker traj;
    traj.header.frame_id = "map";
    traj.header.stamp = now();
    traj.ns = "spline";
    traj.id = 0;
    traj.type = visualization_msgs::msg::Marker::LINE_STRIP;
    traj.action = visualization_msgs::msg::Marker::ADD;
    traj.scale.x = 0.05;
    traj.color.r=0.0; traj.color.g=1.0; traj.color.b=0.0; traj.color.a=1.0;
    traj.pose.orientation.w=1.0;

    const int samples_per_segment=20;
    for (const auto &c : coeffs) {
    double dt_seg = (c.t_end - c.t_start) / samples_per_segment;
    for (int i = 0; i <= samples_per_segment; i++) {
        double t_rel = i * dt_seg;   // relative t for this segment
        geometry_msgs::msg::Point p;
        p.x = evalCubic(c.ax, c.bx, c.cx, c.dx, t_rel);
        p.y = evalCubic(c.ay, c.by, c.cy, c.dy, t_rel);
        p.z = evalCubic(c.az, c.bz, c.cz, c.dz, t_rel);
        traj.points.push_back(p);
        }
    }


    // Waypoints (SPHERE_LIST)
    visualization_msgs::msg::Marker wps;
    wps.header.frame_id="map";
    wps.header.stamp = now();
    wps.ns="waypoints";
    wps.id=1;
    wps.type=visualization_msgs::msg::Marker::SPHERE_LIST;
    wps.action=visualization_msgs::msg::Marker::ADD;
    wps.scale.x = 0.15; wps.scale.y=0.15; wps.scale.z=0.15;
    wps.color.r=1.0; wps.color.g=0.0; wps.color.b=0.0; wps.color.a=1.0;
    wps.pose.orientation.w=1.0;

    for (const auto &wp : waypoints_) {
      geometry_msgs::msg::Point p;
      p.x = wp.x; p.y = wp.y; p.z = wp.z;
      wps.points.push_back(p);
    }

    marker_pub_->publish(traj);
    marker_pub_->publish(wps);
  }

  // ---------- Members ----------
  std::string waypoints_file_;
  std::vector<Waypoint> waypoints_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr spline_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// --------- Main ---------
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SplinePathPlanner>());
  rclcpp::shutdown();
  return 0;
}
