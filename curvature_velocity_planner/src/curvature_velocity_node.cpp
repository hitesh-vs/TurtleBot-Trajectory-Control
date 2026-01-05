// #include <rclcpp/rclcpp.hpp>
// #include <std_msgs/msg/float64_multi_array.hpp>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <vector>
// #include <cmath>

// struct PoseVel {
//     double x, y, v;
// };

// class CurvatureVelocityPlanner : public rclcpp::Node {
// public:
//     CurvatureVelocityPlanner() : Node("curvature_velocity_planner") {
//         this->declare_parameter<int>("samples_per_segment", 20);
//         this->declare_parameter<double>("v_max", 0.5);
//         this->declare_parameter<double>("curvature_factor", 0.5);

//         samples_per_segment_ = this->get_parameter("samples_per_segment").as_int();
//         v_max_ = this->get_parameter("v_max").as_double();
//         curvature_factor_ = this->get_parameter("curvature_factor").as_double();

//         spline_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
//             "/spline_coefficients", 10,
//             std::bind(&CurvatureVelocityPlanner::splineCallback, this, std::placeholders::_1)
//         );

//         waypoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
//             "/time_stamped_waypoints", 10
//         );

//         RCLCPP_INFO(this->get_logger(), "Curvature Velocity Planner node initialized");
//     }

// private:
//     void splineCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
//         const auto& data = msg->data;
//         int num_segments = data.size() / 8;
//         std::vector<PoseVel> points;

//         // Sample spline
//         for (int i = 0; i < num_segments; i++) {
//             double t_start = data[i*8 + 0];
//             double t_end   = data[i*8 + 1];
//             double ax = data[i*8 + 2];
//             double bx = data[i*8 + 3];
//             double cx = data[i*8 + 4];
//             double dx = data[i*8 + 5];
//             double ay = data[i*8 + 6];
//             double by = data[i*8 + 7];
//             double cy = data[i*8 + 8];
//             double dy = data[i*8 + 9];

//             double dt_seg = (t_end - t_start) / samples_per_segment_;

//             for (int j = 0; j <= samples_per_segment_; j++) {
//                 double t = j * dt_seg;

//                 // Position
//                 double x = ax + bx*t + cx*t*t + dx*t*t*t;
//                 double y = ay + by*t + cy*t*t + dy*t*t*t;

//                 // Derivatives
//                 double dx_dt = bx + 2*cx*t + 3*dx*t*t;
//                 double dy_dt = by + 2*cy*t + 3*dy*t*t;
//                 double ddx_dt = 2*cx + 6*dx*t;
//                 double ddy_dt = 2*cy + 6*dy*t;

//                 // Curvature
//                 double kappa = std::abs(dx_dt*ddy_dt - dy_dt*ddx_dt) /
//                                std::pow(dx_dt*dx_dt + dy_dt*dy_dt, 1.5);
//                 if (kappa < 1e-5) kappa = 1e-5;

//                 // Velocity assignment
//                 double v = std::min(v_max_, curvature_factor_ / kappa);

//                 points.push_back({x, y, v});
//             }
//         }

//         // Publish waypoints with velocity in z
//         for (size_t i = 0; i < points.size(); i++) {
//             geometry_msgs::msg::PoseStamped pose;
//             pose.header.stamp = this->get_clock()->now();
//             pose.header.frame_id = "map";
//             pose.pose.position.x = points[i].x;
//             pose.pose.position.y = points[i].y;
//             pose.pose.position.z = points[i].v; // <-- velocity here

//             waypoint_pub_->publish(pose);
//         }

//         RCLCPP_INFO(this->get_logger(), "Published %zu waypoints with velocity", points.size());
//     }

//     int samples_per_segment_;
//     double v_max_;
//     double curvature_factor_;

//     rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr spline_sub_;
//     rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_pub_;
// };

// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<CurvatureVelocityPlanner>());
//     rclcpp::shutdown();
//     return 0;
// }
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <fstream>
#include <vector>
#include <cmath>

struct PoseVel {
    double x, y, v;
};

class CurvatureVelocityCSV : public rclcpp::Node {
public:
    CurvatureVelocityCSV() : Node("curvature_velocity_csv") {
        this->declare_parameter<int>("samples_per_segment", 20);
        this->declare_parameter<double>("v_max", 0.5);
        this->declare_parameter<double>("curvature_factor", 0.5);
        this->declare_parameter<std::string>("output_file", "waypoints_time.csv");

        samples_per_segment_ = this->get_parameter("samples_per_segment").as_int();
        v_max_ = this->get_parameter("v_max").as_double();
        curvature_factor_ = this->get_parameter("curvature_factor").as_double();
        output_file_ = this->get_parameter("output_file").as_string();

        spline_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/spline_coefficients", 10,
            std::bind(&CurvatureVelocityCSV::splineCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Curvature CSV node ready");
    }

private:
    void splineCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    const auto& data = msg->data;
    int num_segments = data.size() / 10;
    std::vector<PoseVel> points;

    for (int i = 0; i < num_segments; i++) {
        double t_start = data[i*10 + 0];
        double t_end   = data[i*10 + 1];
        double ax = data[i*10 + 2];
        double bx = data[i*10 + 3];
        double cx = data[i*10 + 4];
        double dx = data[i*10 + 5];
        double ay = data[i*10 + 6];
        double by = data[i*10 + 7];
        double cy = data[i*10 + 8];
        double dy = data[i*10 + 9];

        double segment_duration = t_end - t_start;
        
        // Calculate velocity ONCE per segment based on maximum curvature
        double max_kappa = 0.0;
        int curvature_samples = 20;
        
        for (int k = 0; k <= curvature_samples; k++) {
            double t_local = k * segment_duration / curvature_samples;
            
            double dx_dt = bx + 2*cx*t_local + 3*dx*t_local*t_local;
            double dy_dt = by + 2*cy*t_local + 3*dy*t_local*t_local;
            double ddx_dt = 2*cx + 6*dx*t_local;
            double ddy_dt = 2*cy + 6*dy*t_local;
            
            double numerator = std::abs(dx_dt*ddy_dt - dy_dt*ddx_dt);
            double denominator = std::pow(dx_dt*dx_dt + dy_dt*dy_dt, 1.5);
            double kappa = (denominator > 1e-9) ? numerator / denominator : 1e-5;
            
            max_kappa = std::max(max_kappa, kappa);
        }
        
        if (max_kappa < 1e-5) max_kappa = 1e-5;
        
        // Constant velocity for entire segment based on maximum curvature
        double segment_velocity = std::min(v_max_, curvature_factor_ / max_kappa);
        
        // Now sample points with this constant velocity
        double dt_seg = segment_duration / samples_per_segment_;

        for (int j = 0; j <= samples_per_segment_; j++) {
            double t_local = j * dt_seg;

            // Position
            double x = ax + bx*t_local + cx*t_local*t_local + dx*t_local*t_local*t_local;
            double y = ay + by*t_local + cy*t_local*t_local + dy*t_local*t_local*t_local;

            points.push_back({x, y, segment_velocity});  // Same velocity for all points in segment
        }
    }

    // Write CSV
    std::ofstream ofs(output_file_);
    ofs << "x,y,v,t\n";
    
    double cumulative_time = 0.0;
    ofs << points[0].x << "," << points[0].y << "," << points[0].v << ",0.0\n";
    
    for (size_t i = 1; i < points.size(); i++) {
        double dx = points[i].x - points[i-1].x;
        double dy = points[i].y - points[i-1].y;
        double ds = std::sqrt(dx*dx + dy*dy);
        double dt = (points[i].v > 1e-6) ? ds / points[i].v : 0.0;
        cumulative_time += dt;
        ofs << points[i].x << "," << points[i].y << "," << points[i].v << "," << cumulative_time << "\n";
    }
    ofs.close();
    RCLCPP_INFO(this->get_logger(), "Saved %zu waypoints to %s", points.size(), output_file_.c_str());
}

    int samples_per_segment_;
    double v_max_;
    double curvature_factor_;
    std::string output_file_;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr spline_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CurvatureVelocityCSV>());
    rclcpp::shutdown();
    return 0;
}
