#include "rclcpp/rclcpp.hpp"
#include "nav2_core/controller.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace
{
// Observation layout (554): [lidar 6x90][temporal 8][goal_dist 1][goal_angle 1][vel 2][prev_act 2]
constexpr int N_RAYS = 90;
constexpr int N_FRAMES = 6;
constexpr int N_LIDAR = N_RAYS * N_FRAMES;   // 540
constexpr int N_SECTORS = 8;
constexpr int OBS_DIM = 554;

constexpr float LIDAR_CAP = 3.5f;            // training lidar normalization range (m)
constexpr float TEMPORAL_SCALE = 77.8f;      // raw sector diff -> velocity units
constexpr float TEMPORAL_EMA = 0.8f;         // smoothing on the sector diff
constexpr float TEMPORAL_OUT = 0.05f;        // output normalization for the sector diff

constexpr double MAX_LIN = 0.22;             // ict_bot max linear (m/s)
constexpr double MAX_ANG = 2.0;              // ict_bot max angular (rad/s)
constexpr double GOAL_NORM = 7.0710678;      // sqrt(5^2 + 5^2), training goal-dist scale
constexpr double TURN_SLOWDOWN = 0.6;        // cut forward speed up to 60% in hard turns -> sharper radius
constexpr double MIN_SPEED_FRAC = 0.3;       // never drop below 30% of the commanded forward speed
constexpr bool DIRECT_GOAL_MODE = false;     // true = aim straight at the final goal (raw-policy test, no path-following)

// Angular offset (rad) applied when sampling /scan. 0 assumes scan angle 0 aligns
// with the training lidar's ray 0. Nudge by +/- pi/2 if dodging looks rotated.
constexpr float LIDAR_OFFSET = 0.0f;

double yawFromQuat(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double wrapAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}
}  // namespace

class RLController : public nav2_core::Controller
{
public:
  RLController() = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer>,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override
  {
    node_ = parent.lock();
    logger_ = node_->get_logger();

    // carrot distance ahead on the path (m); tune in nav2_params.yaml or via `ros2 param set`
    lookahead_param_ = name + ".carrot_lookahead";
    if (!node_->has_parameter(lookahead_param_)) {
      node_->declare_parameter(lookahead_param_, carrot_lookahead_);
    }

    env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "rl");
    session_options_ = std::make_unique<Ort::SessionOptions>();
    session_options_->SetIntraOpNumThreads(1);

    std::string model_path =
      "/home/user/Documents/ros2_ws_ict_nav2/src/rl_nav_cpp/td3_final.onnx";
    session_ = std::make_unique<Ort::Session>(*env_, model_path.c_str(), *session_options_);

    scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(scan_mtx_);
        last_scan_ = msg;
      });

    RCLCPP_INFO(logger_, "RL Controller configured (TD3 ConvActor, 554-dim obs)");
  }

  void cleanup() override {}
  void activate() override {}
  void deactivate() override {}
  void setSpeedLimit(const double &, const bool &) override {}

  void setPlan(const nav_msgs::msg::Path & path) override
  {
    current_path_ = path;
    carrot_idx_ = 0;
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker *) override
  {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.frame_id = "base_footprint";
    cmd.header.stamp = node_->now();

    if (current_path_.poses.empty()) {
      return cmd;  // no plan yet -> stop
    }

    carrot_lookahead_ = node_->get_parameter(lookahead_param_).as_double();

    // 1. lidar: resample /scan to 90 rays, then update frame stack + temporal diff
    std::vector<float> scan = sampleScan();
    updateLidarBuffers(scan);

    // 2. carrot: a point a short distance ahead on the path, fed as the goal
    double cx, cy;
    findCarrot(pose, cx, cy);

    // 3. goal features in the robot's base_footprint frame (+X forward)
    double rx = pose.pose.position.x;
    double ry = pose.pose.position.y;
    double ryaw = yawFromQuat(pose.pose.orientation);
    double dx = cx - rx;
    double dy = cy - ry;
    float goal_dist = static_cast<float>(std::clamp(std::hypot(dx, dy) / GOAL_NORM, 0.0, 1.0));
    float goal_angle = static_cast<float>(wrapAngle(std::atan2(dy, dx) - ryaw) / M_PI);

    // 4. velocity (body frame, normalized)
    float lin_n = static_cast<float>(std::clamp(velocity.linear.x / MAX_LIN, -1.0, 1.0));
    float ang_n = static_cast<float>(std::clamp(velocity.angular.z / MAX_ANG, -1.0, 1.0));

    // 5. assemble the 554-dim observation
    std::array<float, OBS_DIM> obs{};
    int o = 0;
    for (int f = 0; f < N_FRAMES; ++f) {
      for (int k = 0; k < N_RAYS; ++k) obs[o++] = frames_[f][k];
    }
    for (int s = 0; s < N_SECTORS; ++s) {
      obs[o++] = std::clamp(temporal_[s] / TEMPORAL_OUT, -1.0f, 1.0f);
    }
    obs[o++] = goal_dist;
    obs[o++] = goal_angle;
    obs[o++] = lin_n;
    obs[o++] = ang_n;
    obs[o++] = last_action_[0];
    obs[o++] = last_action_[1];

    // 6. inference
    std::array<int64_t, 2> shape{1, OBS_DIM};
    Ort::MemoryInfo mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value input = Ort::Value::CreateTensor<float>(
      mem, obs.data(), obs.size(), shape.data(), shape.size());

    const char * in_names[] = {"input"};
    const char * out_names[] = {"output"};
    auto outputs = session_->Run(Ort::RunOptions{nullptr}, in_names, &input, 1, out_names, 1);
    float * action = outputs[0].GetTensorMutableData<float>();

    last_action_[0] = std::clamp(action[0], -1.0f, 1.0f);
    last_action_[1] = std::clamp(action[1], -1.0f, 1.0f);

    // 7. map to cmd_vel (forward-only linear, same as training action term)
    double linear = (last_action_[0] + 1.0) * 0.5 * MAX_LIN;
    double angular = last_action_[1] * MAX_ANG;

    // slow down when the policy commands a hard turn -> tighter radius (sharp corners)
    double turn_scale = 1.0 - TURN_SLOWDOWN * std::abs(last_action_[1]);
    linear *= std::max(turn_scale, MIN_SPEED_FRAC);

    cmd.twist.linear.x = linear;
    cmd.twist.angular.z = angular;

    RCLCPP_INFO(logger_,
      "act[%.2f %.2f] goal d=%.2f a=%.2f -> lin=%.3f ang=%.3f",
      last_action_[0], last_action_[1], goal_dist, goal_angle, linear, angular);

    return cmd;
  }

private:
  // Resample the latest /scan into 90 rays at the training lidar's angles.
  std::vector<float> sampleScan()
  {
    std::vector<float> out(N_RAYS, 1.0f);  // default = far / no hit

    sensor_msgs::msg::LaserScan::SharedPtr s;
    {
      std::lock_guard<std::mutex> lock(scan_mtx_);
      s = last_scan_;
    }
    if (!s || s->ranges.empty() || s->angle_increment == 0.0f) return out;

    int n = static_cast<int>(s->ranges.size());
    for (int i = 0; i < N_RAYS; ++i) {
      float ang = 2.0f * static_cast<float>(M_PI) * i / N_RAYS + LIDAR_OFFSET;
      ang = std::atan2(std::sin(ang), std::cos(ang));            // wrap to [-pi, pi]
      int idx = static_cast<int>(std::lround((ang - s->angle_min) / s->angle_increment));
      idx = ((idx % n) + n) % n;                                 // wrap (full-circle scan)
      float r = s->ranges[idx];
      out[i] = (!std::isfinite(r) || r >= LIDAR_CAP)
               ? 1.0f
               : std::clamp(r / LIDAR_CAP, 0.0f, 1.0f);
    }
    return out;
  }

  // Push the new scan into the 6-frame stack and update the 8-sector temporal EMA.
  void updateLidarBuffers(const std::vector<float> & scan)
  {
    if (!buffers_ready_) {
      for (int f = 0; f < N_FRAMES; ++f) frames_[f] = scan;
      prev_scan_ = scan;
      temporal_.assign(N_SECTORS, 0.0f);
      buffers_ready_ = true;
      return;
    }

    for (int f = N_FRAMES - 1; f >= 1; --f) frames_[f] = frames_[f - 1];
    frames_[0] = scan;

    // per-sector min of the per-ray range diff, scaled, then EMA-smoothed
    int base = N_RAYS / N_SECTORS;   // 11
    int rem = N_RAYS % N_SECTORS;    // 2
    int start = 0;
    for (int s = 0; s < N_SECTORS; ++s) {
      int len = base + (s < rem ? 1 : 0);
      float mn = std::numeric_limits<float>::infinity();
      for (int k = start; k < start + len; ++k) {
        mn = std::min(mn, scan[k] - prev_scan_[k]);
      }
      float scaled = mn * TEMPORAL_SCALE;
      temporal_[s] = TEMPORAL_EMA * temporal_[s] + (1.0f - TEMPORAL_EMA) * scaled;
      start += len;
    }
    prev_scan_ = scan;
  }

  // Advance along the path and return a point ~CARROT_LOOKAHEAD ahead of the robot.
  void findCarrot(const geometry_msgs::msg::PoseStamped & pose, double & cx, double & cy)
  {
    const auto & poses = current_path_.poses;

    // direct-goal test mode: ignore the path shape, aim straight at the final goal
    if (DIRECT_GOAL_MODE) {
      cx = poses.back().pose.position.x;
      cy = poses.back().pose.position.y;
      return;
    }

    double rx = pose.pose.position.x;
    double ry = pose.pose.position.y;

    // nearest path index, never moving backward
    double best = std::numeric_limits<double>::max();
    for (size_t i = carrot_idx_; i < poses.size(); ++i) {
      double d = std::hypot(poses[i].pose.position.x - rx, poses[i].pose.position.y - ry);
      if (d < best) {
        best = d;
        carrot_idx_ = i;
      }
    }

    // walk forward until we accumulate the lookahead distance
    size_t j = carrot_idx_;
    double acc = 0.0;
    while (j + 1 < poses.size() && acc < carrot_lookahead_) {
      acc += std::hypot(
        poses[j + 1].pose.position.x - poses[j].pose.position.x,
        poses[j + 1].pose.position.y - poses[j].pose.position.y);
      ++j;
    }
    cx = poses[j].pose.position.x;
    cy = poses[j].pose.position.y;
  }

  rclcpp::Logger logger_{rclcpp::get_logger("rl_controller")};
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
  std::mutex scan_mtx_;

  nav_msgs::msg::Path current_path_;
  size_t carrot_idx_{0};
  double carrot_lookahead_{0.3};
  std::string lookahead_param_;

  std::array<std::vector<float>, N_FRAMES> frames_;
  std::vector<float> prev_scan_;
  std::vector<float> temporal_;
  bool buffers_ready_{false};

  float last_action_[2]{0.0f, 0.0f};

  std::unique_ptr<Ort::Env> env_;
  std::unique_ptr<Ort::Session> session_;
  std::unique_ptr<Ort::SessionOptions> session_options_;
};

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(RLController, nav2_core::Controller)
