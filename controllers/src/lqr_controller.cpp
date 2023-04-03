// Copyright 2021 RoboJackets
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <angles/angles.h>
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <string>
#include <algorithm>
#include <nav2_core/controller.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include "controller_helpers.h"

namespace controllers
{

class LqrController : public nav2_core::Controller
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & node,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override
  {
    node_ = node;

    auto node_shared = node_.lock();
    if (!node_shared) {
      throw std::runtime_error{"Could not acquire node."};
    }

    traj_viz_pub_ = node_shared->create_publisher<nav_msgs::msg::Path>(
      "~/tracking_traj",
      rclcpp::SystemDefaultsQoS());

    // BEGIN STUDENT CODE
    T_ = node_shared -> declare_parameter<double>(name + ".T", 1.0);
    dt_ = node_shared -> declare_parameter<double>(name + ".dt", 0.1);
    time_between_states_ = node_shared -> declare_parameter<double>(name + ".time_between_states", 3.0);
    iterations_ = node_shared -> declare_parameter<int>(name + ".iterations", 1);
    
    std::vector<double> Q_temp = node_shared->declare_parameter<std::vector<double>>(name + ".Q", {1.0,1.0, 0.3});
    if (Q_temp.size() != 3) {
      RCLCPP_ERROR(node_shared->get_logger(), "incorrect size Q, must be 3 values");
      exit(0);
    }
    Q_(0, 0) = Q_temp[0];
    Q_(1, 1) = Q_temp[1];
    Q_(2, 2) = Q_temp[2];

    std::vector<double> Qf_temp = node_shared->declare_parameter<std::vector<double>>(
      name + ".Qf", {10.0,
        10.0, 0.1});
    if (Qf_temp.size() != 3) {
      RCLCPP_ERROR(node_shared->get_logger(), "incorrect size Qf, must be 3 values");
      exit(0);
    }
    Qf_(0, 0) = Qf_temp[0];
    Qf_(1, 1) = Qf_temp[1];
    Qf_(2, 2) = Qf_temp[2];

    std::vector<double> R_temp = node_shared->declare_parameter<std::vector<double>>(
      name + ".R", {0.1,
        0.05});
    if (R_temp.size() != 2) {
      RCLCPP_ERROR(node_shared->get_logger(), "incorrect size R, must be 2 values");
      exit(0);
    }
    R_(0, 0) = R_temp[0];
    R_(1, 1) = R_temp[1];

    prev_u_ = std::vector<Eigen::Vector2d>(T_ / dt_, Eigen::Vector2d::Zero());
    prev_x_ = std::vector<Eigen::Vector3d>(T_ / dt_, Eigen::Vector3d::Zero());
    S_ = std::vector<Eigen::Matrix3d>(T_ / dt_, Eigen::Matrix3d::Zero());
    // END STUDENT CODE
  }

  Eigen::Vector3d interpolateState(const rclcpp::Time time)
  {
    if (time.seconds() > path_start_time_.seconds() + (time_between_states_ * trajectory_.size())) {
      return trajectory_.back();
    }
    if (time < path_start_time_) {
      return trajectory_.front();
    }

    double rel_time = (time - path_start_time_).seconds();
    int lower_idx = std::floor(rel_time / time_between_states_);
    int upper_idx = lower_idx + 1;
    double alpha = (rel_time - lower_idx * time_between_states_) / time_between_states_;

    Eigen::Vector3d interpolated = (1 - alpha) * trajectory_[lower_idx] + alpha *
      trajectory_[upper_idx];

    // correct the angle average
    if ((trajectory_[lower_idx](2) > 0 && trajectory_[upper_idx](2) < 0 ||
      trajectory_[lower_idx](2) < 0 && trajectory_[upper_idx](2) > 0) &&
      (std::abs(trajectory_[lower_idx](2)) > M_PI_2 && std::abs(
        trajectory_[upper_idx](2)) > M_PI_2))
    {
      double angle_diff = angles::shortest_angular_distance(
        trajectory_[lower_idx](2), trajectory_[upper_idx](2));
      interpolated(2) = trajectory_[lower_idx](2) + alpha * angle_diff;
      interpolated(2) = angles::normalize_angle(interpolated(2));
    }

    return interpolated;
  }

  void activate() override
  {
    traj_viz_pub_->on_activate();
  }

  void deactivate() override {}

  void cleanup() override {}

  void setPlan(const nav_msgs::msg::Path & path) override
  {
    auto node_shared = node_.lock();
    if (!node_shared) {
      throw std::runtime_error{"Could not acquire node."};
    }

    trajectory_.clear();
    std::transform(
      path.poses.begin(), path.poses.end(), std::back_inserter(
        trajectory_), StateFromMsg);
    path_start_time_ = node_shared->now();
    resetStates(Eigen::Vector3d::Zero());
  }

  void resetStates(Eigen::Vector3d init_state)
  {
    auto node_shared = node_.lock();
    if (!node_shared) {
      throw std::runtime_error{"Could not acquire node."};
    }

    prev_x_[0] = init_state;
    prev_u_ = std::vector<Eigen::Vector2d>(T_ / dt_, Eigen::Vector2d::Ones());
    for (int t = 1; t < T_ / dt_; t++) {
      prev_x_[t] = computeNextState(prev_x_[t - 1], prev_u_[t]);
    }

    pubPath();
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override
  {
    // TODO(barulicm) goal_checker added in humble
    auto node_shared = node_.lock();
    if (!node_shared) {
      throw std::runtime_error{"Could not acquire node."};
    }

    Eigen::Vector3d state = StateFromMsg(pose);

    for (int i = 0; i < iterations_; i++) {
      computeRicattiEquation();
      computeForwardPass(state, pose.header.stamp);
    }

    geometry_msgs::msg::TwistStamped cmd_vel_msg;
    if (prev_u_[0].hasNaN()) {
      RCLCPP_INFO_STREAM(
        node_shared->get_logger(),
        "fixing nan control: " << prev_u_[0].transpose());
    }

    cmd_vel_msg.twist.linear.x = std::clamp(prev_u_[0](0), -2.0, 2.0);
    cmd_vel_msg.twist.angular.z = std::clamp(prev_u_[0](1), -2.0, 2.0);
    cmd_vel_msg.header.frame_id = "base_link";
    cmd_vel_msg.header.stamp = node_shared->now();
    return cmd_vel_msg;
  }

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override
  {
    // TODO(barulicm) implement this
  }

  Eigen::Matrix3d computeAMatrix(const Eigen::Vector3d & x, const Eigen::Vector2d & u)
  {
    Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
    A(0, 2) = -u(0) * sin(x(2)) * dt_;
    A(1, 2) = u(0) * cos(x(2)) * dt_;
    return A;
  }

  Eigen::Matrix<double, 3, 2> computeBMatrix(const Eigen::Vector3d & x)
  {
    Eigen::Matrix<double, 3, 2> B = Eigen::Matrix<double, 3, 2>::Zero();
    B(0, 0) = cos(x(2)) * dt_;
    B(1, 0) = sin(x(2)) * dt_;
    B(2, 1) = dt_;
    return B;
  }

  Eigen::Vector3d computeNextState(const Eigen::Vector3d & x, const Eigen::Vector2d & u)
  {
    Eigen::Vector3d result;
    result(0) = x(0) + u(0) * cos(x(2)) * dt_;
    result(1) = x(1) + u(0) * sin(x(2)) * dt_;
    result(2) = x(2) + u(1) * dt_;
    return result;
  }

  void computeRicattiEquation()
  {
    // does the backwards pass of the ricatti equation
    S_[S_.size() - 1] = Qf_;
    for (int t = T_ / dt_ - 2; t >= 0; t--) {
      auto last_S = S_[t + 1];
      Eigen::Matrix3d A = computeAMatrix(prev_x_[t], prev_u_[t]);
      Eigen::Matrix<double, 3, 2> B = computeBMatrix(prev_x_[t]);
      auto K = (R_ + B.transpose() * last_S * B).inverse() * B.transpose() * last_S * A;
      S_[t] = A.transpose() * last_S * A - (A.transpose() * last_S * B) * K + Q_;
    }
  }

  void computeForwardPass(const Eigen::Vector3d & init_x, rclcpp::Time current_time)
  {
    // computes the forward pass to update the states and controls
    Eigen::Vector3d cur_x = init_x;
    for (int t = 0; t < T_ / dt_; t++) {
      Eigen::Matrix3d A = computeAMatrix(cur_x, prev_u_[t]);
      Eigen::Matrix<double, 3, 2> B = computeBMatrix(prev_x_[t]);
      auto current_S = S_[t];
      auto K = (R_ + B.transpose() * current_S * B).inverse() * B.transpose() * current_S * A;

      Eigen::Vector3d target_x =
        interpolateState(current_time + rclcpp::Duration::from_seconds(dt_ * t));
      Eigen::Vector3d state_error = cur_x - target_x;
      state_error(2) = angles::shortest_angular_distance(target_x(2), cur_x(2));

      Eigen::Vector2d u_star = -K * state_error;

      prev_x_[t] = cur_x;
      prev_u_[t] = u_star;

      cur_x = computeNextState(cur_x, u_star);
    }

    pubPath();
  }

  void pubPath()
  {
    auto node_shared = node_.lock();
    if (!node_shared) {
      throw std::runtime_error{"Could not acquire node."};
    }

    nav_msgs::msg::Path path;
    path.header.stamp = node_shared->now();
    path.header.frame_id = "/map";
    for (const Eigen::Vector3d current_state : prev_x_) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = current_state(0);
      pose.pose.position.y = current_state(1);
      path.poses.push_back(pose);
    }
    traj_viz_pub_->publish(path);
  }

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  // dynamics
  double dt_;
  double T_;

  // cost function
  Eigen::Matrix3d Q_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d Qf_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix2d R_ = Eigen::Matrix2d::Identity();

  // Ricatti equation
  std::vector<Eigen::Matrix3d> S_;
  std::vector<Eigen::Vector3d> prev_x_;
  std::vector<Eigen::Vector2d> prev_u_;
  int iterations_;

  // trajectory to track
  std::vector<Eigen::Vector3d> trajectory_;
  double time_between_states_;
  rclcpp::Time path_start_time_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr traj_viz_pub_;
};

}  // namespace controllers

PLUGINLIB_EXPORT_CLASS(controllers::LqrController, nav2_core::Controller)
