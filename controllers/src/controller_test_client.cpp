#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include "test_path_generator.hpp"

namespace controllers {

class ControllerTestClient : public rclcpp::Node {
    public:
        using FollowPath = nav2_msgs::action::FollowPath;
        using GoalHandle = rclcpp_action::ClientGoalHandle<FollowPath>;
        explicit ControllerTestClient(const rclcpp::NodeOptions &options)
                : rclcpp::Node("controller_test_client", options) {
            client_ = rclcpp_action::create_client<FollowPath>(this, "follow_path");
            rclcpp::QoS qos_profile{1};
            qos_profile.transient_local();
            publisher_ = create_publisher<nav_msgs::msg::Path>(
                    "/plan", qos_profile);
            std::thread(&ControllerTestClient::SendGoal, this).detach();
        }

        void SendGoal() {
            if (!client_ -> wait_for_action_server(std::chrono::seconds(10))) {
                RCLCPP_ERROR(get_logger(), "Action server not ready. Waiting timed out.");
                rclcpp::shutdown();
            }
            FollowPath::Goal goal;
            goal.path = TestPathGenerator(20).BuildPath();
            goal.path.header.frame_id = "map";
            goal.path.header.stamp = now();

            publisher_ -> publish(goal.path);

            rclcpp_action::Client<FollowPath>::SendGoalOptions goal_options;
            goal_options.goal_response_callback = std::bind(&GoalResponseCallback, this, std::placeholders::_1);
            goal_options.feedback_callback = std::bind(&FeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
            goal_options.result_callback = std::bind(&ResultCallback, this, std::placeholders::_1);

            client_ -> async_send_goal(goal, goal_options);
        }

        void GoalResponseCallback(GoalHandle::SharedPtr goal_handle) {
            if (!goal_handle) {
                RCLCPP_ERROR(get_logger(), "Server rejected goal.");
            } else {
                RCLCPP_INFO(get_logger(), "Server accepted goal.");
            }
        }

        void FeedbackCallback(GoalHandle::SharedPtr goal_handle,
                              const std::shared_ptr<const FollowPath::Feedback> feedback) {
            RCLCPP_INFO(get_logger(), "Distance to goal: %f", feedback->distance_to_goal);
        }

        void ResultCallback(const GoalHandle::WrappedResult &result) {
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(get_logger(), "Path following completed!");
                break;
                    case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(get_logger(), "Path following aborted!");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(get_logger(), "Path following canceled!");
                    break;
                case rclcpp_action::ResultCode::UNKNOWN:
                    RCLCPP_ERROR(get_logger(), "Unkown action result code!");
                    break;
            }
            rclcpp::shutdown();
        }

    private:
        rclcpp_action::Client<FollowPath>::SharedPtr client_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
};

} // namespace my_package

RCLCPP_COMPONENTS_REGISTER_NODE(controllers::ControllerTestClient)