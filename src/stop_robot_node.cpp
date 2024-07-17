#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class StopRobotNode : public rclcpp::Node
{
public:
    StopRobotNode()
        : Node("stop_robot_node")
    {
        action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
        send_cancel_request();
    }

private:
    void send_cancel_request()
    {
        if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }

        auto goal_handle = action_client_->async_cancel_all_goals();
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to cancel all goals");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "All goals successfully cancelled");
        }
    }

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StopRobotNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

