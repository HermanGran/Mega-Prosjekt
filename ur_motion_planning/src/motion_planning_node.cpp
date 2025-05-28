#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include "std_srvs/srv/trigger.hpp"  // built-in ROS 2 service type

using std::placeholders::_1;

class MotionPlannerNode : public rclcpp::Node {
    public:

    MotionPlannerNode() : Node("MotionPlannerNode") {
        // Starting Subscriber
        pose_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/cube_pose", 10, std::bind(&MotionPlannerNode::pose_callback, this, _1)
        );

        ready_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/enable_detection", 10);
        
        go_home_service_ = this->create_service<std_srvs::srv::Trigger>("/go_to_home", std::bind(&MotionPlannerNode::go_to_home_callback, this, std::placeholders::_1, std::placeholders::_2));

    }

    
    void setup() {
        RCLCPP_INFO(this->get_logger(), "Initializing MoveGroupInterface...");
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(this->shared_from_this(), "ur_manipulator");
        // Give it time to connect to move group
        rclcpp::sleep_for(std::chrono::seconds(2));
    }

    private:

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Recieved target pose. Planning...");
        
        //geometry_msgs::msg::Pose current_pose = move_group_->getCurrentPose().pose;
        geometry_msgs::msg::PoseStamped target_pose;
        //target_pose.header.frame_id = "base_link";

        target_pose.pose.position.x = home_pose.pose.position.x - msg->pose.position.x;
        target_pose.pose.position.y = home_pose.pose.position.y - msg->pose.position.y;
        target_pose.pose.position.z = 0.15;

        target_pose.pose.orientation.x = -0.38;
        target_pose.pose.orientation.y = -0.92;
        target_pose.pose.orientation.z = 0.0;
        target_pose.pose.orientation.w = 0.0;

        move_group_->setPoseTarget(target_pose);


        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = static_cast<bool>(move_group_->plan(plan));

        if (success) {
            RCLCPP_INFO(this->get_logger(), "Planning succeeded. Executing...");
            move_group_->execute(plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning failed.");
        }

    }

    void ready_callback(const std_msgs::msg::Bool msg) {
        ready_publisher_->publish(msg);
    }

    void go_to_home_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to go to home pose");
        home_pose.header.frame_id = "base_link";

        // Got this position from chatgpt
        // Position: Slightly above the table
        home_pose.pose.position.x = -0.25;
        home_pose.pose.position.y = 0.45;
        home_pose.pose.position.z = 0.66;

        // Orientation: TCP pointing down (180° about X-axis)
        home_pose.pose.orientation.x = -0.38;
        home_pose.pose.orientation.y = -0.92;
        home_pose.pose.orientation.z = 0.0;
        home_pose.pose.orientation.w = 0.0;

        move_group_->setPoseTarget(home_pose);
    
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = static_cast<bool>(move_group_->plan(plan));
    
        std_msgs::msg::Bool msg;

        if (success) {
            move_group_->execute(plan);
            response->success = true;
            response->message = "Moved to home position.";

            msg.data = true;
            ready_callback(msg);
        } else {
            response->success = false;
            response->message = "Failed to move to home.";

            msg.data = false;
            ready_callback(msg);
        }
    }

    // Variables
    rclcpp::TimerBase::SharedPtr timer_;

    // Poses
    geometry_msgs::msg::PoseStamped home_pose;

    // Service, subs and pups
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_publisher_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr go_home_service_;

    // Init move group
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotionPlannerNode>();
    node->setup();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}