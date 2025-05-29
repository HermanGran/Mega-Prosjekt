#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include "std_srvs/srv/trigger.hpp"
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <std_msgs/msg/string.hpp>

class MotionPlannerNode : public rclcpp::Node {
public:
    MotionPlannerNode() : Node("motion_planner_node"),
        move_group_(std::shared_ptr<rclcpp::Node>(this, [](auto*){}), "ur_manipulator") 
    {
        move_group_.setPlanningTime(10.0);  // Increase from default 5s
        move_group_.setNumPlanningAttempts(10);  // Default is 1
        move_group_.setMaxVelocityScalingFactor(0.5); 

        // Declaring parameters
        this->declare_parameter("home_pose.position.x", -0.24);
        this->declare_parameter("home_pose.position.y", 0.43);
        this->declare_parameter("home_pose.position.z", 0.6);
        this->declare_parameter("home_pose.orientation.x", 0.38);
        this->declare_parameter("home_pose.orientation.y", 0.92);
        this->declare_parameter("home_pose.orientation.z", 0.0);
        this->declare_parameter("home_pose.orientation.w", 0.0);

        // Creating Home Pose
        home_pose_.header.frame_id = "base_link";
        home_pose_.pose.position.x = this->get_parameter("home_pose.position.x").as_double();
        home_pose_.pose.position.y = this->get_parameter("home_pose.position.y").as_double();
        home_pose_.pose.position.z = this->get_parameter("home_pose.position.z").as_double();
        home_pose_.pose.orientation.x = this->get_parameter("home_pose.orientation.x").as_double();
        home_pose_.pose.orientation.y = this->get_parameter("home_pose.orientation.y").as_double();
        home_pose_.pose.orientation.z = this->get_parameter("home_pose.orientation.z").as_double();
        home_pose_.pose.orientation.w = this->get_parameter("home_pose.orientation.w").as_double();

        // Creating Subsribers
        pose_subscriber_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/cube_pose", 10, std::bind(&MotionPlannerNode::pose_callback, this, std::placeholders::_1));

        // Creating Publishers
        ready_publisher_ = create_publisher<std_msgs::msg::Bool>("/enable_detection", 10);
        cube_marker_ = create_publisher<geometry_msgs::msg::PoseStamped>("/cube_marker", 10);
        cube_pose_done_ = create_publisher<std_msgs::msg::Bool>("/cube_pose_done", 10);

        // Creating Service
        go_home_service_ = create_service<std_srvs::srv::Trigger>("/go_to_home", std::bind(&MotionPlannerNode::go_to_home_callback, this, std::placeholders::_1, std::placeholders::_2));
        
        RCLCPP_INFO(get_logger(), "Motion planner initialized successfully");
    }

private:
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "Planning to target pose...");
        
        geometry_msgs::msg::PoseStamped target_pose = *msg;
        target_pose.pose.orientation = home_pose_.pose.orientation;
        move_group_.setPoseTarget(target_pose);

        auto const [success, plan] = [this]{
            moveit::planning_interface::MoveGroupInterface::Plan msg;
            auto const ok = static_cast<bool>(move_group_.plan(msg));
            return std::make_pair(ok, msg);
        }();

        geometry_msgs::msg::PoseStamped cube_pose_pub;
        cube_pose_pub = target_pose;
        cube_marker_->publish(cube_pose_pub);

        std_msgs::msg::Bool done_msg;
        done_msg.data = false;

        if (success) {
            RCLCPP_INFO(get_logger(), "Executing plan...");
            moveit::core::MoveItErrorCode result = move_group_.execute(plan);
            if (result == moveit::core::MoveItErrorCode::SUCCESS) {
                done_msg.data = true;
                RCLCPP_INFO(get_logger(), "Moved to cube!");
            }

        } else {
            RCLCPP_ERROR(get_logger(), "Planning failed!");
        }
        cube_pose_done_->publish(done_msg);
    }

    // Service function to go to home pose, then send boolean ready to take picture
    void go_to_home_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

        RCLCPP_INFO(get_logger(), "Moving to home position... ");

        home_pose_.header.frame_id = "base_link";
        move_group_.setPoseTarget(home_pose_);

        auto const [success, plan] = [this]{
            moveit::planning_interface::MoveGroupInterface::Plan msg;
            auto const ok = static_cast<bool>(move_group_.plan(msg));
            return std::make_pair(ok, msg);
        }();

        std_msgs::msg::Bool ready_msg;
        ready_msg.data = false;  // Default to "not ready"

        if (success) {
            moveit::core::MoveItErrorCode result = move_group_.execute(plan);
            if (result == moveit::core::MoveItErrorCode::SUCCESS) {
                response->success = true;
                response->message = "Moved to home position.";
                ready_msg.data = true;  // Only set true after successful execution
            } else {
                response->success = false;
                response->message = "Execution failed.";
            }
        } else {
            response->success = false;
            response->message = "Planning failed.";
        }
    
        ready_publisher_->publish(ready_msg);  // Publish final state
    }

    // Member variables
    moveit::planning_interface::MoveGroupInterface move_group_;
    geometry_msgs::msg::PoseStamped home_pose_;
    
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cube_pose_done_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr cube_marker_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr go_home_service_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotionPlannerNode>();
    
    // Use multi-threaded executor that I got from DeepSeek
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}