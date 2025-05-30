#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include "std_srvs/srv/trigger.hpp"
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <vector>
#include <map>
#include <algorithm>

class MotionPlannerNode : public rclcpp::Node {
public:
    MotionPlannerNode() : Node("motion_planner_node"), 
        move_group_(std::shared_ptr<rclcpp::Node>(this, [](auto*){}), "ur_manipulator") {
        
        // MoveIt configuration
        move_group_.setPlanningTime(10.0);
        move_group_.setNumPlanningAttempts(5); 
        move_group_.setMaxVelocityScalingFactor(0.3); 

        // Parameters
        this->declare_parameter("home_pose.position.x", -0.24);
        this->declare_parameter("home_pose.position.y", 0.43);
        this->declare_parameter("home_pose.position.z", 0.6);
        this->declare_parameter("home_pose.orientation.x", 0.38);
        this->declare_parameter("home_pose.orientation.y", 0.0);
        this->declare_parameter("home_pose.orientation.z", 0.0);
        this->declare_parameter("home_pose.orientation.w", 0.0);
        
        // Search pattern parameters
        this->declare_parameter("search_radius", 0.3);
        this->declare_parameter("search_points", 4);
        this->declare_parameter("search_height", 0.6);
        
        // Home pose
        home_pose_.header.frame_id = "base_link";
        home_pose_.pose.position.x = this->get_parameter("home_pose.position.x").as_double();
        home_pose_.pose.position.y = this->get_parameter("home_pose.position.y").as_double();
        home_pose_.pose.position.z = this->get_parameter("home_pose.position.z").as_double();
        home_pose_.pose.orientation = get_orientation();

        // Publishers
        ready_publisher_ = create_publisher<std_msgs::msg::Bool>("/enable_detection", 10);
        cube_pose_done_ = create_publisher<std_msgs::msg::Bool>("/cube_pose_done", 10);

        // Subscribers
        pose_subscriber_ = create_subscription<geometry_msgs::msg::PoseStamped>("/cube_pose", 10, std::bind(&MotionPlannerNode::pose_callback, this, std::placeholders::_1));
        search_subscriber_ = create_subscription<std_msgs::msg::String>("/request_search", 10, std::bind(&MotionPlannerNode::search_callback, this, std::placeholders::_1));

        // Services
        go_home_service_ = create_service<std_srvs::srv::Trigger>("/go_to_home", std::bind(&MotionPlannerNode::go_to_home_callback, this, std::placeholders::_1, std::placeholders::_2));
        
        RCLCPP_INFO(get_logger(), "Motion planner initialized successfully");
    }

private:
    geometry_msgs::msg::Quaternion get_orientation() {
        geometry_msgs::msg::Quaternion orientation;
        orientation.x = this->get_parameter("home_pose.orientation.x").as_double();
        orientation.y = this->get_parameter("home_pose.orientation.y").as_double();
        orientation.z = this->get_parameter("home_pose.orientation.z").as_double();
        orientation.w = this->get_parameter("home_pose.orientation.w").as_double();
        return orientation;
    }

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


    void search_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::string missing_colors = msg->data;
        RCLCPP_INFO(get_logger(), "Starting search for missing cubes: %s", missing_colors.c_str());
        
        // Go to home position first
        go_to_home(false);
        
        // Execute search pattern
        execute_search_pattern();
    }

    void execute_search_pattern() {
        double radius = this->get_parameter("search_radius").as_double();
        int points = this->get_parameter("search_points").as_int();
        double height = this->get_parameter("search_height").as_double();
        
        // Generate circular search pattern around home position
        std::vector<geometry_msgs::msg::Pose> search_poses;
        for (int i = 0; i < points; ++i) {
            double angle = 2.0 * M_PI * i / points;
            geometry_msgs::msg::Pose pose = home_pose_.pose;
            pose.position.x += radius * cos(angle);
            pose.position.y += radius * sin(angle);
            pose.position.z = height;
            search_poses.push_back(pose);
        }

        // Add home position at the end
        search_poses.push_back(home_pose_.pose);

        // Execute search pattern
        for (const auto& pose : search_poses) {
            move_group_.setPoseTarget(pose);
            auto const [success, plan] = [this]{
                moveit::planning_interface::MoveGroupInterface::Plan msg;
                auto const ok = static_cast<bool>(move_group_.plan(msg));
                return std::make_pair(ok, msg);
            }();

            if (success) {
                move_group_.execute(plan);
                
                // Enable detection at each search point
                enable_detection(true);
                
                // Wait a bit for detection
                rclcpp::sleep_for(std::chrono::seconds(2));
                
                // Disable detection before moving to next point
                enable_detection(false);
            }
        }

        
    }

    void go_to_home(bool enable_detection_after=true) {
        RCLCPP_INFO(get_logger(), "Going home.....");
        move_group_.setPoseTarget(home_pose_);

        auto const [success, plan] = [this]{
            moveit::planning_interface::MoveGroupInterface::Plan msg;
            auto const ok = static_cast<bool>(move_group_.plan(msg));
            return std::make_pair(ok, msg);
        }();

        if (success) {
            moveit::core::MoveItErrorCode result = move_group_.execute(plan);
            if (result == moveit::core::MoveItErrorCode::SUCCESS && enable_detection_after) {
                enable_detection(true);
            }
        }
    }

    void enable_detection(bool enable) {
        auto msg = std_msgs::msg::Bool();
        msg.data = enable;
        ready_publisher_->publish(msg);
    }

    void go_to_home_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request>, 
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        go_to_home();
        response->success = true;
        response->message = "Moved to home position.";
    }

    // Member variables
    moveit::planning_interface::MoveGroupInterface move_group_;
    geometry_msgs::msg::PoseStamped home_pose_;
    
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr search_subscriber_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr cube_marker_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cube_pose_done_;

    // Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr go_home_service_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotionPlannerNode>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}