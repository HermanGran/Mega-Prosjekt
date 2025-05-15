#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

class MotionPlannerNode : public rclcpp::Node
{
public:
  MotionPlannerNode()
  : Node("motion_planner_node")
  {
    using std::placeholders::_1;

    // Subscribe to pose input
    pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/cube_pose", 10, std::bind(&MotionPlannerNode::pose_callback, this, _1));

    // Wait for robot_description parameter
    this->declare_parameter<std::string>("robot_description", "");
    robot_description_ = this->get_parameter("robot_description").as_string();

    if (robot_description_.empty()) {
      RCLCPP_WARN(this->get_logger(), "Waiting for robot_description parameter...");
      robot_description_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        [this]() {
          robot_description_ = this->get_parameter("robot_description").as_string();
          if (!robot_description_.empty()) {
            robot_description_timer_->cancel();
            initializeMoveGroup();
          }
        });
    } else {
      initializeMoveGroup();
    }
  }

private:
  void initializeMoveGroup()
  {
    RCLCPP_INFO(this->get_logger(), "Initializing MoveGroupInterface...");
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "ur_manipulator");
    RCLCPP_INFO(this->get_logger(), "Motion planner node is ready and listening for poses...");
  }

  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!move_group_) {
      RCLCPP_WARN(this->get_logger(), "MoveGroupInterface not initialized yet. Ignoring pose.");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Received target pose. Planning...");

    move_group_->setPoseTarget(*msg);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group_->plan(plan));

    if (success)
    {
      RCLCPP_INFO(this->get_logger(), "Planning succeeded. Executing...");
      move_group_->execute(plan);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Planning failed.");
    }
  }

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
  rclcpp::TimerBase::SharedPtr robot_description_timer_;
  std::string robot_description_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotionPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}