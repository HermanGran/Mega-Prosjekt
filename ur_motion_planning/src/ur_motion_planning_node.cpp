#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("motion_planner_node");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Allow time for MoveIt to load robot model
  rclcpp::sleep_for(std::chrono::seconds(2));

  // Create MoveGroupInterface for the arm
  moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");

  // Plan to a named target (e.g., "home" defined in SRDF)
  /*
  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = 0.4;
  target_pose.position.y = 0.2;
  target_pose.position.z = 0.3;
  target_pose.orientation.w = 1.0;
  move_group.setPoseTarget(target_pose);
  */
  move_group.setNamedTarget("home");


  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = static_cast<bool>(move_group.plan(my_plan));

  if (success)
  {
    RCLCPP_INFO(node->get_logger(), "Planning succeeded!");
    move_group.execute(my_plan);
    // Print or send the trajectory:
    //my_plan.trajectory;
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Planning failed.");
  }

  rclcpp::shutdown();
  return 0;
}
