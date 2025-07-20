#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <chrono>
#include <vector>
#include <string>
#include <map>
#include <thread> // Added for std::this_thread::sleep_for

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "melfa_waypoints_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Fetch parameters from /move_group
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node, "/move_group");
  
  // Wait for service
  while (!parameters_client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for /move_group service.");
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for /move_group service...");
  }

  // Get robot_description and robot_description_semantic
  auto robot_description = parameters_client->get_parameter<std::string>("robot_description");
  auto robot_description_semantic = parameters_client->get_parameter<std::string>("robot_description_semantic");

  // Set them in your node's namespace (required for MoveGroupInterface)
  node->declare_parameter("robot_description", robot_description);
  node->declare_parameter("robot_description_semantic", robot_description_semantic);

  // Now initialize MoveGroupInterface
  moveit::planning_interface::MoveGroupInterface move_group(node, "rv5as");
  
  // Set planning time
  move_group.setPlanningTime(5.0);
  
  // Get waypoints from parameters
  // auto waypoints_param = node->get_parameter("waypoints");
  // RCLCPP_INFO(node->get_logger(), "Waypoints parameter loaded successfully");
  
  // Define waypoints from the YAML structure
  std::vector<std::vector<double>> waypoints;
  
  std::vector<double> wp1 = {0.0, 0.0, 1.5706095551480073,
                            0.0, 1.570767235778412, 0.0};

  std::vector<double> wp2 = {0.7165540729569084, 0.1334769381719665, 1.0015808924991276, 
                             -0.3849133221872274, 1.5268003708870115, 0.6384158763553069 };
  
  
  std::vector<double> wp3 = {-0.6280110717230564, 0.1595050484323332, 0.9183577066893197,
                              0.3423165307512353, 1.5520157510947687, -0.5440797202881191};
  waypoints = {wp1, wp2, wp3};
  
  RCLCPP_INFO(node->get_logger(), "Starting infinite waypoint loop with %zu waypoints", waypoints.size());
  
  // Infinite loop through waypoints
  size_t current_waypoint = 0;
  
  while (rclcpp::ok()) {
    RCLCPP_INFO(node->get_logger(), "Moving to waypoint %zu", current_waypoint + 1);
    
    // Set joint target for current waypoint
    move_group.setJointValueTarget(waypoints[current_waypoint]);
    
    // Plan and execute
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (success) {
      RCLCPP_INFO(node->get_logger(), "Planning successful, executing waypoint %zu...", current_waypoint + 1);
      move_group.execute(plan);
      RCLCPP_INFO(node->get_logger(), "Waypoint %zu reached successfully", current_waypoint + 1);
    } else {
      RCLCPP_ERROR(node->get_logger(), "Planning failed for waypoint %zu", current_waypoint + 1);
    }
    
    // Move to next waypoint
    current_waypoint++;
    
    // Reset to first waypoint when we reach the end
    if (current_waypoint >= waypoints.size()) {
      current_waypoint = 0;
      RCLCPP_INFO(node->get_logger(), "Completed waypoint cycle, starting over...");
    }
    
    // Wait a bit before moving to next waypoint
    std::this_thread::sleep_for(std::chrono::seconds(2));
  }

  rclcpp::shutdown();
  return 0;
}