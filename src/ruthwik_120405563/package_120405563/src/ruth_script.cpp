
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");
  std::cout << "hello";
  

// Next step goes here
//----------------------------------------------------------------------------------------------------------
  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "panda_arm"); // Initialize a MoveGroupInterface for the "panda_arm" planning group.
  auto move_group_interface_hand = MoveGroupInterface(node, "hand"); // Initialize a MoveGroupInterface for the "hand" planning group.



//----------  Correcting gripper position  ------------------------------------------------------------------------------------------------------------------------------

  std::vector<double> turn = {0.0, -0.0, 0.0, -0.0, 0.0, -0.0, 0.785}; // Example joint angles
    move_group_interface.setJointValueTarget(turn);
    move_group_interface.setPlanningTime(10); // Increase planning time to 10 seconds
    move_group_interface.setGoalTolerance(0.01); // Set the tolerance for achieving the goal

  // Create a plan to that target pose
  auto const [success0, plan0] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success0) {
    move_group_interface.execute(plan0);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
    std::cout << "failed";
  }


//----------------- open gripper fingers  ----------------------------------------------------------------------------------------------

  std::vector<double> grip0 = {0.04,0.04}; // Example joint angles
    move_group_interface_hand.setJointValueTarget(grip0);
    move_group_interface_hand.setPlanningTime(10); // Increase planning time to 10 seconds
    move_group_interface_hand.setGoalTolerance(0.01); // Set the tolerance for achieving the goal

    // Create a plan to that target pose
  auto const [success00, plan00] = [&move_group_interface_hand]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface_hand.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success00) {
    move_group_interface.execute(plan00);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
    std::cout << "failed";
  }


//---------------------- reach to pick -----------------------------------------------------------------------------------------

  std::vector<double> pick = {0.35, 0.907, 0.38, -1.39, -0.24, 2.02, -1.09}; // Example joint angles
    move_group_interface.setJointValueTarget(pick);
    move_group_interface.setPlanningTime(10); // Increase planning time to 10 seconds
    move_group_interface.setGoalTolerance(0.01); // Set the tolerance for achieving the goal
 
    // Create a plan to that target pose
  auto const [success1, plan1] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success1) {
    move_group_interface.execute(plan1);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
    std::cout << "failed";
  }

//----------------- close gripper fingers  ----------------------------------------------------------------------------------------------

  std::vector<double> grip01 = {0.0,0.0}; // Example joint angles
    move_group_interface_hand.setJointValueTarget(grip01);
    move_group_interface_hand.setPlanningTime(10); // Increase planning time to 10 seconds
    move_group_interface_hand.setGoalTolerance(0.01); // Set the tolerance for achieving the goal

    // Create a plan to that target pose
  auto const [success001, plan001] = [&move_group_interface_hand]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface_hand.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success001) {
    move_group_interface.execute(plan001);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
    std::cout << "failed";
  }

 

//----------------------- move to another position to place -----------------------------------------------------------------------------------------------------------------

  std::vector<double> place = {1.9, 1.20, 0.0, -0.6, -2.9, 3.4, 0.6}; // Example joint angles
    move_group_interface.setJointValueTarget(place);
    move_group_interface.setPlanningTime(10); // Increase planning time to 10 seconds
    move_group_interface.setGoalTolerance(0.01); // Set the tolerance for achieving the goal

  // Create a plan to that target pose
  auto const [success2, plan2] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success2) {
    move_group_interface.execute(plan2);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
    std::cout << "failed";
  }

//----------------- open gripper fingers  ----------------------------------------------------------------------------------------------

  std::vector<double> grip02 = {0.04,0.04}; // Example joint angles
    move_group_interface_hand.setJointValueTarget(grip02);
    move_group_interface_hand.setPlanningTime(10); // Increase planning time to 10 seconds
    move_group_interface_hand.setGoalTolerance(0.01); // Set the tolerance for achieving the goal

    // Create a plan to that target pose
  auto const [success002, plan002] = [&move_group_interface_hand]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface_hand.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success002) {
    move_group_interface.execute(plan002);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
    std::cout << "failed";
  }

 

//---------------------------------- back to home position ------------------------------------------------------------------------------------------------------

  std::vector<double> home = {0.0, -0.0, 0.0, -0.0, 0.0, -0.0, 0.785}; // Example joint angles
    move_group_interface.setJointValueTarget(home);
    move_group_interface.setPlanningTime(10); // Increase planning time to 10 seconds
    move_group_interface.setGoalTolerance(0.01); // Set the tolerance for achieving the goal

  // Create a plan to that target pose
  auto const [success3, plan3] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success3) {
    move_group_interface.execute(plan3);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
    std::cout << "failed";
  }

//----------------------------------------------------------------------------------------------------------------------------------------









  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}