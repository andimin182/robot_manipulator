#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>


void move_robot(const std::shared_ptr<rclcpp::Node> node){
    auto arm_group = moveit::planning_interface::MoveGroupInterface(node, "arm");
    auto gripper_group = moveit::planning_interface::MoveGroupInterface(node, "gripper");

    std::vector<double> arm_joints_goal = {1.57, 0.0, 0.0};
    std::vector<double> gripper_joints_goal = {-0.7, 0.7};

    bool arm_within_lim = arm_group.setJointValueTarget(arm_joints_goal);
    bool gripper_within_lim = gripper_group.setJointValueTarget(gripper_joints_goal);

    if(!arm_within_lim || !gripper_within_lim){
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Target joint positions are outside the workspace");
        return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
    bool arm_plan_success = arm_group.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS;
    bool gripper_plan_success = gripper_group.plan(gripper_plan)== moveit::core::MoveItErrorCode::SUCCESS;

    if(arm_plan_success && gripper_plan_success){
        arm_group.move();
        gripper_group.move();
    }else{
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "The planning failed");
        return;
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("simple_moveit_api_node");
    move_robot(node);
    rclcpp::shutdown();
    return 0;
}
