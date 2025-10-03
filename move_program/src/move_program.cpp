#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    
    auto const node = std::make_shared<rclcpp::Node>(
        "hello_moveit",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // Spin node in thread
    std::thread spin_thread([node]() { rclcpp::spin(node); });

    auto const logger = rclcpp::get_logger("hello_moveit");

    // Wait for move_group to be ready
    RCLCPP_INFO(logger, "Waiting for move_group action server...");
    rclcpp::sleep_for(std::chrono::seconds(5));

    // Create MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "arm");

    RCLCPP_INFO(logger, "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
    RCLCPP_INFO(logger, "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

    // Your target pose...
    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.x = 0.0;
    target_pose.orientation.y = 0.707;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 0.707;
    target_pose.position.x = 0.0;
    target_pose.position.y = 0.0;
    target_pose.position.z = 0.3;

    move_group_interface.setPoseTarget(target_pose);
    move_group_interface.setPlanningTime(10.0);

    auto const result = move_group_interface.move();
    
    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(logger, "Movement completed successfully!");
    } else {
        RCLCPP_ERROR(logger, "Movement failed with error code: %d", result.val);
    }

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}