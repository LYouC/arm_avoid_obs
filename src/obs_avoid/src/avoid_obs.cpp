#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include "obs_avoid/avoid_obs.hpp"

#include "obs_avoid/moveit_utils.hpp"

// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("avoid_obs");

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto avoid_obs_node = std::make_shared<AvoidObs>("avoid_obs", node_options,"rm_group");
    
    avoid_obs_node->GetMoveitUtils().SetCollision(
        {
            MoveItUtils::MakeBoxCollision(0.0, -0.47, 0.15, 1.5, 0.8, 0.03),
        }
        ,"plane"
    );
    // rclcpp::executors::SingleThreadedExecutor executor;
    // executor.add_node(avoid_obs_node);
    // auto spinner = std::thread([&executor]() { executor.spin(); });

    // RCLCPP_INFO(LOGGER, "AvoidObs node is spinning");


    //spin
    rclcpp::spin(avoid_obs_node);
    rclcpp::shutdown();
    // spinner.join(); 
    return 0;

}
