#include "obs_avoid/moveit_utils.hpp"

void MoveItUtils::SetCollision(const std::vector<std::shared_ptr<Collision>>& collisions, const std::string& id) {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "base_link";
    collision_object.id = id;
    collision_object.operation = collision_object.ADD;
    for (auto& collision : collisions) {
        collision_object.primitives.push_back(collision->ToPrimitive());
        auto poseMsg = MakePoseMsg(
            collision->x, collision->y, collision->z, collision->_x, collision->_y, collision->_z, collision->_w);
        collision_object.primitive_poses.push_back(poseMsg);
    }
    m_Scene.applyCollisionObject(collision_object);
}

void MoveItUtils::RemoveCollision(const std::string& id){
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "base_link";
    collision_object.id = id;
    collision_object.operation = collision_object.REMOVE;
    m_Scene.applyCollisionObject(collision_object);
    m_MoveGroup.setJointValueTarget(m_MoveGroup.getCurrentJointValues());
}

double MoveItUtils::MoveTraj(const std::vector<geometry_msgs::msg::Pose>& waypoints,
                            int try_count,
                            double jump_threshold,
                            double eef_step,
                            bool avoid_collisions) 
{
    if(!MoveTo(waypoints.front())){
        return 0.0;
    }
    moveit_msgs::msg::RobotTrajectory trajectory;
    // trajcetory.joint_trajectory.header.frame_id = "base_link";
    // trajectory.
    m_MoveGroup.setPoseTarget(waypoints.back());
    double res = 0.0;
    for (int i = 0; i < try_count; i++) {
        double fraction = m_MoveGroup.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory,avoid_collisions);
        if (fraction > 0.95) {
            m_MoveGroup.execute(trajectory);
            return fraction;
        }
        res = res < fraction ? fraction : res;
    }
    
    return res;
}

bool MoveItUtils::MoveTo(geometry_msgs::msg::Pose targetPose) {
    m_MoveGroup.setPoseTarget(targetPose);

    // Create a plan to that target pose
    moveit::planning_interface::MoveGroupInterface::Plan plan_msg;
    bool ok = (m_MoveGroup.plan(plan_msg) == moveit::core::MoveItErrorCode::SUCCESS);
    // auto const ok = static_cast<bool>(m_MoveGroup.plan(plan_msg));

    // Execute the plan
    if (ok) {
        m_MoveGroup.execute(plan_msg);
        return true;
    }
    return false;
}

std::tuple<double, double, double, double> MoveItUtils::ConvertRotate(double roll, double pitch, double yaw) {
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    // 将旋转矩阵组合成一个四元数
    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    return {q.x(), q.y(), q.z(), q.w()};
}

geometry_msgs::msg::Pose MoveItUtils::MakePoseMsg(
    double x, double y, double z, double _x, double _y, double _z, double _w) {
    geometry_msgs::msg::Pose msg;
    msg.position.x = x;
    msg.position.y = y;
    msg.position.z = z;
    msg.orientation.x = _x;
    msg.orientation.y = _y;
    msg.orientation.z = _z;
    msg.orientation.w = _w;
    return msg;
}