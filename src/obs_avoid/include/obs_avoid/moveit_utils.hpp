
/*

该文件对需要使用到的部分进行再次封装
主要包括：1.末端位置规划
        2.路径点规划
        3.停止
        4.collison管理
*/

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <memory>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tuple>

struct Collision {
    double x, y, z;
    double _x = 0.0, _y = 0.0, _z = 0.0, _w = 1.0;
    virtual const shape_msgs::msg::SolidPrimitive& ToPrimitive() = 0;
    Collision(double x, double y, double z) : x(x), y(y), z(z) {}
};

struct BoxCollision : public Collision {
    double BOX_X = 0.0, BOX_Y = 0.0, BOX_Z = 0.0;
    shape_msgs::msg::SolidPrimitive Primitive;

    BoxCollision(double x, double y, double z, double boxX, double boxY, double boxZ)
        : Collision(x, y, z), BOX_X(boxX), BOX_Y(boxY), BOX_Z(boxZ) {
        Primitive.type = Primitive.BOX;
        Primitive.dimensions.resize(3);
    }

    virtual const shape_msgs::msg::SolidPrimitive& ToPrimitive() {
        Primitive.dimensions[Primitive.BOX_X] = BOX_X;
        Primitive.dimensions[Primitive.BOX_Y] = BOX_Y;
        Primitive.dimensions[Primitive.BOX_Z] = BOX_Z;
        return Primitive;
    }
};

struct CylinderCollision : public Collision {
    double CYLINDER_RADIUS = 0.0, CYLINDER_HEIGHT = 0.0;
    shape_msgs::msg::SolidPrimitive Primitive;

    CylinderCollision(double x, double y, double z, double radius, double height)
        : Collision(x, y, z), CYLINDER_RADIUS(radius), CYLINDER_HEIGHT(height) {
        Primitive.type = Primitive.CYLINDER;
        Primitive.dimensions.resize(2);
    }

    virtual const shape_msgs::msg::SolidPrimitive& ToPrimitive() {
        Primitive.dimensions[Primitive.CYLINDER_RADIUS] = CYLINDER_RADIUS;
        Primitive.dimensions[Primitive.CYLINDER_HEIGHT] = CYLINDER_HEIGHT;
        return Primitive;
    }
};

class MoveItUtils {
  public:
    MoveItUtils(std::shared_ptr<rclcpp::Node> node, std::string group_name) : m_MoveGroup(node, group_name) {
        m_MoveGroup.setPlanningTime(5.0);
        m_MoveGroup.setNumPlanningAttempts(5);
    }

    /**
     * @brief change the rpy to quaternion
     *
     * @param roll
     * @param pitch
     * @param yaw
     * @return std::tuple<double,double,double,double> x,y,z,w
     */
    static std::tuple<double, double, double, double> ConvertRotate(double roll, double pitch, double yaw);

    static geometry_msgs::msg::Pose MakePoseMsg(
        double x, double y, double z, double roll = 0.0, double pitch = 0.0, double yaw = 0.0) {
        auto [_x, _y, _z, _w] = ConvertRotate(roll, pitch, yaw);
        return MakePoseMsg(x, y, z, _x, _y, _z, _w);
    }

    static geometry_msgs::msg::Pose MakePoseMsg(
        double x, double y, double z, double _x, double _y, double _z, double _w);

    static std::shared_ptr<BoxCollision> MakeBoxCollision(
        double x, double y, double z, double boxX, double boxY, double boxZ) {
        return std::make_shared<BoxCollision>(x, y, z, boxX, boxY, boxZ);
    }

    static std::shared_ptr<CylinderCollision> MakeCylinderCollision(
        double x, double y, double z, double radius, double height) {
        return std::make_shared<CylinderCollision>(x, y, z, radius, height);
    }

    bool MoveTo(geometry_msgs::msg::Pose targetPose);

    inline void Stop() { m_MoveGroup.stop(); }

    void SetCollision(const std::vector<std::shared_ptr<Collision>>& collisions, const std::string& id);

    double MoveTraj(const std::vector<geometry_msgs::msg::Pose>& waypoints,
                    int try_count = 50,
                    double jump_threshold = 0.0,
                    double eef_step = 0.01,
                    bool avoid_collisions = true);

    void RemoveCollision(const std::string& id);

    moveit::planning_interface::MoveGroupInterface m_MoveGroup;
    moveit::planning_interface::PlanningSceneInterface m_Scene;
};