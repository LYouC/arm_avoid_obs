#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "obs_avoid/moveit_utils.hpp"
#include "obs_avoid/transform.hpp"

class AvoidObs : public rclcpp::Node {
  public:
    AvoidObs(const std::string& name, rclcpp::NodeOptions options, const std::string& group_name)
        :rclcpp::Node(name, options),
        m_MoveitUtils(std::shared_ptr<rclcpp::Node>(this), group_name),
        m_CamTrans(0, -1.2, 1.25, -1.57, 0.0, 0.0),
        m_ArmTrans(0, 0, 0.85, 0, 0, 0) 
    {
        m_DetectResultSub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/detect_result", 1, std::bind(&AvoidObs::DetectResultCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(get_logger(), "Detect Result: %d", 887);
    }

    ~AvoidObs(){}

    void DetectResultCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "Detect Result: %d", msg->data.size());
        if(m_AlreadySetCollision and msg->data.size()>0 and msg->data[0]==0){
            return;
        }
        // 在场景中设置障碍物
        std::vector<std::shared_ptr<Collision>> obsArr;
        for(int i=1; i<msg->data.size(); i+=7){
            double x = msg->data[i];
            double y = msg->data[i+1];
            double z = msg->data[i+2];
            double radius = msg->data[i+3];
            double height = msg->data[i+4];
            auto world_pos = m_CamTrans.SelfToWorld(x,y,z,0,0,0);
            RCLCPP_INFO(get_logger(), "cam_pos: %f, %f, %f", x, y, z);
            RCLCPP_INFO(get_logger(), "world_pos: %f, %f, %f", world_pos[0], world_pos[1], world_pos[2]);
            auto arm_pos = m_ArmTrans.WorldToSelf(world_pos[0],world_pos[1],world_pos[2],0,0,0);
            RCLCPP_INFO(get_logger(), "arm_pos: %f, %f, %f", arm_pos[0], arm_pos[1], arm_pos[2]);

            obsArr.push_back(
                MoveItUtils::MakeCylinderCollision(
                    arm_pos[0], arm_pos[1], arm_pos[2],
                    radius, height
                )
            );
        }
        m_MoveitUtils.SetCollision(obsArr,"giso");
        m_AlreadySetCollision = true;
    }

    MoveItUtils& GetMoveitUtils() { return m_MoveitUtils; }

  private:
    MoveItUtils m_MoveitUtils;
    Transform m_CamTrans;
    Transform m_ArmTrans;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr m_DetectResultSub;
    bool m_AlreadySetCollision = false;
};