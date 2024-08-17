#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>

#include "obs_avoid/moveit_utils.hpp"
#include "obs_avoid/transform.hpp"
#include "obs_avoid/Obstacle.hpp"


class AvoidObsNode : public rclcpp::Node {
  public:
    AvoidObsNode(const std::string& name, rclcpp::NodeOptions options, const std::string& group_name)
        :rclcpp::Node(name, options),
        m_MoveitUtils(std::shared_ptr<rclcpp::Node>(this), group_name),
        m_CamTrans(0, -1.2, 1.25, -1.57, 0.0, 0.0),
        m_ArmTrans(0, 0, 0.85, 0, 0, 0) 
    {
        // rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        // auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);


        m_DetectResultSub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/detect_result", 1, std::bind(&AvoidObsNode::DetectResultCallback, this, std::placeholders::_1)
        );

        // 接收 x,y,z,roll,pitch,yaw 信息, 每六个代表一个waypoint
        m_MoveTargetSub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/move_target", 1, std::bind(&AvoidObsNode::MoveTargetCallback, this, std::placeholders::_1)
        );

        m_EnableModelPub = this->create_publisher<std_msgs::msg::Bool>("/enable_model", 1);
    }

    ~AvoidObsNode(){}

    void MoveTargetCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        
        RCLCPP_INFO(get_logger(), "Move Target: %lu", msg->data.size());

        std_msgs::msg::Bool enable_msg;
        enable_msg.data = false;
        m_EnableModelPub->publish(enable_msg);

        // test: 先尝试直接用笛卡尔路径过去
        if(msg->data.size()%6!=0 or msg->data.size()<6){
            RCLCPP_ERROR(get_logger(), "Move Target Error: msg->data.size()%6!=0 and msg must contain at least 1 waypoints");
            return;
        }
        if(msg->data.size()==6){
            m_MoveitUtils.MoveTo(MoveItUtils::MakePoseMsg(msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5]));
            return;
        }
        std::vector<geometry_msgs::msg::Pose> waypoints;
        for(int i=0; i<msg->data.size(); i+=6){
            if(i==0){
                waypoints.push_back(MoveItUtils::MakePoseMsg(
                    msg->data[i], msg->data[i+1], msg->data[i+2], 
                    msg->data[i+3], msg->data[i+4], msg->data[i+5]
                ));
            }
            else{
                std::array<double,3> start_pos = {
                    waypoints.back().position.x,
                    waypoints.back().position.y,
                    waypoints.back().position.z
                };

                std::array<double,3> end_pos = {
                    msg->data[i], msg->data[i+1], msg->data[i+2]
                };
                auto midPoints = MakeWaypoints(start_pos,end_pos,msg->data[i+3], msg->data[i+4], msg->data[i+5], 0.03, 400);
                if(midPoints.empty()){
                    RCLCPP_ERROR(get_logger(), "Move Target Error: MakeWaypoints failed");
                    enable_msg.data = true;
                    m_EnableModelPub->publish(enable_msg);
                    return;
                }
                else{
                    waypoints.insert(waypoints.end(), midPoints.begin(), midPoints.end());
                    RCLCPP_WARN(get_logger(), "MakeWaypoints success, midPoints size: %d", midPoints.size());
                }
            }
        }
        // return;
        // m_MoveitUtils.RemoveCollision("giso");
        auto res = m_MoveitUtils.MoveTraj(waypoints,100,0.0,0.01,false);
        RCLCPP_WARN(get_logger(), "Move Traj Result: %f, points size: %d", res,waypoints.size());
        enable_msg.data = true;
        m_EnableModelPub->publish(enable_msg);

        // write to file
        m_ObsAvoidance.WriteToFile("./src/obs_avoid/config/obs.txt");
        WriteTrajToFile("./src/obs_avoid/config/traj.txt", waypoints);
    }

    void WriteTrajToFile(const std::string& filename, const std::vector<geometry_msgs::msg::Pose>& waypoints){
        std::ofstream ofs(filename);
        for(auto& p : waypoints){
            ofs << p.position.x << " " << p.position.y << " " << p.position.z << " " << p.orientation.x << " " << p.orientation.y << " " << p.orientation.z << " " << p.orientation.w << std::endl;
        }
        ofs.close();
    }

    std::vector<geometry_msgs::msg::Pose> MakeWaypoints(
        std::array<double,3> start_pos,
        std::array<double,3> end_pos,
        double roll, double pitch, double yaw,
        double dt=0.04,int num = 100
    ) {
        std::vector<geometry_msgs::msg::Pose> res;
        Eigen::Vector3f x, xd,target;
        x << start_pos[0], start_pos[1], start_pos[2];
        target << end_pos[0], end_pos[1], end_pos[2];
        bool bContour = false;
        for(int i=0;i<num-1;i++){
            
            xd = target - x;
            if(xd.norm() < 0.02)
                break;
            Eigen::Vector3f modulatedVel; 
            if(xd.norm() > 0.1){
                modulatedVel = m_ObsAvoidance.obsModulationEllipsoid(x, xd, bContour);
            }
            else{
                modulatedVel = xd;
            }
            x += modulatedVel*dt;
            res.push_back(MoveItUtils::MakePoseMsg(x[0],x[1],x[2],roll,pitch,yaw));
        }
        if( xd.norm() > 0.02){
            return {};
        }
        else{
            res.push_back(MoveItUtils::MakePoseMsg(target[0],target[1],target[2],roll,pitch,yaw));
        }
        return res;
    }

    void DetectResultCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "Detect Result: %lu", msg->data.size());
        if(msg->data.empty())return;
        if(m_AlreadySetCollision and msg->data[0]==0){
            return;
        }
        // 在场景中设置障碍物
        m_ObsAvoidance.clearObstacles();
        std::vector<std::shared_ptr<Collision>> obsArr;
        for(int i=1; i<msg->data.size(); i+=7){
            double x = msg->data[i];
            double y = msg->data[i+1];
            double z = msg->data[i+2];
            double radius = msg->data[i+3];
            double height = msg->data[i+4];
            auto world_pos = m_CamTrans.SelfToWorld(x,y,z,0,0,0);
            // RCLCPP_INFO(get_logger(), "cam_pos: %f, %f, %f", x, y, z);
            // RCLCPP_INFO(get_logger(), "world_pos: %f, %f, %f", world_pos[0], world_pos[1], world_pos[2]);
            auto arm_pos = m_ArmTrans.WorldToSelf(world_pos[0],world_pos[1],world_pos[2],0,0,0);
            // RCLCPP_INFO(get_logger(), "arm_pos: %f, %f, %f", arm_pos[0], arm_pos[1], arm_pos[2]);

            m_ObsAvoidance.addObstacle(
                std::vector<float>{
                    arm_pos[0], arm_pos[1], arm_pos[2],
                    radius+0.05,radius+0.05,height,
                    2,2,1
                }
            );

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
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr m_DetectResultSub,m_MoveTargetSub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_EnableModelPub;
    bool m_AlreadySetCollision = false;
    DSObstacleAvoidance m_ObsAvoidance;
};