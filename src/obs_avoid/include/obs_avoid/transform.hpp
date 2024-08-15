#include <tuple>
#include <cmath>
#include <Eigen/Dense>
#include <array>

class Transform {
public:
    // 输入参数为当前坐标系相对于世界坐标系的位姿
    Transform(double x, double y, double z, double roll, double pitch, double yaw) {
        Reset(x, y, z, roll, pitch, yaw);
    }

    void Reset(double x, double y, double z, double roll, double pitch, double yaw) {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d Rx = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d Ry = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d Rz = Eigen::Matrix4d::Identity();

        Rx << 1, 0, 0, 0,
              0, cos(roll), -sin(roll), 0,
              0, sin(roll), cos(roll), 0,
              0, 0, 0, 1;

        Ry << cos(pitch), 0, sin(pitch), 0,
              0, 1, 0, 0,
              -sin(pitch), 0, cos(pitch), 0,
              0, 0, 0, 1;

        Rz << cos(yaw), -sin(yaw), 0, 0,
              sin(yaw), cos(yaw), 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;

        T = Rz * Ry * Rx;
        T.block<3, 1>(0, 3) << x, y, z;
        

        SelfToWorldMat = T;
    }

    // 将当前坐标系下的位姿转换为世界坐标系下的位姿
    std::array<double, 6> SelfToWorld(double x, double y, double z, double roll, double pitch, double yaw) {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d Rx = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d Ry = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d Rz = Eigen::Matrix4d::Identity();

        Rx << 1, 0, 0, 0,
              0, cos(roll), -sin(roll), 0,
              0, sin(roll), cos(roll), 0,
              0, 0, 0, 1;

        Ry << cos(pitch), 0, sin(pitch), 0,
              0, 1, 0, 0,
              -sin(pitch), 0, cos(pitch), 0,
              0, 0, 0, 1;

        Rz << cos(yaw), -sin(yaw), 0, 0,
              sin(yaw), cos(yaw), 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;

        T = Rz * Ry * Rx;
        T.block<3, 1>(0, 3) << x, y, z;
        

        Eigen::Matrix4d worldT = SelfToWorldMat * T;

        double worldX = worldT(0, 3);
        double worldY = worldT(1, 3);
        double worldZ = worldT(2, 3);

        Eigen::Matrix3d worldR = worldT.block<3, 3>(0, 0);
        double worldRoll = atan2(worldR(2, 1), worldR(2, 2));
        double worldPitch = atan2(-worldR(2, 0), sqrt(worldR(2, 1) * worldR(2, 1) + worldR(2, 2) * worldR(2, 2)));
        double worldYaw = atan2(worldR(1, 0), worldR(0, 0));

        return {worldX, worldY, worldZ, worldRoll, worldPitch, worldYaw};
    }

    // 将世界坐标系下的位姿转换为当前坐标系下的位姿
    std::array<double, 6> WorldToSelf(double x, double y, double z, double roll, double pitch, double yaw) {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d Rx = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d Ry = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d Rz = Eigen::Matrix4d::Identity();

        Rx << 1, 0, 0, 0,
              0, cos(roll), -sin(roll), 0,
              0, sin(roll), cos(roll), 0,
              0, 0, 0, 1;

        Ry << cos(pitch), 0, sin(pitch), 0,
              0, 1, 0, 0,
              -sin(pitch), 0, cos(pitch), 0,
              0, 0, 0, 1;

        Rz << cos(yaw), -sin(yaw), 0, 0,
              sin(yaw), cos(yaw), 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;

        T = Rz * Ry * Rx;
        T.block<3, 1>(0, 3) << x, y, z;
        

        Eigen::Matrix4d selfT = SelfToWorldMat.inverse() * T;

        double selfX = selfT(0, 3);
        double selfY = selfT(1, 3);
        double selfZ = selfT(2, 3);

        Eigen::Matrix3d selfR = selfT.block<3, 3>(0, 0);
        double selfRoll = atan2(selfR(2, 1), selfR(2, 2));
        double selfPitch = atan2(-selfR(2, 0), sqrt(selfR(2, 1) * selfR(2, 1) + selfR(2, 2) * selfR(2, 2)));
        double selfYaw = atan2(selfR(1, 0), selfR(0, 0));

        return {selfX, selfY, selfZ, selfRoll, selfPitch, selfYaw};
    }

private:
    Eigen::Matrix4d SelfToWorldMat;
};