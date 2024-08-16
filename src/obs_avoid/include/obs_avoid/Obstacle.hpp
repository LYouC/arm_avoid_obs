//
// Created by lyouc on 24-8-16.
//

#ifndef OBSTACLE_H
#define OBSTACLE_H


#include <vector>
#include <iostream>
#include <cmath>
#include <fstream>
#include <Eigen/Dense>

struct Obstacle
{
    Eigen::Vector3f _a, _p, _x0;    // x0位置，a为尺寸，p为指数
    double _safetyFactor, _rho, _thR = 0.0;
    bool _tailEffect = false, _bContour = true;
};

class DSObstacleAvoidance
{
private:

    Obstacle _obs;
    std::vector<Obstacle> _obstacles;
    Eigen::Vector3f _modulatedVel;
    Eigen::Matrix3f _modulationMatrix, _rotationMatrix, _basisMatrix;
    std::vector<Eigen::Matrix3f> _basisMatrixes;
    double _gamma = 0.0;
    std::vector<double> _gammas;

public:

    DSObstacleAvoidance();

    void addObstacle(const std::vector<float>& obs_data);

    void setObstacle(Obstacle &obs);

    void addObstacle(Obstacle &obs);

    void addObstacles(std::vector<Obstacle> obstacles);

    void clearObstacles();

    Eigen::Vector3f obsModulationEllipsoid(Eigen::Vector3f x, Eigen::Vector3f xd, bool bContour);

private:

    void computeBasisMatrix(Eigen::Vector3f x);
    void computeBasisMatrix(Eigen::Vector3f x, int nbObj);

};



#endif //OBSTACLE_H
