#pragma once

#include <vector>

#include "rover_msgs/Obstacle.hpp"
#include "rover_msgs/TargetList.hpp"
#include "rover_msgs/ObstacleList.hpp"


using namespace rover_msgs;

class Environment {
private:
    std::vector<Obstacle> mObstacles;
    TargetList mTargets{};

public:
    Environment();

    std::vector<Obstacle> & getObstacles();

    void setObstacles(const std::vector<Obstacle> & obstacles);
    void setObstacles(const ObstacleList* obstacleList);

    TargetList getTargets();

    void setTargets(TargetList const& targets);
};