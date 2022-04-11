#include <iostream>
#include <vector>

#include "environment.hpp"

Environment::Environment() = default;

void Environment::setObstacles(const std::vector<Obstacle> &obstacles) {
    mObstacles = obstacles;
}

void Environment::setObstacles(const ObstacleList* obstacleList) {
    mObstacles = {};
    mObstacles.reserve(obstacleList->numObstacles);

    for (int i = 0; i < obstacleList->numObstacles; ++i) {
        mObstacles.push_back(obstacleList->obstacles[i]);
    }
}

std::vector<Obstacle> & Environment::getObstacles() {
    return mObstacles;
}

TargetList Environment::getTargets() {
    return mTargets;
}

void Environment::setTargets(TargetList const& targets) {
    mTargets = targets;
}
