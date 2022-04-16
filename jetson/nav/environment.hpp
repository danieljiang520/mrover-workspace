#pragma once

#include <vector>
#include <optional>

#include <eigen3/Eigen/Core>

#include "rover.hpp"
#include "filter.hpp"
#include "rover_msgs/Obstacle.hpp"
#include "rover_msgs/TargetList.hpp"
#include "rover_msgs/ObstacleList.hpp"

using Eigen::Vector2d;
using namespace rover_msgs;

class Rover;

class Environment {
private:
    std::vector<Obstacle> mObstacles;
    TargetList mTargets{};

public:
    explicit Environment(const rapidjson::Document& config);

    std::vector<Obstacle> & getObstacles();

    void setObstacles(const std::vector<Obstacle> & obstacles);
    void setObstacles(const ObstacleList* obstacleList);

    void setBaseGateID(int b);

    int getBaseGateID();

    Target getLeftTarget();

    Target getRightTarget();

    Odometry getPostOneLocation();

    Odometry getPostTwoLocation();

    Vector2d getPostOneOffsetInCartesian(Odometry cur);

    Vector2d getPostTwoOffsetInCartesian(Odometry cur);

    void setTargets(TargetList const& targets);

    void updateTargets(std::shared_ptr<Rover> const& rover, std::shared_ptr<CourseProgress> const& course);

    [[nodiscard]] bool hasGateLocation() const;

    [[nodiscard]] bool hasPostOneLocation() const;

    [[nodiscard]] bool hasPostTwoLocation() const;

    [[nodiscard]] bool areTargetFiltersReady() const;

    [[nodiscard]] bool isLeftTargetFilterReady() const;

    [[nodiscard]] bool isRightTargetFilterReady() const;

    std::optional<Target> tryGetTargetWithId(int32_t id);
};