#include "rover_msgs/Obstacle.hpp"
#include "rover_msgs/Odometry.hpp"
#include "rover.hpp"
#include <vector>
#include <iostream>
#include <string>
#include <cassert>

using namespace std;

void run_tests();
void test_empty_clear_bearing_list();
void test_zero_zero_is_clear();
void test_zero_zero_not_clear_in_threshold();
void test_zero_zero_not_clear_not_in_threshold();
void test_zero_zero_not_clear_many_obs();
void test_zero_zero_not_clear_many_obs_in_threshold();
void test_zero_zero_not_clear_many_obs_not_in_threshold();


int main() {
    run_tests();
    return 0;
}

void run_tests() {
    cout << "Running getBearingDecision Tests"
    cout << "=====================================\n";

    test_empty_clear_bearing_list();
    cout << "test_empty_clear_bearing_list PASS\n";

    test_zero_zero_is_clear();
    cout << "test_zero_zero_is_clear PASS\n";

    test_zero_zero_not_clear_in_threshold();
    cout << "test_zero_zero_not_clear_in_threshold PASS\n";

    test_zero_zero_not_clear_not_in_threshold();
    cout << "test_zero_zero_not_clear_no_in_threshold PASS\n";

    test_zero_zero_is_clear_many_obs();
    cout << " test_zero_zero_is_clear_many_obs PASS\n";

    test_zero_zero_not_clear_many_obs_in_threshold();
    cout << "test_zero_zero_not_clear_many_obs_in_threshold PASS\n";

    test_zero_zero_not_clear_many_obs_not_in_threshold();
    cout << "test_zero_zero_not_clear_many_obs_not_in_threshold PASS\n";

    cout << "=====================================\n";
    cout << "ALL TESTS PASS\n";
    return;
}

void test_empty_clear_bearing_list() {
    vector<Obstacle> v;
    Odometry roverOdom = {38, 24.384, -110, 47.52, 30.0, 3};
    Odometry dest = {38, 24.484, -110, 47.52, 30.0, 3};
    BearingDecision output = getDesiredBearingDecision(v, roverOdom, dest );
    BearingDecision compare = {NavState::Turn, 111};
    assert(output.obstacleControllerOutputState == compare.obstacleControllerOutputState);
    assert(output.desiredBearing == compare.desiredBearing);
    return;
}

void test_zero_zero_is_clear() {
    vector<Obstacle> v;
    Odometry roverOdom = {38, 24.384, -110, 47.52, 30.0, 3};
    Odometry dest = {38, 24.484, -110, 47.52, 30.0, 3};
    Obstacle obs = {{-1, -1, 3}, {-1, -1, 3}};
    v.push_back(obs);
    BearingDecision output = getDesiredBearingDecision(v, roverOdom, dest );
    BearingDecision compare = {NavState::Drive, 0.0};
    assert(output.obstacleControllerOutputState == compare.obstacleControllerOutputState);
    assert(output.desiredBearing == compare.desiredBearing);
    return;
}

void test_zero_zero_not_clear_in_threshold() {
    vector<Obstacle> v;
    Odometry roverOdom = {38, 24.384, -110, 47.52, 30.0, 3};
    Odometry dest = {38, 24.484, -110, 47.52, 30.0, 3};
    Obstacle obs = {{-2, -2, 1}, {2, 2, 1}};
    v.push_back(obs);

    double idealBearingUnadjusted = getIdealDesiredBearing(roverOdom, dest, clearBearings);
    double adjustedIdealBearing = getLatencyAdjustedDesiredBearing(roverOdom, idealBearingUnadjusted);
    BearingDecision output = getDesiredBearingDecision(v, roverOdom, dest );
    BearingDecision compare = {NavState::Drive, adjustedIdealBearing};

    assert(output.obstacleControllerOutputState == compare.obstacleControllerOutputState);
    assert(output.desiredBearing == compare.desiredBearing);
    return;
}

void test_zero_zero_not_clear_not_in_threshold() {
    vector<Obstacle> v;
    Odometry roverOdom = {38, 24.384, -110, 47.52, 30.0, 3};
    Odometry dest = {38, 24.484, -110, 47.52, 30.0, 3};
    Obstacle obs = {{-2, -2, 30}, {2, 2, 30}};
    v.push_back(obs);

    double idealBearingUnadjusted = getIdealDesiredBearing(roverOdom, dest, clearBearings);
    double adjustedIdealBearing = getLatencyAdjustedDesiredBearing(roverOdom, idealBearingUnadjusted);
    BearingDecision output = getDesiredBearingDecision(v, roverOdom, dest);
    BearingDecision compare = {NavState::Drive, adjustedIdealBearing};

    assert(output.obstacleControllerOutputState == compare.obstacleControllerOutputState);
    assert(output.desiredBearing == compare.desiredBearing);
    return;
}

void test_zero_zero_is_clear_many_obs() {
    vector<Obstacle> v;
    Odometry roverOdom = {38, 24.384, -110, 47.52, 30.0, 3};
    Odometry dest = {38, 24.484, -110, 47.52, 30.0, 3};
    Obstacle obs1 = {{-5, -5, 4}, {-4, -4, 4}};
    Obstacle obs2 = {{-3, -3, 5}, {-2, -2, 5}};
    Obstacle obs3 = {{3, 3, 5}, {6, 6, 5}};
    Obstacle obs4 = {{4, 4, 2}, {5, 5, 2}};
    v.push_back(obs1);
    v.push_back(obs2);
    v.push_back(obs3);
    v.push_back(obs4);

    double idealBearingUnadjusted = getIdealDesiredBearing(roverOdom, dest, clearBearings);
    double adjustedIdealBearing = getLatencyAdjustedDesiredBearing(roverOdom, idealBearingUnadjusted);
    BearingDecision output = getDesiredBearingDecision(v, roverOdom, dest);
    BearingDecision compare = {NavState::Drive, adjustedIdealBearing};

    assert(output.obstacleControllerOutputState == compare.obstacleControllerOutputState);
    assert(output.desiredBearing == compare.desiredBearing);
    return;
}

void test_zero_zero_not_clear_many_obs_in_threshold() {
    vector<Obstacle> v;
    Odometry roverOdom = {38, 24.384, -110, 47.52, 30.0, 3};
    Odometry dest = {38, 24.484, -110, 47.52, 30.0, 3};
    Obstacle obs1 = {{-1, -1, 1}, {1, 1, 1.5}};
    Obstacle obs2 = {{-2, -2, 2}, {2, 3, 2.3}};
    Obstacle obs3 = {{-3, -2, 0.5}, {-3, 3, 0.1}};
    Obstacle obs4 = {{-1, -2, 2}, {-3, 3, 1}};
    v.push_back(obs1);
    v.push_back(obs2);
    v.push_back(obs3);
    v.push_back(obs4);

    double idealBearingUnadjusted = getIdealDesiredBearing(roverOdom, dest, clearBearings);
    double adjustedIdealBearing = getLatencyAdjustedDesiredBearing(roverOdom, idealBearingUnadjusted);
    BearingDecision output = getDesiredBearingDecision(v, roverOdom, dest);
    BearingDecision compare = {NavState::Drive, adjustedIdealBearing};

    assert(output.obstacleControllerOutputState == compare.obstacleControllerOutputState);
    assert(output.desiredBearing == compare.desiredBearing);
    return;
}

void test_zero_zero_not_clear_many_obs_not_in_threshold() {
    vector<Obstacle> v;
    Odometry roverOdom = {38, 24.384, -110, 47.52, 30.0, 3};
    Odometry dest = {38, 24.484, -110, 47.52, 30.0, 3};
    Obstacle obs1 = {{-1, -1, 10.4}, {1, 1, 10.4}};
    Obstacle obs2 = {{-2, -2, 11}, {2, 3, 11.3}};
    Obstacle obs3 = {{-3, -2, 14.1}, {-3, 3, 9}};
    Obstacle obs4 = {{-1, -2, 12}, {-3, 3, 17.6}};
    v.push_back(obs1);
    v.push_back(obs2);
    v.push_back(obs3);
    v.push_back(obs4);

    double idealBearingUnadjusted = getIdealDesiredBearing(roverOdom, dest, clearBearings);
    double adjustedIdealBearing = getLatencyAdjustedDesiredBearing(roverOdom, idealBearingUnadjusted);
    BearingDecision output = getDesiredBearingDecision(v, roverOdom, dest);
    BearingDecision compare = {NavState::Drive, adjustedIdealBearing};

    assert(output.obstacleControllerOutputState == compare.obstacleControllerOutputState);
    assert(output.desiredBearing == compare.desiredBearing);
    return;
}

