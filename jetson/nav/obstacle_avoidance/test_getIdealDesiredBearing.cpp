#include "rover_msgs/Obstacle.hpp"
#include "rover_msgs/Odometry.hpp"
#include "obstacleAvoidance.hpp"
#include "rapidjson/document.h"
#include "rover.hpp"
#include <vector>
#include <iostream>
#include <string>
#include <cassert>

#include "test_suites.hpp"

using namespace std;

// void run_ideal_bearing_test_suite(rapidjson::Document & mRoverConfig);
void test_1();
void test_basic(rapidjson::Document & mRoverConfig);

void run_ideal_bearing_test_suite(rapidjson::Document & mRoverConfig) {
    // cout << "Running test suite"
    // test_1();
    test_basic(mRoverConfig);

}

void test_1() {
    cout << "Running the actual test 1\n";
    assert(true);
}

void test_basic(rapidjson::Document & mRoverConfig) {
    cout << "Running test_basic\n";

    ObstacleAvoidance oa(mRoverConfig);

    // getIdealDesiredBearing(Odometry roverOdom, Odometry dest, std::vector<double> clearBearings)

    
    vector<Obstacle> v;
    Odometry roverOdom = {38, 24.384, -110, 47.52, 30.0, 3};
    Odometry dest = {38, 24.484, -110, 47.52, 30.0, 3};
    Obstacle obs = {{-2, -2, 1}, {2, 2, 1}};
    v.push_back(obs);
    vector<double> clearBearings = oa.test_getClearBearings(v);
    clearBearings = {-55.0, -54.0, -53.0, -52.0, -51.0, -50.0, 10, 11, 12, 13, 14, 15, 16};
    double idealBearingUnadjusted = oa.test_getIdealDesiredBearing(roverOdom, dest, clearBearings);
    double adjustedIdealBearing = oa.test_getLatencyAdjustedDesiredBearing(roverOdom, idealBearingUnadjusted);
    cout << "From ideal " << idealBearingUnadjusted << " to adjusted " << adjustedIdealBearing << "\n";
    cout << "Clear bearings of size " << clearBearings.size() << "\n";
    for (const double & a : clearBearings) {
        cout << a << ", ";
    }
    cout << "\n";
    ObstacleAvoidance::BearingDecision output = oa.getDesiredBearingDecision(v, roverOdom, dest );
    ObstacleAvoidance::BearingDecision compare = {NavState::Drive, adjustedIdealBearing};

    // assert(true);
    assert(output.obstacleControllerOutputState == compare.obstacleControllerOutputState);
    assert(output.desiredBearing == compare.desiredBearing);

    cout << "Passed test_basic\n";
    return;
}
