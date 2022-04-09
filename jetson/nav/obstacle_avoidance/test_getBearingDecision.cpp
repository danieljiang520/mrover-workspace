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

// void run_tests();
void test_basic_clear_bearing(rapidjson::Document & mRoverConfig);
void test_empty_clear_bearing_list();
void test_zero_zero_is_clear();
void test_zero_zero_is_clear_many_obs();
void test_zero_zero_not_clear_in_threshold();
void test_zero_zero_not_clear_not_in_threshold();
void test_zero_zero_not_clear_many_obs();
void test_zero_zero_not_clear_many_obs_in_threshold();
void test_zero_zero_not_clear_many_obs_not_in_threshold();

void test_getIdealBearing(rapidjson::Document & mRoverConfig);
void run_tests(rapidjson::Document & mRoverConfig);

// int main() {
//     run_tests();
//     return 0;
// }

void run_tests(rapidjson::Document & mRoverConfig) {
    cout << "Running getBearingDecision Tests\n";
    cout << "=====================================\n";

    test_basic_clear_bearing(mRoverConfig);
    cout << "test_basic_clear_bearing PASS\n";

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

void test_basic_clear_bearing(rapidjson::Document & mRoverConfig) {
    ObstacleAvoidance oa(mRoverConfig);
    vector<Obstacle> v;
    Odometry roverOdom = {38, 24.384, -110, 47.52, 30.0, 3};
    Odometry dest = {38, 24.484, -110, 47.52, 30.0, 3};
    Obstacle obs = {{1, 0, 1}, {2, 1, 2}};
    // Obstacle obs = {{-2, 0, 3}, {2, 1, 4}};
    v.push_back(obs);
    vector<double> clearBearings = oa.test_getClearBearings(v);
    cout << "Clear bearings (" << clearBearings.size() << ")\n";
    for (double i : clearBearings) {
        cout << i << ", ";
    }
    cout << "\n";
}

void test_empty_clear_bearing_list() {
    rapidjson::Document mRoverConfig;
    ifstream configFile;
    string configPath = getenv("MROVER_CONFIG");
    configPath += "/config_nav/config.json";
    configFile.open( configPath );
    string config = "";
    string setting;
    while( configFile >> setting )
    {
        config += setting;
    }
    configFile.close();
    mRoverConfig.Parse( config.c_str() );
    ObstacleAvoidance oa(mRoverConfig);
    vector<Obstacle> v;
    Odometry roverOdom = {38, 24.384, -110, 47.52, 30.0, 3};
    Odometry dest = {38, 24.484, -110, 47.52, 30.0, 3};
    ObstacleAvoidance::BearingDecision output = oa.getDesiredBearingDecision(v, roverOdom, dest );
    ObstacleAvoidance::BearingDecision compare = {NavState::Turn, 111};
    assert(output.obstacleControllerOutputState == compare.obstacleControllerOutputState);
    assert(output.desiredBearing == compare.desiredBearing);
    return;
}

void test_zero_zero_is_clear() {
    rapidjson::Document mRoverConfig;
    ifstream configFile;
    string configPath = getenv("MROVER_CONFIG");
    configPath += "/config_nav/config.json";
    configFile.open( configPath );
    string config = "";
    string setting;
    while( configFile >> setting )
    {
        config += setting;
    }
    configFile.close();
    mRoverConfig.Parse( config.c_str() );
    ObstacleAvoidance oa(mRoverConfig);
    vector<Obstacle> v;
    Odometry roverOdom = {38, 24.384, -110, 47.52, 30.0, 3};
    Odometry dest = {38, 24.484, -110, 47.52, 30.0, 3};
    Obstacle obs = {{-1, -1, 3}, {-1, -1, 4}};
    v.push_back(obs);
    ObstacleAvoidance::BearingDecision output = oa.getDesiredBearingDecision(v, roverOdom, dest );
    ObstacleAvoidance::BearingDecision compare = {NavState::Drive, 0.0};
    assert(output.obstacleControllerOutputState == compare.obstacleControllerOutputState);
    assert(output.desiredBearing == compare.desiredBearing);
    return;
}

void test_zero_zero_not_clear_in_threshold() {
    rapidjson::Document mRoverConfig;
    ifstream configFile;
    string configPath = getenv("MROVER_CONFIG");
    configPath += "/config_nav/config.json";
    configFile.open( configPath );
    string config = "";
    string setting;
    while( configFile >> setting )
    {
        config += setting;
    }
    configFile.close();
    mRoverConfig.Parse( config.c_str() );
    ObstacleAvoidance oa(mRoverConfig);
    vector<Obstacle> v;
    Odometry roverOdom = {38, 24.384, -110, 47.52, 30.0, 3};
    Odometry dest = {38, 24.484, -110, 47.52, 30.0, 3};
    Obstacle obs = {{-2, -2, 1}, {2, 2, 2}};
    v.push_back(obs);
    vector<double> clearBearings = oa.test_getClearBearings(v);
    double idealBearingUnadjusted = oa.test_getIdealDesiredBearing(roverOdom, dest, clearBearings);
    double adjustedIdealBearing = oa.test_getLatencyAdjustedDesiredBearing(roverOdom, idealBearingUnadjusted);
    ObstacleAvoidance::BearingDecision output = oa.getDesiredBearingDecision(v, roverOdom, dest );
    ObstacleAvoidance::BearingDecision compare = {NavState::Drive, adjustedIdealBearing};

    assert(output.obstacleControllerOutputState == compare.obstacleControllerOutputState);
    assert(output.desiredBearing == compare.desiredBearing);
    return;
}

void test_zero_zero_not_clear_not_in_threshold() {
    rapidjson::Document mRoverConfig;
    ifstream configFile;
    string configPath = getenv("MROVER_CONFIG");
    configPath += "/config_nav/config.json";
    configFile.open( configPath );
    string config = "";
    string setting;
    while( configFile >> setting )
    {
        config += setting;
    }
    configFile.close();
    mRoverConfig.Parse( config.c_str() );
    ObstacleAvoidance oa(mRoverConfig);
    vector<Obstacle> v;
    Odometry roverOdom = {38, 24.384, -110, 47.52, 30.0, 3};
    Odometry dest = {38, 24.484, -110, 47.52, 30.0, 3};
    Obstacle obs = {{-2, -2, 30}, {2, 2, 30}};
    v.push_back(obs);
    vector<double> clearBearings = oa.test_getClearBearings(v);
    double idealBearingUnadjusted = oa.test_getIdealDesiredBearing(roverOdom, dest, clearBearings);
    double adjustedIdealBearing = oa.test_getLatencyAdjustedDesiredBearing(roverOdom, idealBearingUnadjusted);
    ObstacleAvoidance::BearingDecision output = oa.getDesiredBearingDecision(v, roverOdom, dest);
    ObstacleAvoidance::BearingDecision compare = {NavState::Drive, adjustedIdealBearing};

    assert(output.obstacleControllerOutputState == compare.obstacleControllerOutputState);
    assert(output.desiredBearing == compare.desiredBearing);
    return;
}

void test_zero_zero_is_clear_many_obs() {
    rapidjson::Document mRoverConfig;
    ifstream configFile;
    string configPath = getenv("MROVER_CONFIG");
    configPath += "/config_nav/config.json";
    configFile.open( configPath );
    string config = "";
    string setting;
    while( configFile >> setting )
    {
        config += setting;
    }
    configFile.close();
    mRoverConfig.Parse( config.c_str() );
    ObstacleAvoidance oa(mRoverConfig);
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
    vector<double> clearBearings = oa.test_getClearBearings(v);
    double idealBearingUnadjusted = oa.test_getIdealDesiredBearing(roverOdom, dest, clearBearings);
    double adjustedIdealBearing = oa.test_getLatencyAdjustedDesiredBearing(roverOdom, idealBearingUnadjusted);
    ObstacleAvoidance::BearingDecision output = oa.getDesiredBearingDecision(v, roverOdom, dest);
    ObstacleAvoidance::BearingDecision compare = {NavState::Drive, adjustedIdealBearing};

    assert(output.obstacleControllerOutputState == compare.obstacleControllerOutputState);
    assert(output.desiredBearing == compare.desiredBearing);
    return;
}

void test_zero_zero_not_clear_many_obs_in_threshold() {
    rapidjson::Document mRoverConfig;
    ifstream configFile;
    string configPath = getenv("MROVER_CONFIG");
    configPath += "/config_nav/config.json";
    configFile.open( configPath );
    string config = "";
    string setting;
    while( configFile >> setting )
    {
        config += setting;
    }
    configFile.close();
    mRoverConfig.Parse( config.c_str() );
    ObstacleAvoidance oa(mRoverConfig);
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
    vector<double> clearBearings = oa.test_getClearBearings(v);
    double idealBearingUnadjusted = oa.test_getIdealDesiredBearing(roverOdom, dest, clearBearings);
    double adjustedIdealBearing = oa.test_getLatencyAdjustedDesiredBearing(roverOdom, idealBearingUnadjusted);
    ObstacleAvoidance::BearingDecision output = oa.getDesiredBearingDecision(v, roverOdom, dest);
    ObstacleAvoidance::BearingDecision compare = {NavState::Drive, adjustedIdealBearing};

    assert(output.obstacleControllerOutputState == compare.obstacleControllerOutputState);
    assert(output.desiredBearing == compare.desiredBearing);
    return;
}

void test_zero_zero_not_clear_many_obs_not_in_threshold() {
    rapidjson::Document mRoverConfig;
    ifstream configFile;
    string configPath = getenv("MROVER_CONFIG");
    configPath += "/config_nav/config.json";
    configFile.open( configPath );
    string config = "";
    string setting;
    while( configFile >> setting )
    {
        config += setting;
    }
    configFile.close();
    mRoverConfig.Parse( config.c_str() );
    ObstacleAvoidance oa(mRoverConfig);
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
    vector<double> clearBearings = oa.test_getClearBearings(v);
    double idealBearingUnadjusted = oa.test_getIdealDesiredBearing(roverOdom, dest, clearBearings);
    double adjustedIdealBearing = oa.test_getLatencyAdjustedDesiredBearing(roverOdom, idealBearingUnadjusted);
    ObstacleAvoidance::BearingDecision output = oa.getDesiredBearingDecision(v, roverOdom, dest);
    ObstacleAvoidance::BearingDecision compare = {NavState::Drive, adjustedIdealBearing};

    assert(output.obstacleControllerOutputState == compare.obstacleControllerOutputState);
    assert(output.desiredBearing == compare.desiredBearing);
    return;
}

