#pragma once

#include <memory>
#include <chrono>

#include <lcm/lcm-cpp.hpp>

#include "rover.hpp"
#include "environment.hpp"
#include "courseProgress.hpp"
#include "search/searchStateMachine.hpp"
#include "gate_search/gateStateMachine.hpp"
<<<<<<< HEAD
#include "obstacle_avoidance/simpleAvoidance.hpp"
=======
#include "environment.hpp"
#include "courseProgress.hpp"
>>>>>>> ankith/obstacle-avoidance


using namespace rover_msgs;
using namespace std::chrono_literals;
using time_point = std::chrono::high_resolution_clock::time_point;

static auto const LOOP_DURATION = 0.01s;

// This class implements the logic for the state machine for the
// autonomous navigation of the rover.
class StateMachine : public std::enable_shared_from_this<StateMachine> {
public:
    /*************************************************************************/
    /* Public Member Functions */
    /*************************************************************************/
    StateMachine(rapidjson::Document& config,
                 std::shared_ptr<Rover> rover, std::shared_ptr<Environment> env, std::shared_ptr<CourseProgress> courseProgress,
                 lcm::LCM& lcmObject);

    void run();

<<<<<<< HEAD
    void updateObstacleElements(double leftBearing, double rightBearing, double distance);

    void updateObstacleDistance(double distance);

    void setSearcher(SearchType type);

    void setGateSearcher();
=======
    void setSearcher(SearchType type, const std::shared_ptr<Rover>& rover, const rapidjson::Document& roverConfig);
>>>>>>> ankith/obstacle-avoidance

    std::shared_ptr<Environment> getEnv();

    std::shared_ptr<CourseProgress> getCourseState();

    std::shared_ptr<Rover> getRover();

<<<<<<< HEAD
    lcm::LCM& getLCM();

    double getDtSeconds();

=======
    bool pushNewRoverStatus();
    
>>>>>>> ankith/obstacle-avoidance
    /*************************************************************************/
    /* Public Member Variables */
    /*************************************************************************/
    // Gate State Machine instance
    std::shared_ptr<GateStateMachine> mGateStateMachine;

private:
    /*************************************************************************/
    /* Private Member Functions */
    /*************************************************************************/
    void publishNavState() const;

    NavState executeOff();

    NavState executeDone();

    NavState executeDrive();

    std::string stringifyNavState() const;

<<<<<<< HEAD
=======
    bool isWaypointReachable(double distance);

>>>>>>> ankith/obstacle-avoidance
    /*************************************************************************/
    /* Private Member Variables */
    /*************************************************************************/
    // Configuration file for the rover.
    rapidjson::Document& mConfig;

    // Rover object to do basic rover operations in the state machine.
    std::shared_ptr<Rover> mRover;

    std::shared_ptr<Environment> mEnv;

    std::shared_ptr<CourseProgress> mCourseProgress;

    // Lcm object for sending and receiving messages.
    lcm::LCM& mLcmObject;

    // Search pointer to control search states
    std::shared_ptr<SearchStateMachine> mSearchStateMachine;

<<<<<<< HEAD
    // Avoidance pointer to control obstacle avoidance states
    std::shared_ptr<ObstacleAvoidanceStateMachine> mObstacleAvoidanceStateMachine;

    time_point mTimePoint, mPrevTimePoint;
=======
>>>>>>> ankith/obstacle-avoidance
}; // StateMachine
