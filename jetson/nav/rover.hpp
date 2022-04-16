#pragma once

#include <queue>
#include <memory>

#include <lcm/lcm-cpp.hpp>

#include "rover_msgs/AutonState.hpp"
#include "rover_msgs/Bearing.hpp"
#include "rover_msgs/Course.hpp"
#include "rover_msgs/Obstacle.hpp"
#include "rover_msgs/Odometry.hpp"
#include "rover_msgs/TargetList.hpp"
#include "rover_msgs/Waypoint.hpp"
#include "rover_msgs/AutonDriveControl.hpp"
#include "rover_msgs/SearchPoints.hpp"
#include "rapidjson/document.h"
#include "courseProgress.hpp"
#include "environment.hpp"
#include "pid.hpp"
#include "obstacle_avoidance/obstacleAvoidance.hpp"

using namespace rover_msgs;

// This class is the representation of the navigation states.
enum class NavState {
    // Base States
    Off = 0,
    Done = 1,

    DriveWaypoints = 10,

    // Search States
    BeginSearch = 22,
    Search = 20,

    // Target Found States
    DriveToTarget = 28,

<<<<<<< HEAD
    // Obstacle Avoidance States
    TurnAroundObs = 30,
    DriveAroundObs = 31,
    SearchTurnAroundObs = 32,
    SearchDriveAroundObs = 33,

    // Gate Search
    BeginGateSearch = 40,
    GateMakePath = 41,
    GateTraverse = 42,

=======
    // Gate Search States
    GateSpin = 40,
    GateSpinWait = 41,
    GateTurn = 42,
    GateDrive = 43,
    GateTurnToCentPoint = 44,
    GateDriveToCentPoint = 45,
    GateFace = 46,
    GateDriveThrough = 47,
    GateTurnToFarPost = 48,
    GateDriveToFarPost = 49,
    GateTurnToGateCenter = 50,
    
>>>>>>> ankith/obstacle-avoidance
    // Unknown State
    Unknown = 255
}; // AutonState

// This class is the representation of the drive status.
enum class DriveStatus {
    Arrived,
    OnCourse,
    OffCourse,
    TurnFromObstacle
}; // DriveStatus

// This class creates a Rover object which can perform operations that
// the real rover can perform.
class Rover {
public:
    Rover(const rapidjson::Document& config, lcm::LCM& lcm_in);

<<<<<<< HEAD
    bool drive(const Odometry& destination, double stopDistance, double dt);
=======
    DriveStatus drive(std::shared_ptr<Environment> const& env, const Odometry& destination);
>>>>>>> ankith/obstacle-avoidance

    bool drive(double distance, double bearing, double threshold, double dt);

    bool turn(Odometry const& destination, double dt);

    bool turn(double absoluteBearing, double dt);

    void stop();

    PidLoop& bearingPid();

    [[nodiscard]] double longMeterInMinutes() const;

    [[nodiscard]] NavState const& currentState() const;

    [[nodiscard]] AutonState const& autonState() const;

    [[nodiscard]] Odometry const& odometry() const;

    void setAutonState(AutonState state);

    void setOdometry(Odometry const& odometry);

<<<<<<< HEAD
=======
    void updateTargets(std::shared_ptr<Environment>const &env, std::shared_ptr<CourseProgress> const& course);

>>>>>>> ankith/obstacle-avoidance
    void setState(NavState state);

    void setLongMeterInMinutes(double LongMeterInMinutes);

private:
    /*************************************************************************/
    /* Private Member Functions */
    /*************************************************************************/
    void publishAutonDriveCmd(double leftVel, double rightVel);

    /*************************************************************************/
    /* Private Member Variables */
    /*************************************************************************/

    // A reference to the configuration file.
    const rapidjson::Document& mConfig;

    // A reference to the lcm object that will be used for
    // communicating with the actual rover and the base station.
    lcm::LCM& mLcmObject;

    // The pid loop for turning.
    PidLoop mBearingPid;

    // The conversion factor from arcminutes to meters. This is based
    // on the rover's current latitude.
    double mLongMeterInMinutes;

    // The rover's current navigation state.
    NavState mCurrentState{NavState::Off};

    // The rover's current auton state.
    AutonState mAutonState{};

    // The rover's current odometry information.
    Odometry mOdometry{};
<<<<<<< HEAD
=======

    // The rover's current target information from computer
    // vision.
    Target mTargetLeft{-1.0, 0.0, 0};
    Target mTargetRight{-1.0, 0.0, 0};

    // Cached Target
    // Left means left in the pixel space
    Target mCacheTargetLeft{-1.0, 0.0, 0};
    Target mCacheTargetRight{-1.0, 0.0, 0};

    // Count of misses with cache
    int mCountLeftMisses = 0;
    int mCountRightMisses = 0;

    // Count hits for avoiding FPs
    int mCountLeftHits = 0;
    int mCountRightHits = 0;

    //Obstacle Avoidance Manager
    ObstacleAvoidance mObstacleAvoider;
>>>>>>> ankith/obstacle-avoidance
};
