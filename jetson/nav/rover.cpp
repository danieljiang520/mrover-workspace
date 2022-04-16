#include "rover.hpp"
#include "utilities.hpp"
#include "rover_msgs/Joystick.hpp"

#include <vector>
#include <cmath>
#include <iostream>

// Constructs a rover object with the given configuration file and lcm
// object with which to use for communications.
Rover::Rover(const rapidjson::Document& config, lcm::LCM& lcmObject)
        : mConfig(config), mLcmObject(lcmObject),
          mBearingPid(config["bearingPid"]["kP"].GetDouble(),
                      config["bearingPid"]["kI"].GetDouble(),
<<<<<<< HEAD
                      config["bearingPid"]["kD"].GetDouble(), 360.0),
          mLongMeterInMinutes(-1) {
} // Rover(

/***
 * Drive to the global position defined by destination, turning if necessary.
 *
 * @param destination   Global destination
 * @param stopDistance  If we are this distance or closer, stop
 * @param dt            Delta time in seconds
 * @return              Whether we have reached the target
 */
bool Rover::drive(const Odometry& destination, double stopDistance, double dt) {
    double distance = estimateDistance(mOdometry, destination);
    double bearing = estimateBearing(mOdometry, destination);
    return drive(distance, bearing, stopDistance, dt);
} // drive()

bool Rover::drive(double distance, double bearing, double threshold, double dt) {
    if (distance < threshold) {
        return true;
    }

    if (turn(bearing, dt)) {
        double destinationBearing = mod(bearing, 360);
        double turningEffort = mBearingPid.update(mOdometry.bearing_deg, destinationBearing, dt);
        // When we drive to a target, we want to go as fast as possible so one of the sides is fixed at one and the other is 1 - abs(turningEffort)
        // if we need to turn clockwise, turning effort will be positive, so leftVel will be 1, and rightVel will be in between 0 and 1
        // if we need to turn ccw, turning effort will be negative, so rightVel will be 1 and leftVel will be in between 0 and 1
        // TODO: use std::clamp
        double leftVel = std::min(1.0, std::max(0.0, 1.0 + turningEffort));
        double rightVel = std::min(1.0, std::max(0.0, 1.0 - turningEffort));
        publishAutonDriveCmd(leftVel, rightVel);
    }

    return false;
} // drive()


/***
 *
 * @param destination
 * @param dt
 * @return
 */
bool Rover::turn(Odometry const& destination, double dt) {
    double bearing = estimateBearing(mOdometry, destination);
    return turn(bearing, dt);
} // turn()


bool Rover::turn(double absoluteBearing, double dt) {
    absoluteBearing = mod(absoluteBearing, 360);
    throughZero(absoluteBearing, mOdometry.bearing_deg);
    if (fabs(absoluteBearing - mOdometry.bearing_deg) <= mConfig["navThresholds"]["turningBearing"].GetDouble()) {
        return true;
    }
    double turningEffort = mBearingPid.update(mOdometry.bearing_deg, absoluteBearing, dt);
    // to turn in place we apply +turningEffort, -turningEffort on either side and make sure they're both within [-1, 1]
    double leftVel = std::max(std::min(1.0, +turningEffort), -1.0);
    double rightVel = std::max(std::min(1.0, -turningEffort), -1.0);
    publishAutonDriveCmd(leftVel, rightVel);
=======
                      config["bearingPid"]["kD"].GetDouble()),
          mLongMeterInMinutes(-1),
          mObstacleAvoider(config) {
} // Rover(

// Sends a joystick command to drive forward from the current odometry
// to the destination odometry. This joystick command will also turn
// the rover small amounts as "course corrections".
// The return value indicates if the rover has arrived or if it is
// on-course or off-course.
DriveStatus Rover::drive(std::shared_ptr<Environment> const& env, const Odometry& destination) {
    double distance = estimateNoneuclid(mOdometry, destination);
    //using obstacle avoidance bearing ->
    //double bearing = calcBearing(mOdometry, destination);
    ObstacleAvoidance::BearingDecision bearingDecision = mObstacleAvoider.getDesiredBearingDecision(env->getObstacles(), mOdometry, destination);
    double bearing = bearingDecision.desiredBearing;
    if (bearingDecision.obstacleControllerOutputState == NavState::Turn){
        //breakaway to turn state - we say we're off course and the turn controller will actually turn away from obstacle
        return DriveStatus::TurnFromObstacle;
    }
    return drive(distance, bearing, false);
} // drive()

// Sends a joystick command to drive forward from the current odometry
// in the direction of bearing. The distance is used to determine how
// quickly to drive forward. This joystick command will also turn the
// rover small amounts as "course corrections". target indicates
// if the rover is driving to a target rather than a waypoint and
// determines which distance threshold to use.
// The return value indicates if the rover has arrived or if it is
// on-course or off-course.
DriveStatus Rover::drive(const double distance, const double bearing, const bool target) {
    if ((!target && distance < mRoverConfig["navThresholds"]["waypointDistance"].GetDouble()) ||
        (target && distance < mRoverConfig["navThresholds"]["targetDistance"].GetDouble())) {
        return DriveStatus::Arrived;
    }

    double destinationBearing = mod(bearing, 360);
    throughZero(destinationBearing,
                mOdometry.bearing_deg); // will go off course if inside if because through zero not calculated

    if (fabs(destinationBearing - mOdometry.bearing_deg) <
        mRoverConfig["navThresholds"]["drivingBearing"].GetDouble()) {
        double turningEffort = mBearingPid.update(mOdometry.bearing_deg, destinationBearing);
        //When we drive to a target, we want to go as fast as possible so one of the sides is fixed at one and the other is 1 - abs(turningEffort)
        //if we need to turn clockwise, turning effort will be positive, so left_vel will be 1, and right_vel will be in between 0 and 1
        //if we need to turn ccw, turning effort will be negative, so right_vel will be 1 and left_vel will be in between 0 and 1
        // TODO: use std::clamp
        double left_vel = std::min(1.0, std::max(0.0, 1.0 + turningEffort));
        double right_vel = std::min(1.0, std::max(0.0, 1.0 - turningEffort));
        publishAutonDriveCmd(left_vel, right_vel);
        return DriveStatus::OnCourse;
    }
    std::cerr << "off course\n";
    return DriveStatus::OffCourse;
} // drive()

// Sends a joystick command to turn the rover toward the destination
// odometry. Returns true if the rover has finished turning, false
// otherwise.
bool Rover::turn(Odometry const& destination) {
    double bearing = calcBearing(mOdometry, destination);
    return turn(bearing);
} // turn()

// Sends a joystick command to turn the rover. The bearing is the
// absolute bearing. Returns true if the rover has finished turning, false
// otherwise.
bool Rover::turn(double bearing) {
    bearing = mod(bearing, 360);
    throughZero(bearing, mOdometry.bearing_deg);
    double turningBearingThreshold;
    turningBearingThreshold = mRoverConfig["navThresholds"]["turningBearing"].GetDouble();
    
    if (fabs(bearing - mOdometry.bearing_deg) <= turningBearingThreshold) {
        return true;
    }
    double turningEffort = mBearingPid.update(mOdometry.bearing_deg, bearing);
//    std::cout << "cur bearing: " << mOdometry.bearing_deg << " target bearing: " << bearing << " effort: " << turningEffort << std::endl;
    //to turn in place we apply +turningEffort, -turningEffort on either side and make sure they're both within [-1, 1]
    double left_vel = std::max(std::min(1.0, +turningEffort), -1.0);
    double right_vel = std::max(std::min(1.0, -turningEffort), -1.0);
//    std::cout << left_vel << ", " << right_vel << std::endl;
    publishAutonDriveCmd(left_vel, right_vel);
>>>>>>> ankith/obstacle-avoidance
    return false;
} // turn()

void Rover::stop() {
//    std::cout << "stopping" << std::endl;
    publishAutonDriveCmd(0.0, 0.0);
} // stop()

// Calculates the conversion from minutes to meters based on the
// rover's current latitude.
double Rover::longMeterInMinutes() const {
    if (mLongMeterInMinutes <= 0.0) {
        throw std::runtime_error("Invalid conversion");
    }
    return mLongMeterInMinutes;
}

// Gets the rover's turning pid object.
PidLoop& Rover::bearingPid() {
    return mBearingPid;
} // bearingPid()

void Rover::publishAutonDriveCmd(const double leftVel, const double rightVel) {
    AutonDriveControl driveControl{
            .left_percent_velocity = leftVel,
            .right_percent_velocity = rightVel
    };
    //std::cout << leftVel << " " << rightVel << std::endl;
    std::string autonDriveControlChannel = mConfig["lcmChannels"]["autonDriveControlChannel"].GetString();
    mLcmObject.publish(autonDriveControlChannel, &driveControl);
}

<<<<<<< HEAD
bool Rover::isTurningAroundObstacle(NavState currentState) {
    return currentState == NavState::TurnAroundObs || currentState == NavState::SearchTurnAroundObs;
} // isTurningAroundObstacle()
=======
void Rover::updateTargets(std::shared_ptr<Environment> const& env, std::shared_ptr<CourseProgress> const& course) {
    // TODO: I'm a little skeptical about how this function fits into the architecture.
    // TODO: It seems like it should be a part of the environment, not the rover.
    if (mAutonState.is_auton) {
        mTargetLeft = env->getTargets().targetList[0];
        mTargetRight = env->getTargets().targetList[1];
        // Cache Left Target if we had detected one
        if (mTargetLeft.distance != mRoverConfig["navThresholds"]["noTargetDist"].GetDouble()) {
            // Associate with single post
            if (mTargetLeft.id == course->getRemainingWaypoints().front().id) {
                mCountLeftHits++;
            } else {
                mCountLeftHits = 0;
            }
            // Update leftTarget if we have 3 or more consecutive hits
            if (mCountLeftHits >= 3) {
                mCacheTargetLeft = mTargetLeft;
                mCountLeftMisses = 0;
            }
            // Cache Right Target if we had detected one (only can see right if we see the left one, otherwise
            // results in some undefined behavior)
            if (mTargetRight.distance != mRoverConfig["navThresholds"]["noTargetDist"].GetDouble()) {
                mCacheTargetRight = mTargetRight;
                mCountRightMisses = 0;
            } else {
                mCountRightMisses++;
            }
        } else {
            mCountLeftMisses++;
            mCountRightMisses++; // need to increment since we don't see both
            mCountLeftHits = 0;
            mCountRightHits = 0;
        }

        // Check if we need to reset left cache
        if (mCountLeftMisses > mRoverConfig["navThresholds"]["cacheMissMax"].GetDouble()) {
            mCountLeftMisses = 0;
            mCountLeftHits = 0;
            // Set to empty target
            mCacheTargetLeft = {-1, 0, 0};
        }
        // Check if we need to reset right cache
        if (mCountRightMisses > mRoverConfig["navThresholds"]["cacheMissMax"].GetDouble()) {
            mCountRightMisses = 0;
            mCountRightHits = 0;
            // Set to empty target
            mCacheTargetRight = {-1, 0, 0};
        }
    } else {
        double cosine = cos(degreeToRadian(mOdometry.latitude_deg, mOdometry.latitude_min));
        mLongMeterInMinutes = 60 / (EARTH_CIRCUM * cosine / 360);
    }
}
>>>>>>> ankith/obstacle-avoidance

// Gets a reference to the rover's current navigation state.
NavState const& Rover::currentState() const {
    return mCurrentState;
} // currentState()

// Gets a reference to the rover's current auton state.
AutonState const& Rover::autonState() const {
    return mAutonState;
} // autonState()

// Gets a reference to the rover's current odometry information.
Odometry const& Rover::odometry() const {
    return mOdometry;
} // odometry()

void Rover::setAutonState(AutonState state) {
    mAutonState = state;
}

void Rover::setOdometry(const Odometry& odometry) {
    mOdometry = odometry;
}

void Rover::setState(NavState state) {
    mCurrentState = state;
}

void Rover::setLongMeterInMinutes(double l) {
    mLongMeterInMinutes = l;
}