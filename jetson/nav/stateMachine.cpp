#include "stateMachine.hpp"

#include <map>
#include <thread>
#include <utility>
#include <iostream>

#include "utilities.hpp"
#include "rover_msgs/NavStatus.hpp"
#include "gate_search/diamondGateSearch.hpp"

// Constructs a StateMachine object with the input lcm object.
// Reads the configuration file and constructs a Rover objet with this
// and the lcmObject. Sets mStateChanged to true so that on the first
// iteration of run the rover is updated.
StateMachine::StateMachine(
        rapidjson::Document& config,
        std::shared_ptr<Rover> rover, std::shared_ptr<Environment> env, std::shared_ptr<CourseProgress> courseProgress,
        lcm::LCM& lcmObject
) : mConfig(config), mRover(move(rover)), mEnv(move(env)), mCourseProgress(move(courseProgress)),
    mLcmObject(lcmObject) {
    mSearchStateMachine = SearchFactory(weak_from_this(), SearchType::FROM_PATH_FILE, mRover, mConfig);
    mGateStateMachine = GateFactory(weak_from_this(), mConfig);
} // StateMachine()

void StateMachine::setSearcher(SearchType type) {
    mSearchStateMachine = SearchFactory(weak_from_this(), type, mRover, mConfig);
    mSearchStateMachine->initializeSearch(mConfig, mConfig["computerVision"]["visionDistance"].GetDouble());
}

void StateMachine::setGateSearcher() {
    mGateStateMachine = GateFactory(weak_from_this(), mConfig);
}

// Runs the state machine through one iteration. The state machine will
// run if the state has changed or if the rover's status has changed.
// Will call the corresponding function based on the current state.
void StateMachine::run() {
    mPrevTimePoint = mTimePoint;
    auto now = std::chrono::high_resolution_clock::now();
    mTimePoint = now;

    static long i = 0;
    if (++i % 256 == 0) {
        std::cout << "Update rate: " << 1.0 / getDtSeconds() << std::endl;
    }

    mEnv->updateTargets(mRover, mCourseProgress);

    publishNavState();
    NavState nextState = NavState::Unknown;

    if (mRover->autonState().is_auton) {
        switch (mRover->currentState()) {
            case NavState::Off: {
                nextState = executeOff();
                break;
            }

            case NavState::Done: {
                nextState = executeDone();
                break;
            }

            case NavState::DriveWaypoints: {
                nextState = executeDrive();
                break;
            }

            case NavState::Search:
            case NavState::DriveToTarget: {
                nextState = mSearchStateMachine->run();
                break;
            }

            case NavState::TurnAroundObs:
            case NavState::SearchTurnAroundObs:
            case NavState::DriveAroundObs:
            case NavState::SearchDriveAroundObs: {
                nextState = mObstacleAvoidanceStateMachine->run();
                break;
            }

            case NavState::BeginSearch: {
                setSearcher(SearchType::FROM_PATH_FILE);
                nextState = NavState::Search;
                break;
            }

            case NavState::BeginGateSearch: {
                setGateSearcher();
                nextState = mGateStateMachine->run();
                break;
            }
            case NavState::GateMakePath:
            case NavState::GateTraverse: {
                nextState = mGateStateMachine->run();
                break;
            }

            case NavState::Unknown: {
                throw std::runtime_error("Entered unknown state.");
            }
        } // switch

        if (nextState != mRover->currentState()) {
            mRover->setState(nextState);
            mRover->bearingPid().reset();
        }
    } else {
        nextState = NavState::Off;
        mRover->setState(executeOff()); // turn off immediately
        if (nextState != mRover->currentState()) {
            mRover->setState(nextState);
        }
    }

        case NavState::Done: {
            nextState = executeDone();
            break;
        }


        case NavState::Turn: {
            nextState = executeTurn();
            break;
        }


        case NavState::Drive: {
            nextState = executeDrive();
            break;
        }


        case NavState::SearchFaceNorth:
        case NavState::SearchTurn:
        case NavState::SearchDrive:
        case NavState::TurnToTarget:
        case NavState::DriveToTarget: {
            nextState = mSearchStateMachine->run();
            break;
        }

        case NavState::ChangeSearchAlg: {
            double visionDistance = mConfig["computerVision"]["visionDistance"].GetDouble();
            setSearcher(SearchType::FROM_PATH_FILE, mRover, mConfig);

            mSearchStateMachine->initializeSearch(mConfig, visionDistance);
            nextState = NavState::SearchTurn;
            break;
        }

        case NavState::GateSpin:
        case NavState::GateSpinWait:
        case NavState::GateTurn:
        case NavState::GateDrive:
        case NavState::GateTurnToCentPoint:
        case NavState::GateDriveToCentPoint:
        case NavState::GateFace:
        case NavState::GateDriveThrough:
        case NavState::GateTurnToFarPost:
        case NavState::GateDriveToFarPost:
        case NavState::GateTurnToGateCenter: {
            nextState = mGateStateMachine->run();
            break;
        }

        case NavState::Unknown: {
            throw std::runtime_error("Entered unknown state.");
        }
    } // switch

    if (nextState != mRover->currentState()) {
        mRover->setState(nextState);
        mRover->bearingPid().reset();
    }
    std::cerr << std::flush;
} // run()

// Publishes the current navigation state to the nav status lcm channel.
void StateMachine::publishNavState() const {
    NavStatus navStatus{
            .nav_state_name = stringifyNavState(),
            .completed_wps = static_cast<int32_t>(mCourseProgress->getRemainingWaypoints().size()),
            .total_wps = mCourseProgress->getCourse().num_waypoints
    };
    const std::string& navStatusChannel = mConfig["lcmChannels"]["navStatusChannel"].GetString();
    mLcmObject.publish(navStatusChannel, &navStatus);
} // publishNavState()

// Executes the logic for off. If the rover is turned on, it updates
// the roverStatus. If the course is empty, the rover is done  with
// the course otherwise it will turn to the first waypoint. Else the
// rover is still off.
NavState StateMachine::executeOff() {
    if (mRover->autonState().is_auton) {
        NavState nextState = mCourseProgress->getCourse().num_waypoints ? NavState::DriveWaypoints : NavState::Done;
        mCourseProgress->clearProgress();
        return nextState;
    }
    mRover->stop();
    return NavState::Off;
} // executeOff()

// Executes the logic for the done state. Stops and turns off the rover.
NavState StateMachine::executeDone() {
    mRover->stop();
    return NavState::Done;
} // executeDone()

/**
 * Drive through the waypoints defined by course progress.
 * @return Next state
 */
NavState StateMachine::executeDrive() {
    Waypoint const& nextWaypoint = mCourseProgress->getRemainingWaypoints().front();
    // double distance = estimateNoneuclid(mRover->odometry(), nextWaypoint.odom);

//    if ((nextWaypoint.search || nextWaypoint.gate)
//        && mRover->leftCacheTarget().id == nextWaypoint.id
//        && distance <= mConfig["navThresholds"]["waypointRadius"].GetDouble()) {
//        return NavState::TurnToTarget;
//    }

    DriveStatus driveStatus = mRover->drive(mEnv,nextWaypoint.odom);

    if (driveStatus == DriveStatus::Arrived) {
        if (nextWaypoint.search || nextWaypoint.gate) {
            return NavState::ChangeSearchAlg;
        }
        mCourseProgress->completeCurrentWaypoint();
        std::cout << "Completed waypoint" << std::endl;
        if (currentWaypoint.search) {
            return NavState::BeginSearch;
        } else if (mCourseProgress->getRemainingWaypoints().empty()) {
            return NavState::Done;
        }
    }
    return NavState::DriveWaypoints;
} // executeDrive()

// Gets the string representation of a nav state.
std::string StateMachine::stringifyNavState() const {
    static const std::unordered_map<NavState, std::string> navStateNames =
            {
                    {NavState::Off,                  "Off"},
                    {NavState::Done,                 "Done"},
                    {NavState::DriveWaypoints,       "Drive Waypoints"},
                    {NavState::BeginSearch,          "Change Search Algorithm"},
                    {NavState::Search,               "Search"},
                    {NavState::DriveToTarget,        "Drive to Target"},
                    {NavState::GateSpin,             "Gate Spin"},
                    {NavState::GateSpinWait,         "Gate Spin Wait"},
                    {NavState::GateTurn,             "Gate Turn"},
                    {NavState::GateDrive,            "Gate Drive"},
                    {NavState::GateTurnToCentPoint,  "Gate Turn to Center Point"},
                    {NavState::GateDriveToCentPoint, "Gate Drive to Center Point"},
                    {NavState::GateFace,             "Gate Face"},
                    {NavState::GateTurnToFarPost,    "Gate Turn to Far Post"},
                    {NavState::GateDriveToFarPost,   "Gate Drive to Far Post"},
                    {NavState::GateTurnToGateCenter, "Gate Turn to Gate Center"},
                    {NavState::GateDriveThrough,     "Gate Drive Through"},

                    {NavState::Unknown,              "Unknown"}
            };

    return navStateNames.at(mRover->currentState());
} // stringifyNavState()

std::shared_ptr<Environment> StateMachine::getEnv() {
    return mEnv;
}

std::shared_ptr<CourseProgress> StateMachine::getCourseState() {
    return mCourseProgress;
}

std::shared_ptr<Rover> StateMachine::getRover() {
    return mRover;
}

lcm::LCM& StateMachine::getLCM() {
    return mLcmObject;
}

double StateMachine::getDtSeconds() {
    return std::chrono::duration<double>(mTimePoint - mPrevTimePoint).count();
}
