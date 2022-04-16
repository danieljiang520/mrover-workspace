#include "gateStateMachine.hpp"

#include <utility>
#include <iostream>

#include "utilities.hpp"
#include "environment.hpp"
#include "stateMachine.hpp"

using Eigen::Vector2d;

// Constructs a GateStateMachine object with mStateMachine
GateStateMachine::GateStateMachine(std::weak_ptr<StateMachine> stateMachine, const rapidjson::Document& roverConfig) :
        mStateMachine(move(stateMachine)),
        mConfig(roverConfig) {
}

GateStateMachine::~GateStateMachine() = default;

void GateStateMachine::updateGateTraversalPath() {
    //TODO: update the gatePath vector here with a path to go to
//    std::shared_ptr<Environment> env = mStateMachine.lock()->getEnv();
//    Odometry leftPost = env->getLeftPostLocation();
//    Odometry rightPost = env->getRightPostLocation();
}

// Execute loop through gate state machine.
NavState GateStateMachine::run() {
    std::shared_ptr<StateMachine> sm = mStateMachine.lock();
    std::shared_ptr<Environment> env = sm->getEnv();
    std::shared_ptr<Rover> rover = sm->getRover();

    publishGatePath();
    switch (rover->currentState()) {
        case NavState::BeginGateSearch: {
            mPath.clear();
            return NavState::GateMakePath;
        }
        case NavState::GateMakePath: {
//            mPath.push_back(createOdom(rover->odometry(), {4.0, 0.0}, rover));
//            mPath.push_back(createOdom(rover->odometry(), {4.0, 4.0}, rover));
            // if (env->areTargetFiltersReady()) {
            makeSpiderPath(rover, env);
//                makeDualSegmentPath(rover, env);
//                return NavState::GateTraverse;
            // } else {
            //    rover->stop();
            //    mPath.clear();
            //}
            //return NavState::GateMakePath;
        }
        case NavState::GateTraverse: {
            if (mPath.empty()) {
//                std::exit(1);
                return NavState::Done;
            } else {
                Odometry const& front = mPath.front();
                double dt = sm->getDtSeconds();
                if (rover->drive(front, mConfig["navThresholds"]["waypointDistance"].GetDouble(), dt)) {
                    mPath.pop_front();
                }
            }
            return NavState::GateTraverse;
        }
        default: {
            std::cerr << "Entered Unknown NavState in search state machine" << std::endl;
            return NavState::Unknown;
        }
    } // switch
}

<<<<<<< HEAD
void printPoint(Vector2d p) {
    std::cout << "Vec2D: (" << p.x() << " , " << p.y() << ")" << std::endl;
}
=======
// Perform spin search for a waypoint
NavState GateStateMachine::executeGateSpin() {
    auto rover = mStateMachine.lock()->getRover();
    // degrees to turn to before performing a search wait.
    double waitStepSize = mRoverConfig["search"]["searchWaitStepSize"].GetDouble();
    static double nextStop = 0; // to force the rover to wait initially
    static double mOriginalSpinAngle = 0; //initialize, is corrected on first call
    // double maximumPostWidth = mRoverConfig[ "navThresholds" ][ "maximumPostWidth" ].GetDouble();
>>>>>>> ankith/obstacle-avoidance

void GateStateMachine::makeDualSegmentPath(std::shared_ptr<Rover> const& rover, std::shared_ptr<Environment>& env) {
    Vector2d p1 = env->getPostOneOffsetInCartesian(rover->odometry());
    Vector2d p2 = env->getPostTwoOffsetInCartesian(rover->odometry());
    Vector2d v = p2 - p1;
    Vector2d m = p1 + v / 2;
    double driveDist = v.dot(m) / v.norm();
    double deltaBearing = radianToDegree(atan2(v.y(), v.x()));
    if (driveDist < 0.0) deltaBearing = deltaBearing - 180.0;
    double perpBearing = rover->odometry().bearing_deg + deltaBearing;
    double finalDriveDist = fabs(driveDist) + mConfig["navThresholds"]["waypointDistance"].GetDouble();
    Odometry perpOdometry = createOdom(rover->odometry(), perpBearing, finalDriveDist, rover);
    mPath.push_back(perpOdometry);
    double rotateBearing = perpBearing - (driveDist > 0 ? 105.0 : -105);
    Odometry throughOdometry = createOdom(perpOdometry, rotateBearing, m.norm() + 2.0, rover);
    mPath.push_back(throughOdometry);
}


void GateStateMachine::makeSpiderPath(std::shared_ptr<Rover> const& rover, std::shared_ptr<Environment>& env) {
    Vector2d p1 = env->getPostOneOffsetInCartesian(rover->odometry());
    Vector2d p2 = env->getPostTwoOffsetInCartesian(rover->odometry());
    Vector2d center = (p1 + p2) / 2;
    // TODO make this a constant
    double approachDistance = 2.0;
    Vector2d postDir = (p2 - p1).normalized();
    Vector2d perp = {-postDir.y(), postDir.x()};
    Vector2d approachPoints[2] = {(perp * approachDistance) + center,
                                  (perp * -approachDistance) + center};
    Vector2d prepPoints[4] = {(2 * approachDistance * perp) + p1,
                              (2 * approachDistance * perp) + p2,
                              (-2 * approachDistance * perp) + p1,
                              (-2 * approachDistance * perp) + p2};

    // TODO: add logic to go to farthest point along the path that doesn't collide with gate

    // find closest prep point
    double minNorm = -1.0;
    Vector2d prepPoint;
    for (auto& i: prepPoints) {
        double dist = i.norm();
        if (minNorm == -1.0 || dist < minNorm) {
            minNorm = dist;
            prepPoint = i;
        }
    }

<<<<<<< HEAD
    // find the closest approach point to prep point and set the other one as a victory point (we're through the gate)
    minNorm = -1.0;
    Vector2d approachPoint;
    Vector2d victoryPoint;
    double distance1 = (approachPoints[0] - prepPoint).norm();
    double distance2 = (approachPoints[1] - prepPoint).norm();
    if (distance1 < distance2) {
        approachPoint = approachPoints[0];
        victoryPoint = approachPoints[1];
=======
    if (!started) {
        rover->stop();
        startTime = time(nullptr);
        started = true;
    }
    double waitTime = mRoverConfig["search"]["searchWaitTime"].GetDouble();
    if (difftime(time(nullptr), startTime) > waitTime) {
        started = false;
        return NavState::GateSpin;
    }
    return NavState::GateSpinWait;
} // executeGateSpinWait()

// Turn to determined waypoint
NavState GateStateMachine::executeGateTurn() {
    auto rover = mStateMachine.lock()->getRover();
    if (mGateSearchPoints.empty()) {
        initializeSearch();
    }

    if (rover->rightCacheTarget().distance >= 0 ||
        (rover->leftCacheTarget().distance >= 0 && rover->leftCacheTarget().id != lastKnownRightPost.id)) {
        updatePost2Info();
        calcCenterPoint();
        return NavState::GateTurnToCentPoint;
    }

    Odometry& nextSearchPoint = mGateSearchPoints.front();
    if (rover->turn(nextSearchPoint)) {
        return NavState::GateDrive;
    }
    return NavState::GateTurn;
} // executeGateTurn()

// Drive to determined waypoint
NavState GateStateMachine::executeGateDrive() {
    auto rover = mStateMachine.lock()->getRover();
    if (rover->rightCacheTarget().distance >= 0 ||
        (rover->leftCacheTarget().distance >= 0 && rover->leftCacheTarget().id != lastKnownRightPost.id)) {
        updatePost2Info();
        calcCenterPoint();
        return NavState::GateTurnToCentPoint;
    }

    const Odometry& nextSearchPoint = mGateSearchPoints.front();
    DriveStatus driveStatus = rover->drive(mStateMachine.lock()->getEnv(), nextSearchPoint);

    if (driveStatus == DriveStatus::Arrived) {
        mGateSearchPoints.pop_front();
        return NavState::GateSpin;
    }
    if (driveStatus == DriveStatus::OnCourse) {
        return NavState::GateDrive;
    }
    return NavState::GateTurn;
} // executeGateDrive()

// Turn to center of the two gate posts
NavState GateStateMachine::executeGateTurnToCentPoint() {
    if (mStateMachine.lock()->getRover()->turn(centerPoint1)) {
        return NavState::GateDriveToCentPoint;
    }
    return NavState::GateTurnToCentPoint;
} // executeGateTurnToCentPoint()

// Drive to the center point defined by the two posts
NavState GateStateMachine::executeGateDriveToCentPoint() {
    DriveStatus driveStatus = mStateMachine.lock()->getRover()->drive(mStateMachine.lock()->getEnv(), centerPoint1);

    if (driveStatus == DriveStatus::Arrived) {
        return NavState::GateFace;
    }
    if (driveStatus == DriveStatus::OnCourse) {
        return NavState::GateDriveToCentPoint;
    }
    return NavState::GateTurnToCentPoint;
} // executeGateDriveToCentPoint()

// Turn to the face of the gate posts 
NavState GateStateMachine::executeGateFace() {
    if (mStateMachine.lock()->getRover()->turn(centerPoint2)) {
        return NavState::GateTurnToFarPost;
    }
    return NavState::GateFace;
} // executeGateFace()

// Turn to furthest post (or the only post if only one is available)
NavState GateStateMachine::executeGateTurnToFarPost() {
    auto rover = mStateMachine.lock()->getRover();
    if (rover->rightCacheTarget().distance > 0) {
        if (rover->leftCacheTarget().distance < rover->rightCacheTarget().distance) {
            if (rover->turn(rover->rightCacheTarget().bearing + rover->odometry().bearing_deg)) {
                return NavState::GateDriveToFarPost;
            }
        } else {
            if (rover->turn(rover->leftCacheTarget().bearing + rover->odometry().bearing_deg)) {
                return NavState::GateDriveToFarPost;
            }
        }
>>>>>>> ankith/obstacle-avoidance
    } else {
        approachPoint = approachPoints[1];
        victoryPoint = approachPoints[0];
    }
    Odometry cur = rover->odometry();
//    std::cout << prepPoint.x() << ", " << prepPoint.y() << " , " << approachPoint.x() << " , " << approachPoint.y()) << std::endl;
    Odometry prepOdom = createOdom(cur, prepPoint, rover);
    Odometry approachOdom = createOdom(cur, approachPoint, rover);
    Odometry victoryOdom = createOdom(cur, victoryPoint, rover);
    mPath.push_back(prepOdom);
    mPath.push_back(approachOdom);
    mPath.push_back(victoryOdom);

    printPoint(p1);
    printPoint(p2);
    printPoint(prepPoint);
    printPoint(approachPoint);
    printPoint(center);
    printPoint(victoryPoint);

<<<<<<< HEAD
    std::cout << "finished making path" << std::endl;
}

// Creates an GateStateMachine object
std::shared_ptr<GateStateMachine> GateFactory(const std::weak_ptr<StateMachine>& sm, const rapidjson::Document& roverConfig) {
    return std::make_shared<GateStateMachine>(sm, roverConfig);
} // GateFactory()

// Sends search path rover takes when trying to find posts
void GateStateMachine::publishGatePath() {
    // Construct vector from deque
    std::vector<Odometry> arr(mPath.begin(), mPath.end());
    SearchPoints gatePathPoints{
        .search_pattern_size  = (int32_t)arr.size(),
        .points = arr
    };

    std::string gatePathChannel = mConfig["lcmChannels"]["gatePathChannel"].GetString();
    mStateMachine.lock()->getLCM().publish(gatePathChannel, &gatePathPoints);

} // publishSearchPoints()
=======
    // Set to first target, since we should have atleast one in sight/detected
    double distance = rover->leftCacheTarget().distance - gateAdjustmentDist;
    double bearing = rover->leftCacheTarget().bearing + rover->odometry().bearing_deg;

    if (rover->rightCacheTarget().distance > 0 &&
        rover->leftCacheTarget().distance < rover->rightCacheTarget().distance) {
        // Set our variables to drive to target/post 2, which is farther away
        distance = rover->rightCacheTarget().distance - gateAdjustmentDist;
        bearing = rover->rightCacheTarget().bearing + rover->odometry().bearing_deg;
    }

    DriveStatus driveStatus = rover->drive(distance, bearing, true);

    if (driveStatus == DriveStatus::Arrived) {
        return NavState::GateTurnToGateCenter;
    }
    if (driveStatus == DriveStatus::OnCourse) {
        return NavState::GateDriveToFarPost;
    }
    return NavState::GateDriveToFarPost;
} // executeGateDriveToFarPost()

// Execute turn back to center point for driving through the gate
NavState GateStateMachine::executeGateTurnToGateCenter() {
    if (mStateMachine.lock()->getRover()->turn(centerPoint2)) {
        return NavState::GateDriveThrough;
    }
    return NavState::GateTurnToGateCenter;
} // executeGateTurnToGateCenter()

// Drive through gate posts
NavState GateStateMachine::executeGateDriveThrough() {
    DriveStatus driveStatus = mStateMachine.lock()->getRover()->drive(mStateMachine.lock()->getEnv(), centerPoint2);

    if (driveStatus == DriveStatus::Arrived) {
        if (!isCorrectGateDir) // Check if we drove through the incorrect direction
        {
            const Odometry temp = centerPoint1;
            centerPoint1 = centerPoint2;
            centerPoint2 = temp;
            isCorrectGateDir = true;
            return NavState::GateSpin;
        }
        mStateMachine.lock()->getCourseState()->completeCurrentWaypoint();
        return NavState::Turn;
    }
    if (driveStatus == DriveStatus::OnCourse) {
        return NavState::GateDriveThrough;
    }
    return NavState::GateDriveThrough;
} // executeGateDriveThrough()

// Update stored location and id for second post.
void GateStateMachine::updatePost2Info() {
    auto rover = mStateMachine.lock()->getRover();
    Odometry const& odometry = rover->odometry();
    if (rover->rightCacheTarget().distance >= 0 && rover->leftCacheTarget().id == lastKnownRightPost.id) {
        const double targetAbsAngle = mod(odometry.bearing_deg + rover->rightCacheTarget().bearing, 360);
        lastKnownLeftPost.odom = createOdom(odometry, targetAbsAngle, rover->rightCacheTarget().distance, rover);
        lastKnownLeftPost.id = rover->rightCacheTarget().id;
    } else {
        const double targetAbsAngle = mod(odometry.bearing_deg + rover->leftCacheTarget().bearing, 360);
        lastKnownLeftPost.odom = createOdom(odometry, targetAbsAngle, rover->leftCacheTarget().distance, rover);
        lastKnownLeftPost.id = rover->leftCacheTarget().id;
    }
} // updatePost2Info()

// Find the point centered in front of the gate.
// Find the angle that the rover should face from that point to face the gate.
// This point should be on the correct side of the gate so that we drive
// through it in the correct direction.
void GateStateMachine::calcCenterPoint() {
    auto rover = mStateMachine.lock()->getRover();
    const Odometry& currOdom = rover->odometry();
    const double distFromGate = 3;
    const double gateWidth = mStateMachine.lock()->getCourseState()->getRemainingWaypoints().front().gate_width;
    const double tagToPointAngle = radianToDegree(atan2(distFromGate, gateWidth / 2));
    const double gateAngle = calcBearing(lastKnownRightPost.odom, lastKnownLeftPost.odom);
    const double absAngle1 = mod(gateAngle + tagToPointAngle, 360);
    const double absAngle2 = mod(absAngle1 + 180, 360);
    const double tagToPointDist = sqrt(pow(gateWidth / 2, 2) + pow(distFromGate, 2));

    // Assuming that CV works well enough that we don't pass through the gate before
    // finding the second post. Thus, centerPoint1 will always be closer.
    centerPoint1 = createOdom(lastKnownRightPost.odom, absAngle1, tagToPointDist, rover);
    centerPoint2 = createOdom(lastKnownLeftPost.odom, absAngle2, tagToPointDist, rover);
    const double cp1Dist = estimateNoneuclid(currOdom, centerPoint1);
    const double cp2Dist = estimateNoneuclid(currOdom, centerPoint2);
    if (lastKnownRightPost.id % 2) {
        isCorrectGateDir = true;
    } else {
        isCorrectGateDir = false;
    }
    if (cp1Dist > cp2Dist) {
        const Odometry temp = centerPoint1;
        centerPoint1 = centerPoint2;
        centerPoint2 = temp;
        isCorrectGateDir = !isCorrectGateDir;
    }

} // calcCenterPoint()

// Creates an GateStateMachine object
std::shared_ptr<GateStateMachine> GateFactory(std::weak_ptr<StateMachine> stateMachine, const rapidjson::Document& roverConfig) {
    return std::make_shared<DiamondGateSearch>(stateMachine, roverConfig);
} // GateFactory()
>>>>>>> ankith/obstacle-avoidance
