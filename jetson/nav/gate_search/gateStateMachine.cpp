#include "gateStateMachine.hpp"

#include "utilities.hpp"
#include "stateMachine.hpp"
#include "./gate_search/diamondGateSearch.hpp"
#include <cmath>
#include <iostream>

// Constructs a GateStateMachine object with roverStateMachine
GateStateMachine::GateStateMachine( StateMachine* stateMachine, Rover* rover, const rapidjson::Document& roverConfig )
    : mRoverStateMachine( stateMachine )
    , mRoverConfig( roverConfig )
    , mRover( rover ) {}

GateStateMachine::~GateStateMachine() {}

// Execute loop through gate state machine.
NavState GateStateMachine::run()
{
    double maximumPostWidth = mRoverConfig[ "navThresholds" ][ "maximumPostWidth" ].GetDouble();
    if (distanceBetweenPosts() > maximumPostWidth) {

        // Invalid posts were found, so resort to spinning
        std::cout << "Gate posts should not be greater than " << maximumPostWidth << 
            " (at " << distanceBetweenPosts() << ") meters apart!" << std::endl;
        return executeGateSpin();
    }
    
    switch ( mRover->roverStatus().currentState() )
    {
        case NavState::GateSpin:
        {
            return executeGateSpin();
        }

        case NavState::GateSpinWait:
        {
            return executeGateSpinWait();
        }

        case NavState::GateTurn:
        {
            return executeGateTurn();
        }

        case NavState::GateDrive:
        {
            return executeGateDrive();
        }

        case NavState::GateTurnToCentPoint:
        {
            return executeGateTurnToCentPoint();
        }

        case NavState::GateFace:
        {
            return executeGateFace();
        }

        case NavState::GateDriveToCentPoint:
        {
            return executeGateDriveToCentPoint();
        }

        case NavState::GateTurnToFarPost:
        {
            return executeGateTurnToFarPost();
        }

        case NavState::GateDriveToFarPost:
        {
            return executeGateDriveToFarPost();
        }

        case NavState::GateTurnToGateCenter:
        {
            return executeGateTurnToGateCenter();
        }

        case NavState::GateDriveThrough:
        {
            return executeGateDriveThrough();
        }

        case NavState::GateTurnToPerpPoint:
        {
            
            return executeGateTurnToPerpPoint();
        }

        case NavState::GateDriveToPerpPoint:
        {

            return executeGateDriveToPerpPoint();
        }

        case NavState::GateTurnToDivePoint:
        {

            return executeGateTurnToDivePoint();
        }

        case NavState::GateDriveToDivePoint:
        {

            return executeGateDriveToDivePoint();
        }

        case NavState::GateTurnToAcrossPoint:
        {

            return executeGateTurnToAcrossPoint;
        }

        case NavState::GateDriveToAcrossPoint:
        {

            return executeGateDriveToAcrossPoint;
        }    



        default:
        {
            cerr << "Entered Unknown NavState in search state machine" << endl;
            return NavState::Unknown;
        }
    } // switch
} // run

// Perform spin search for a waypoint
NavState GateStateMachine::executeGateSpin()
{
    // degrees to turn to before performing a search wait.
    double waitStepSize = mRoverConfig[ "search" ][ "searchWaitStepSize" ].GetDouble();
    static double nextStop = 0; // to force the rover to wait initially
    static double mOriginalSpinAngle = 0; //initialize, is corrected on first call
    double maximumPostWidth = mRoverConfig[ "navThresholds" ][ "maximumPostWidth" ].GetDouble();

    if( (mRover->roverStatus().rightCacheTarget().distance >= 0 ||
        mRover->roverStatus().leftCacheTarget().distance >= 0)
        && mRover->roverStatus().leftCacheTarget().id != lastKnownRightPost.id
        && distanceBetweenPosts() <= maximumPostWidth)
    {
        mRover->roverStatus().getLeftMisses() = 0; // reset
        mRover->roverStatus().getRightMisses() = 0; // reset
        updatePost2Info();
        calcCenterPoint();
        return NavState::GateTurnToCentPoint;
    }

    if( nextStop == 0 )
    {
        // get current angle and set as origAngle
        mOriginalSpinAngle = mRover->roverStatus().odometry().bearing_deg; //doublecheck
        nextStop = mOriginalSpinAngle;
    }
    if( mRover->turn( nextStop ) )
    {
        if( nextStop - mOriginalSpinAngle >= 360 )
        {
            nextStop = 0;
            return NavState::GateTurn;
        }
        nextStop += waitStepSize;
        return NavState::GateSpinWait;
    }
    return NavState::GateSpin;
} // executeGateSpin()

// Wait for predetermined time before performing GateSpin
NavState GateStateMachine::executeGateSpinWait()
{
    static bool started = false;
    static time_t startTime;

    if( mRover->roverStatus().rightCacheTarget().distance >= 0 ||
        ( mRover->roverStatus().leftCacheTarget().distance >= 0 && mRover->roverStatus().leftCacheTarget().id != lastKnownRightPost.id ) )
    {
        updatePost2Info();
        calcCenterPoint();
        return NavState::GateTurnToCentPoint;
    }

    if( !started )
    {
        mRover->stop();
        startTime = time( nullptr );
        started = true;
    }
    double waitTime = mRoverConfig[ "search" ][ "searchWaitTime" ].GetDouble();
    if( difftime( time( nullptr ), startTime ) > waitTime )
    {
        started = false;
        return NavState::GateSpin;
    }
    return NavState::GateSpinWait;
} // executeGateSpinWait()

// Turn to determined waypoint
NavState GateStateMachine::executeGateTurn()
{
    if( mGateSearchPoints.empty() )
    {
        initializeSearch();
    }

    if( mRover->roverStatus().rightCacheTarget().distance >= 0 ||
        ( mRover->roverStatus().leftCacheTarget().distance >= 0 && mRover->roverStatus().leftCacheTarget().id != lastKnownRightPost.id ) )
    {
        updatePost2Info();
        calcCenterPoint();
        return NavState::GateTurnToCentPoint;
    }

    Odometry& nextSearchPoint = mGateSearchPoints.front();
    if( mRover->turn( nextSearchPoint ) )
    {
        return NavState::GateDrive;
    }
    return NavState::GateTurn;
} // executeGateTurn()

// Drive to determined waypoint
NavState GateStateMachine::executeGateDrive()
{
    if( mRover->roverStatus().rightCacheTarget().distance >= 0 ||
        ( mRover->roverStatus().leftCacheTarget().distance >= 0 && mRover->roverStatus().leftCacheTarget().id != lastKnownRightPost.id ) )
    {
        updatePost2Info();
        calcCenterPoint();
        return NavState::GateTurnToCentPoint;
    }

    const Odometry& nextSearchPoint = mGateSearchPoints.front();
    DriveStatus driveStatus = mRover->drive( nextSearchPoint );

    if( driveStatus == DriveStatus::Arrived )
    {
        mGateSearchPoints.pop_front();
        return NavState::GateSpin;
    }
    if( driveStatus == DriveStatus::OnCourse )
    {
        return NavState::GateDrive;
    }
    return NavState::GateTurn;
} // executeGateDrive()

// Turn to center of the two gate posts
NavState GateStateMachine::executeGateTurnToCentPoint()
{
    if( mRover->turn( centerPoint1 ) )
    {
        return NavState::GateDriveToCentPoint;
    }
    return NavState::GateTurnToCentPoint;
} // executeGateTurnToCentPoint()

// Drive to the center point defined by the two posts
NavState GateStateMachine::executeGateDriveToCentPoint()
{
    DriveStatus driveStatus = mRover->drive( centerPoint1 );

    if( driveStatus == DriveStatus::Arrived )
    {
        return NavState::GateFace;
    }
    if( driveStatus == DriveStatus::OnCourse )
    {
        return NavState::GateDriveToCentPoint;
    }
    return NavState::GateTurnToCentPoint;
} // executeGateDriveToCentPoint()

// Turn to the face of the gate posts 
NavState GateStateMachine::executeGateFace()
{
    if( mRover->turn( centerPoint2 ) )
    {
        return NavState::GateTurnToFarPost;
    }
    return NavState::GateFace;
} // executeGateFace()

// Turn to furthest post (or the only post if only one is available)
NavState GateStateMachine::executeGateTurnToFarPost()
{
    if( mRover->roverStatus().rightCacheTarget().distance > 0 ) 
    {
        if( mRover->roverStatus().leftCacheTarget().distance < mRover->roverStatus().rightCacheTarget().distance ) 
        {
            if( mRover->turn( mRover->roverStatus().rightCacheTarget().bearing + mRover->roverStatus().odometry().bearing_deg ) )
            {
                return NavState::GateDriveToFarPost;
            }
        }
        else 
        {
            if( mRover->turn( mRover->roverStatus().leftCacheTarget().bearing + mRover->roverStatus().odometry().bearing_deg ) ) 
            {
                return NavState::GateDriveToFarPost;
            }   
        }
    }
    else
    {
        if( mRover->turn( mRover->roverStatus().leftCacheTarget().bearing + mRover->roverStatus().odometry().bearing_deg ) ) 
        {
            return NavState::GateDriveToFarPost;
        }
    }
    return NavState::GateTurnToFarPost;
} // executeGateTurnToFarPost()

// Drive to furthest post (or the only post if only one is available)
NavState GateStateMachine::executeGateDriveToFarPost()
{
    // Minor adjustment to gate targeting, due to issue of driving through a 
    // post when driving through the wrong direction
    double gateAdjustmentDist = mRoverConfig[ "gateAdjustment" ][ "adjustmentDistance" ].GetDouble();

    // Set to first target, since we should have atleast one in sight/detected
    double distance = mRover->roverStatus().leftCacheTarget().distance - gateAdjustmentDist;
    double bearing = mRover->roverStatus().leftCacheTarget().bearing + mRover->roverStatus().odometry().bearing_deg;

    if( mRover->roverStatus().rightCacheTarget().distance > 0 &&
        mRover->roverStatus().leftCacheTarget().distance < mRover->roverStatus().rightCacheTarget().distance ) 
    {
        // Set our variables to drive to target/post 2, which is farther away
        distance = mRover->roverStatus().rightCacheTarget().distance - gateAdjustmentDist;
        bearing = mRover->roverStatus().rightCacheTarget().bearing + mRover->roverStatus().odometry().bearing_deg;
    }

    DriveStatus driveStatus = mRover->drive( distance, bearing, true );

    if( driveStatus == DriveStatus::Arrived )
    {
        return NavState::GateTurnToGateCenter;
    }
    if( driveStatus == DriveStatus::OnCourse )
    {
        return NavState::GateDriveToFarPost;
    }
    return NavState::GateDriveToFarPost;
} // executeGateDriveToFarPost()

// Execute turn back to center point for driving through the gate
NavState GateStateMachine::executeGateTurnToGateCenter()
{
    if( mRover->turn( centerPoint2 ) ) 
    {
        return NavState::GateDriveThrough;
    }
    return NavState::GateTurnToGateCenter;
} // executeGateTurnToGateCenter()

// Drive through gate posts
NavState GateStateMachine::executeGateDriveThrough()
{
    DriveStatus driveStatus = mRover->drive( centerPoint2 );

    if( driveStatus == DriveStatus::Arrived )
    {
        if( !isCorrectGateDir ) // Check if we drove through the incorrect direction
        {
            const Odometry temp = centerPoint1;
            centerPoint1 = centerPoint2;
            centerPoint2 = temp;
            isCorrectGateDir = true;
            return NavState::GateSpin;
        }
        mRover->roverStatus().path().pop_front();
        mRoverStateMachine->updateCompletedPoints();
        return NavState::Turn;
    }
    if( driveStatus == DriveStatus::OnCourse )
    {
        return NavState::GateDriveThrough;
    }
    return NavState::GateDriveThrough;
} // executeGateDriveThrough()

// Update stored location and id for second post.
void GateStateMachine::updatePost2Info()
{
    if( mRover->roverStatus().rightCacheTarget().distance >= 0 && mRover->roverStatus().leftCacheTarget().id == lastKnownRightPost.id )
    {
        const double targetAbsAngle = mod( mRover->roverStatus().odometry().bearing_deg +
                                          mRover->roverStatus().rightCacheTarget().bearing,
                                          360 );
        lastKnownLeftPost.odom = createOdom( mRover->roverStatus().odometry(),
                                          targetAbsAngle,
                                          mRover->roverStatus().rightCacheTarget().distance,
                                          mRover );
        lastKnownLeftPost.id = mRover->roverStatus().rightCacheTarget().id;
    }
    else
    {
        const double targetAbsAngle = mod( mRover->roverStatus().odometry().bearing_deg +
                                          mRover->roverStatus().leftCacheTarget().bearing,
                                          360 );
        lastKnownLeftPost.odom = createOdom( mRover->roverStatus().odometry(),
                                          targetAbsAngle,
                                          mRover->roverStatus().leftCacheTarget().distance,
                                          mRover );
        lastKnownLeftPost.id = mRover->roverStatus().leftCacheTarget().id;
    }
} // updatePost2Info()

// Find the point centered in front of the gate.
// Find the angle that the rover should face from that point to face the gate.
// This point should be on the correct side of the gate so that we drive
// through it in the correct direction.
void GateStateMachine::calcCenterPoint()
{
    const Odometry& currOdom = mRover->roverStatus().odometry();
    const double distFromGate = 3;
    const double gateWidth = mRover->roverStatus().path().front().gate_width;
    const double tagToPointAngle = radianToDegree( atan2( distFromGate, gateWidth / 2 ) );
    const double gateAngle = calcBearing( lastKnownRightPost.odom, lastKnownLeftPost.odom );
    const double absAngle1 = mod( gateAngle + tagToPointAngle, 360 );
    const double absAngle2 = mod( absAngle1 + 180, 360 );
    const double tagToPointDist = sqrt( pow( gateWidth / 2, 2 ) + pow( distFromGate, 2 ) );
    
    // Assuming that CV works well enough that we don't pass through the gate before
    // finding the second post. Thus, centerPoint1 will always be closer.
    centerPoint1 = createOdom( lastKnownRightPost.odom, absAngle1, tagToPointDist, mRover );
    centerPoint2 = createOdom( lastKnownLeftPost.odom, absAngle2, tagToPointDist, mRover );
    const double cp1Dist = estimateNoneuclid( currOdom, centerPoint1 );
    const double cp2Dist = estimateNoneuclid( currOdom, centerPoint2 );
    if( lastKnownRightPost.id % 2 )
    {
        isCorrectGateDir = true;
    }
    else
    {
        isCorrectGateDir = false;
    }
    if( cp1Dist > cp2Dist )
    {
        const Odometry temp = centerPoint1;
        centerPoint1 = centerPoint2;
        centerPoint2 = temp;
        isCorrectGateDir = !isCorrectGateDir;
    }

} // calcCenterPoint()

// Creates an GateStateMachine object
GateStateMachine* GateFactory( StateMachine* stateMachine, Rover* rover, const rapidjson::Document& roverConfig )
{
    return new DiamondGateSearch( stateMachine, rover, roverConfig );
} // GateFactory()

// Helper function to find the distance between the posts
double GateStateMachine::distanceBetweenPosts() {
    mRoverStateMachine->pushNewRoverStatus();
    const Target &t1 = mRover->roverStatus().leftTarget();
    const Target &t2 = mRover->roverStatus().rightTarget();
    double a = t1.distance;
    double b = t2.distance;

    // Both posts must exist
    if (a < 0 || b < 0) {
        return -1;
    }
    else {
        double angle = fabs(t1.bearing - t2.bearing) * 3.141592653 / 180.0;
        double distance = sqrt(a * a + b * b - 2.0 * a * b * cos(angle));
        
        return distance;
    }
} // distanceBetweenPosts()



// determine rover's location on grid (in x,y coordinates)
NavState GateStateMachine::executeGateTurnToPerpPoint() {
    Point perpPoint = makePerpPoint();
    if( mRover->turn(perpPoint.toOdometry()))
    {
        return NavState::GateDriveToPerpPoint;
    }
    return NavState::GateTurnToPerpPoint;
}

NavState GateStateMachine::executeGateDriveToPerpPoint(){
    Point perpPoint = makePerpPoint();

    DriveStatus drivestatus = mRover->drive(perpPoint.toOdometry());

    if(drivestatus == DriveStatus::Arrived )
    {
        return NavState::GateTurnToDivePoint;
    }
    if( drivestatus == DriveStatus::OnCourse )
    {
        return NavState::GateDriveToPerpPoint;
    }
    return NavState::GateDriveToPerpPoint;
}

NavState GateStateMachine::executeGateTurnToDivePoint(){
    Point divePoint = makeDivePoint();
    if( mRover->turn(divePoint.toOdometry()))
    {
        return NavState::GateDriveToDivePoint;
    }
    return NavState::GateTurnToDivePoint;

}

NavState GateStateMachine::executeGateDriveToDivePoint(){
    Point divePoint = makeDivePoint();

    DriveStatus drivestatus = mRover->drive(divePoint.toOdometry());

    if(drivestatus == DriveStatus::Arrived )
    {
        return NavState::GateTurnToAcrossPoint;
    }
    if( drivestatus == DriveStatus::OnCourse )
    {
        return NavState::GateDriveToDivePoint;
    }
    return NavState::GateDriveToDivePoint;
}

NavState GateStateMachine::executeGateTurnToAcrossPoint() {
    Point acrossPoint = makeAcrossPoint();

    if( mRover->turn(acrossPoint.toOdometry()))
    {
        return NavState::GateDriveToAcrossPoint;
    }
    return NavState::GateTurnToAcrossPoint;

}

NavState GateStateMachine::executeGateDriveToAcrossPoint() {
    Point acrossPoint = makeAcrossPoint();

    DriveStatus drivestatus = mRover->drive(acrossPoint.toOdometry());

    if(drivestatus == DriveStatus::Arrived )
    {
        return NavState::Turn;
    }
    if( drivestatus == DriveStatus::OnCourse )
    {
        return NavState::GateDriveToAcrossPoint;
    }
    return NavState::GateDriveToAcrossPoint;

}

Point GateStateMachine::makePerpPoint(){
    Point perpPoint;
    Point roverLocation = determineRoverPoint();
    int perpQuadrant = roverLocation.getQuadrantIn();
    switch(perpQuadrant) {
        case 1:
            perpPoint = Point(1, 1);
        case 2:
            perpPoint = Point(-1, 1);
        case 3:
            perpPoint = Point(-1, -1);
        case 4:
            perpPoint = Point(1, -1);
        default:
            perpPoint = Point(0, 0);
    }
    Point leftGate(mRover->roverStatus().leftCacheTarget().odometry());
    Point rightGate(mRover->rightCacheTarget().odometry());
    Point fakeLeftGate(-1, 0);
    Point fakeRightGate(1, 0);
    return Point::Map(perpPoint, fakeLeftGate, fakeRightGate, leftGate, rightGate);
}

Point GateStateMachine::determineRoverPoint() {
    Point actualRoverPoint(mRover->roverStatus().odometry());
    Point leftGate(mRover->roverStatus().leftCacheTarget().odometry());
    Point rightGate(mRover->rightCacheTarget().odometry());
    Point fakeLeftGate(-1, 0);
    Point fakeRightGate(1, 0);
    return Point::Map(actualRoverPoint, leftGate, rightGate, fakeLeftGate, fakeRightGate);
}

Point GateStateMachine::makeDivePoint() {
    Point divePoint(0,-1);
    Point leftGate(mRover->roverStatus().leftCacheTarget().odometry());
    Point rightGate(mRover->rightCacheTarget().odometry());
    Point fakeLeftGate(-1, 0);
    Point fakeRightGate(1, 0);
    return Point::Map(divePoint, fakeLeftGate, fakeRightGate, leftGate, rightGate);
}

Point GateStateMachine::makeAcrossPoint() {
    Point acrossPoint(0,-1);
    Point leftGate(mRover->roverStatus().leftCacheTarget().odometry());
    Point rightGate(mRover->rightCacheTarget().odometry());
    Point fakeLeftGate(-1, 0);
    Point fakeRightGate(1, 0);
    return Point::Map(acrossPoint, fakeLeftGate, fakeRightGate, leftGate, rightGate);
}




/*
void spiderSearch() {
    // In order to properly drive through the gate, it is required that we must initialize our entry through said gate with a small enough incident angle such that a collision with said gates, also known as "posts", practically impossible, allowing the rover a safe and probalistically likely entry.
    // This is easily done with a program we have developed, with conjunction of the team based in Ann Arbor, Michigan, United States of America (formally known as the The University of the West, or more simply, The Blue) a practical algorithm that makes the implementation of said execution through said "posts" trivial - leaving application quite simple - left as an exerice to the reader.
    // With this said, the actual interface developed is quite intuitive, allowing the rover to properly navigate with a program dubbed "The Spider Search". This "Spider Search" is quite effectively and is up for nomination for several Nobel Peace Prizes in its effective employment of saving several lives on Mars.
    // Below this is the implementation that is so well known today as the savior of modern humanity and its world of influence.

    // Implementation of "Spider-Search" method to go through a gate. This program begins by using advanced Multi Variable and Differential Calculus-Style Algebra in order to determine which quadrant (hereby known as "quadrant") so that it may properly assign a target point of entry into said "post".
   
    
    // Step 1: 
    // Call map to conver rover odometry to (x,y) point
    // Determine which quadrant the rover is in
    // see whichQuadrant, once grid is created, convert odom data to x,y
    // Generate a path to the fixed quadrant point
    // Drive to the perp point 
    // Generate a path to the midline
    // Drive to the from the pre point to the midline
    // Turn to face the gate center
    // Drive from the point on the midline through the gate center point

    bool isLeftOfGateEntryPostsSoThisMustTurnRight = false;
    PerpPoint leftPerpPoint = new PerpPoint(bool isLeft = true);
    PerpPoint rightPerpPoint = leftPerpPoint.reflect({ x: -1, y: 0 });
    
    if (isLeftOfGateEntryPostsSoThisMustTurnRight) {
        // The rover is Left Of Gate Entry Posts, So This Must Turn Right.
        Rover.getInstance().driveToPerpPoint(leftPerpPoint);
    }
    else {
        // Turn left to the perp point
        Rover.getInstance().driveToPerpPoint(rightPerpPoint);

    }
    
    if (Rover.isAtAPerpPoint.anyPerpPointAtThisPoint()) {
        // We are at (really any) perp point (at this point), so drive to the dive points (specifically the one below the entry)
        DrivePoint driveOne = new DrivePoint(Direction.BelowEntry);
    }

       /* Point actualRoverPoint(mRover->roverStatus()->odom());
    Point leftGate(leftGateOdom);
    Point rightGate(rightGateOdom);
    Point fakeLeftGate(-1, 0);
    Point fakeRightGate(1, 0);
    Point fakeRoverPoint = Point::Map(actualRoverPoint, leftGate, rightGate, fakeLeftPoint, fakeRightPoint);
    */
 
  //  Odometry odom(point);
  // center is 0,0, right post 1,0, left post -1,0
  // 
    //Leftpost + halfOfGate
    //Create an odom
    //Pass the odom into constructor
  //  gives us lat,lon in deg and min


    //Convert Post two to x and y

    
/*
void gridCreation(){
 // Transforms position data from odometry into an x,y coordinate grid with midpoint of gate (0,0)
 // Calculate the center line based on PerpPoints.
 // Determine the quadrants
 // Add points in each quadrant
 // Calculate path from quadrant point to midline
    Odometry centerPoint = createCenterPoint();


//convert from x,y in meters to x,y in coordinates
    Point firstGate(1, 0);
    Point secondGate(-1, 0);
    calcBearing(firstGate.toOdometry(), secondGate.toOdometry());


    if (firstGate.distance(secondGate) < 2) {
        // I don't know how this got into the imaginary dimension, but it did and so we must correct its destination
        
    }

    
}
*/




    
    
