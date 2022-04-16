#include "obstacle_avoidance/obstacleAvoidance.hpp"
#include "utilities.hpp"
#include <iostream>

using namespace std;

ObstacleAvoidance::ObstacleAvoidance(const rapidjson::Document& roverConfig ) 
    : mRoverConfig(roverConfig) {}

//returns a bearing decision struct representing the desired NavState and bearing of the obstacle avoidance controller
ObstacleAvoidance::BearingDecision ObstacleAvoidance::getDesiredBearingDecision(std::vector<Obstacle>& obstacles, Odometry roverOdom, Odometry dest){
    //Get the vector of clear bearings and see if it is empty
    std::vector<double> clearBearings = getClearBearings(obstacles);
    
    if(clearBearings.size() == 0) {
        return {NavState::Turn, getLatencyAdjustedDesiredBearing(roverOdom, mRoverConfig[ "computerVision" ][ "fieldOfViewAngle" ].GetDouble() + 1.0)};
    }

        
    double idealBearingUnadjusted = getIdealDesiredBearing(roverOdom, dest, clearBearings);
    double adjustedIdealBearing = getLatencyAdjustedDesiredBearing(roverOdom, idealBearingUnadjusted);

    //Search for 0.0 in clear bearings
    bool zeroPointZeroClear = false;
    for(double bearing : clearBearings) {

        if(fabs(bearing - 0.0) < 
                mRoverConfig[ "navThresholds" ][ "clearBearingComparisonThreshold" ].GetDouble()) {
            zeroPointZeroClear = true;
        }
    }   



    //If there is an obstacle directly in front of the Rover calculate the distance to it
    if(!zeroPointZeroClear) {
        Obstacle obsInFront;
        BearingLines bearings(0.0, mRoverConfig[ "roverMeasurements" ][ "width" ].GetDouble());
        for (Obstacle& obstacle : obstacles) {
            if(isObstacleInBearing(obstacle, bearings)){
                obsInFront = obstacle;
            }
        }

        double distanceToObstacle;

        if(obsInFront.bottom_left_coordinate_meters[2] < obsInFront.top_right_coordinate_meters[2]) {
            distanceToObstacle = obsInFront.bottom_left_coordinate_meters[2];
        }
        else {
            distanceToObstacle = obsInFront.top_right_coordinate_meters[2];  
        }

        double threshold = mRoverConfig[ "navThresholds" ][ "obstacleDistanceThreshold" ].GetDouble();

        //If there is enough room in front of the Rover, drive to the adjusted ideal bearing
        if(distanceToObstacle > threshold) {
            return {NavState::Drive, adjustedIdealBearing};
        }
        
        //If there is not enough room in front of the Rover, turn to the adjusted ideal bearing
        else if(distanceToObstacle < threshold) {
            return {NavState::Turn, adjustedIdealBearing};
        }
    }

    //If there are no obstacles in front of the rover, continue driving on current trajectory
    return {NavState::Drive, calcBearing(roverOdom,dest)};  
}

bool ObstacleAvoidance::isObstacleInBearing(Obstacle& obstacle, BearingLines& bearings) {
    double LBL_botLeft = (bearings.n(0) * (obstacle.bottom_left_coordinate_meters[0] - bearings.bLeft(0))) + (bearings.n(1) * (obstacle.bottom_left_coordinate_meters[2] - bearings.bLeft(1)));
    double RBL_botLeft = (bearings.n(0) * (obstacle.bottom_left_coordinate_meters[0] - bearings.bRight(0))) + (bearings.n(1) * (obstacle.bottom_left_coordinate_meters[2] - bearings.bRight(1)));

    double LBL_topRight = (bearings.n(0) * (obstacle.top_right_coordinate_meters[0] - bearings.bLeft(0))) + (bearings.n(1) * (obstacle.top_right_coordinate_meters[2] - bearings.bLeft(1)));
    double RBL_topRight = (bearings.n(0) * (obstacle.top_right_coordinate_meters[0] - bearings.bRight(0))) + (bearings.n(1) * (obstacle.top_right_coordinate_meters[2] - bearings.bRight(1)));

    double LBL_botRight = (bearings.n(0) * (obstacle.top_right_coordinate_meters[0] - bearings.bLeft(0))) + (bearings.n(1) * (obstacle.bottom_left_coordinate_meters[2] - bearings.bLeft(1)));
    double RBL_botRight = (bearings.n(0) * (obstacle.top_right_coordinate_meters[0] - bearings.bRight(0))) + (bearings.n(1) * (obstacle.bottom_left_coordinate_meters[2] - bearings.bRight(1)));

    double LBL_topLeft = (bearings.n(0) * (obstacle.bottom_left_coordinate_meters[0] - bearings.bLeft(0))) + (bearings.n(1) * (obstacle.top_right_coordinate_meters[2] - bearings.bLeft(1)));
    double RBL_topLeft = (bearings.n(0) * (obstacle.bottom_left_coordinate_meters[0] - bearings.bRight(0))) + (bearings.n(1) * (obstacle.top_right_coordinate_meters[2] - bearings.bRight(1)));

    // Check if obstacle its between bearing lines
    if ((LBL_botLeft > 0 && RBL_botLeft < 0) || (LBL_botLeft < 0 && RBL_botLeft > 0)
        || LBL_botLeft == 0 || RBL_botLeft == 0) {
        return true; // This is not a clear path
    }

    if ((LBL_botRight > 0 && RBL_botRight < 0) || (LBL_botRight < 0 && RBL_botRight > 0)
        || LBL_botRight == 0 || RBL_botRight == 0) {
        return true; // This is not a clear path
    }

    if ((LBL_topLeft > 0 && RBL_topLeft < 0) || (LBL_topLeft < 0 && RBL_topLeft > 0)
        || LBL_topLeft == 0 || RBL_topLeft == 0) {
        return true; // This is not a clear path
    }

    if ((LBL_topRight > 0 && RBL_topRight < 0) || (LBL_topRight < 0 && RBL_topRight > 0)
        || LBL_topRight == 0 || RBL_topRight == 0) {
        return true; // This is not a clear path
    }

    // Check if obstacle is larger than span of bearing lines
    if ((LBL_botLeft > 0 && RBL_botRight < 0) || (LBL_topLeft > 0 && RBL_topRight < 0)) {
        return true; // This is not a clear path
    }
    return false;
}

//returns a vector of doubles representing clear bearings through a list of obstacles
//discretized in 1 degree increments, starting from -FOV to +FOV.
std::vector<double> ObstacleAvoidance::getClearBearings(std::vector<Obstacle>& obstacles){
    double left_fov_max = -mRoverConfig[ "computerVision" ][ "fieldOfViewAngle" ].GetDouble()/2.0;
    double right_fov_max = mRoverConfig[ "computerVision" ][ "fieldOfViewAngle" ].GetDouble()/2.0;
    std::vector<double> possible_bearings;
    for (double heading = left_fov_max; heading < right_fov_max; heading++) {
        BearingLines bearings(heading * 3.1415926535 / 180.0, mRoverConfig[ "roverMeasurements" ][ "width" ].GetDouble()); //Create bearing using radians
        bool obstacle_in_heading = false;
        for (Obstacle& obstacle : obstacles) {
            if (isObstacleInBearing(obstacle, bearings))
                obstacle_in_heading = true;
        }
        if (!obstacle_in_heading)
            possible_bearings.push_back(heading);
    }
    return possible_bearings;
}

//returns a bearing that the rover should target to try to get to the destination while also getting around obstacles
//not latency adjusted
double ObstacleAvoidance::getIdealDesiredBearing(Odometry roverOdom, Odometry dest, std::vector<double> clearBearings){
    if (clearBearings.size() == 0) {
        return mRoverConfig[ "computerVision" ][ "fieldOfViewAngle" ].GetDouble() + 1.0;
    }
    // First get the ideal bearing
    double idealBearing = calcBearing(roverOdom, dest);
    // cout << "closest to ideal bearing " << idealBearing << "\n";
    // Now we must find the closest clear bearings
    double closestBearing = 900; //arbitrary max value
    for (const double & bearing : clearBearings) {
        if (abs(bearing - idealBearing) < abs(closestBearing - idealBearing)) {
            // New closest
            closestBearing = bearing;
        }
    }

    return closestBearing;
}

//returns an adjusted target bearing based on latency specifications (thresholding on maximum allowable change in bearing)
double ObstacleAvoidance::getLatencyAdjustedDesiredBearing(Odometry roverOdom, double desiredBearing){
    // find latency 
    double threshold = mRoverConfig[ "latencyThreshold" ][ "threshold" ].GetDouble();
    // std::cout << "have " << desiredBearing << ", thresh " << threshold << "\n";
    if(desiredBearing > threshold){
        return desiredBearing+threshold;
    }
    if(-desiredBearing > threshold){
        return desiredBearing-threshold;
    }
    return desiredBearing;
    
}
