#include "obstacleAvoidance.hpp"
#include "utilities.hpp"

ObstacleAvoidance::ObstacleAvoidance(const rapidjson::Document& roverConfig ) 
    : mRoverConfig(roverConfig) {}

//returns a bearing decision struct representing the desired NavState and bearing of the obstacle avoidance controller
ObstacleAvoidance::BearingDecision ObstacleAvoidance::getDesiredBearingDecision(std::vector<Obstacle>& obstacles, Odometry roverOdom, Odometry dest){
    //TODO implement
    return {NavState::Drive, -1.0};
}

bool ObstacleAvoidance::isObstacleInBearing(Obstacle& obstacle, BearingLines& bearings) {
    double LBL_botLeft = (bearings.n(0) * (obstacle.bottom_left_coordinate_meters[0] - bearings.bLeft(0))) + (bearings.n(1) * (obstacle.bottom_left_coordinate_meters[2] - bearings.bLeft(1)));
    double RBL_botLeft = (bearings.n(0) * (obstacle.bottom_left_coordinate_meters[0] - bearings.bRight(0))) + (bearings.n(1) * (obstacle.bottom_left_coordinate_meters[2] - bearings.bRight(1)));

    double LBL_topRight = (bearings.n(0) * (obstacle.top_right_coordinate_meters[0] - bearings.bLeft(0))) + (bearings.n(1) * (obstacle.top_right_coordinate_meters[2] - bearings.bLeft(1)));
    double RBL_topRight = (bearings.n(0) * (obstacle.top_right_coordinate_meters[0] - bearings.bRight(0))) + (bearings.n(1) * (obstacle.top_right_coordinate_meters[2] - bearings.bRight(1)));

    if (
        // Check if botLeft is in Rover path
        ((LBL_botLeft > 0 && RBL_botLeft < 0) || (LBL_botLeft < 0 && RBL_botLeft > 0)
        || LBL_botLeft == 0 || RBL_botLeft == 0) ||
        // Check if topRight is in Rover path
        ((LBL_topRight > 0 && RBL_topRight < 0) || (LBL_topRight < 0 && RBL_topRight > 0)
        || LBL_topRight == 0 || RBL_topRight == 0) ||
        // Check if obstacle spans Rover path
        (LBL_botLeft < 0 && RBL_topRight > 0) || (RBL_botLeft > 0 && LBL_topRight < 0) 
        ) {
        return true;
    }
    return false;
}

//returns a vector of doubles representing clear bearings through a list of obstacles
//discretized in 1 degree increments, starting from -FOV to +FOV.
std::vector<double> ObstacleAvoidance::getClearBearings(std::vector<Obstacle>& obstacles){
    //TODO implement
    double left_fov_max = -mRoverConfig[ "computerVision" ][ "fieldOfViewAngle" ].GetDouble()/2.0;
    double right_fov_max = -mRoverConfig[ "computerVision" ][ "fieldOfViewAngle" ].GetDouble()/2.0;
    vector<double> possible_bearings;
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
    //TODO: implement
    
    // First get the ideal bearing
    double idealBearing = calcBearing(roverOdom, dest);

    // Now we must find the closest clear bearings
    double closestBearing = 900;
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
    if(desiredBearing > threshold){
        return desiredBearing+threshold;
    }
    return desiredBearing;
    
}
