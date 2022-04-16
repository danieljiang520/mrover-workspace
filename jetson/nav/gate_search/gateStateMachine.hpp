#pragma once
<<<<<<< HEAD
=======
// #ifndef GATE_STATE_MACHINE_HPP
// #define GATE_STATE_MACHINE_HPP
>>>>>>> ankith/obstacle-avoidance

#include <deque>
#include <memory>

#include <eigen3/Eigen/Core>

#include "../rover.hpp"
#include "../filter.hpp"
#include "rover_msgs/Odometry.hpp"

using namespace rover_msgs;

class StateMachine;

class GateStateMachine {
public:
    /*************************************************************************/
    /* Public Member Functions */
    /*************************************************************************/
    GateStateMachine(std::weak_ptr<StateMachine> stateMachine, const rapidjson::Document& roverConfig);

    ~GateStateMachine();

    NavState run();

    void updateGateTraversalPath();

    /*************************************************************************/
    /* Public Member Variables */
    /*************************************************************************/

protected:
    /*************************************************************************/
    /* Protected Member Variables */
    /*************************************************************************/

    // Pointer to rover State Machine to access member functions
    std::weak_ptr<StateMachine> mStateMachine;

private:
    /*************************************************************************/
    /* Private Member Functions */
    /*************************************************************************/

<<<<<<< HEAD
    void publishGatePath();
=======
    NavState executeGateSpinWait();

    NavState executeGateTurn();

    NavState executeGateDrive();

    NavState executeGateTurnToCentPoint();

    NavState executeGateDriveToCentPoint();

    NavState executeGateFace();

    NavState executeGateTurnToFarPost();

    NavState executeGateDriveToFarPost();

    NavState executeGateTurnToGateCenter();
    
    NavState executeGateShimmy();

    NavState executeGateDriveThrough();

    NavState executeGateTurnToPerpPoint();

    NavState executeGateDriveToPerpPoint();

    NavState executeGateTurnToDivePoint();

    NavState executeGateDriveToDivePoint();

    NavState executeGateTurnToAcrossPoint();

    NavState executeGateDriveToAcrossPoint();


    

    void updatePost2Info();

    void calcCenterPoint();
    
    Odometry createCenterPoint();

    // Helper function to find the distance between the posts
    double distanceBetweenPosts();
>>>>>>> ankith/obstacle-avoidance

    /*************************************************************************/
    /* Private Member Variables */
    /*************************************************************************/
    const rapidjson::Document& mConfig;

    std::deque<Odometry> mPath;

<<<<<<< HEAD
    void makeDualSegmentPath(std::shared_ptr<Rover> const& rover, std::shared_ptr<Environment>& env);
=======
    // Points in front of center of gate
    Odometry centerPoint1;
    Odometry centerPoint2;
>>>>>>> ankith/obstacle-avoidance

    void makeSpiderPath(std::shared_ptr<Rover> const& rover, std::shared_ptr<Environment>& env);
};

<<<<<<< HEAD
std::shared_ptr<GateStateMachine> GateFactory(const std::weak_ptr<StateMachine>& sm, const rapidjson::Document& roverConfig);
=======
std::shared_ptr<GateStateMachine>
GateFactory(std::weak_ptr<StateMachine> stateMachine, const rapidjson::Document& roverConfig);

// #endif //GATE_STATE_MACHINE_HPP
>>>>>>> ankith/obstacle-avoidance
