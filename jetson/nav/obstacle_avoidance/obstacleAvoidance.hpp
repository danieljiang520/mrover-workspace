#include <vector>
#include "rover_msgs/Obstacle.hpp"
#include "rover_msgs/Odometry.hpp"
#include "rover.hpp"
#include <eigen3/Eigen/Dense>
#include <cmath>

using namespace Eigen;


class ObstacleAvoidance{
    public:
        ObstacleAvoidance(const rapidjson::Document& roverConfig);
        struct BearingDecision{
            NavState obstacleControllerOutputState;
            double desiredBearing;
        };

        class BearingLines {
            public:
                double heading; //Each bearingline defined by a heading
                Eigen::Vector2d n; //Normal vector to bearingline
                Eigen::Vector2d bLeft; //Left offset
                Eigen::Vector2d bRight; //Right offset

                BearingLines(double heading_in, double roverWidth) : heading{heading_in} {
                    n << -cos(heading_in); //Calculate x component of orthogonal vec from heading_in
                    n << sin(heading_in); //Calculate y component of orthogonal vec from heading_in
                    bLeft << (-roverWidth / 2.0) * cos(heading_in); //Calculate bLeft x offset from heading_in
                    bLeft << (roverWidth / 2.0) * sin(heading_in); //Calculate bLeft y offset from heading_in
                    bRight << (roverWidth / 2.0) * cos(heading_in); //Calculate bRight x offset from heading_in
                    bRight << (-roverWidth / 2.0) * sin(heading_in); //Calculate bRight y offset from heading_in
                }
        };

        //returns a bearing decision struct representing the desired NavState and bearing of the obstacle avoidance controller
        BearingDecision getDesiredBearingDecision(std::vector<Obstacle>& obstacles, Odometry roverOdom, Odometry dest);
    
    private:
        //returns a vector of doubles representing clear bearings through a list of obstacles
        //discretized in 1 degree increments, starting from -FOV to +FOV.
        std::vector<double> getClearBearings(std::vector<Obstacle>& obstacles);
        
        //returns a bearing that the rover should target to try to get to the destination while also getting around obstacles
        //not latency adjusted
        double getIdealDesiredBearing(Odometry roverOdom, Odometry dest, std::vector<double> clearBearings);

        //returns an adjusted target bearing based on latency specifications (thresholding on maximum allowable change in bearing)
        double getLatencyAdjustedDesiredBearing(Odometry roverOdom, double desiredBearing);

        bool isObstacleInBearing(Obstacle& obstacle, BearingLines& bearings);

        const rapidjson::Document& mRoverConfig;
};