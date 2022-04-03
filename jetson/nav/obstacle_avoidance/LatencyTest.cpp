#include "obstacleAvoidance.hpp"
#include "obstacleAvoidance.cpp"

using namespace std;

TEST(test_threshold){
    desiredBearing =  5.0;
    Odometry roverOdom = { 38, 24, -110, 47.52, 30, 0};
    double threshold = desiredBearing;
    getLatencyAdjustedDesiredBearing(Odometry roverOdom, desiredBearing);
  
    ASSERT_EQUAL(threshold,5.0);
}

TEST_MAIN()

