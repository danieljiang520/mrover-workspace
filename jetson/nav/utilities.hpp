#ifndef NAV_UTILITES
#define NAV_UTILITES

#include <deque>
#include "rover_msgs/Waypoint.hpp"
#include "rover_msgs/Odometry.hpp"
#include "rover.hpp"

using namespace std;
using namespace rover_msgs;

enum Quadrant {
    BottomLeft = 3,
    BottomRight = 4,
    TopLeft = 2,
    TopRight = 1,
};

class Point {
public:
    Point();
    Point(double x_in, double y_in);
    Point(const Odometry &point);
    double getX() const;
    double getY() const;
    void setX(int x_in);
    void setY(int y_in);
    Odometry toOdometry() const;
    double distance(const Point& other) const;
    static Point Map(const Point& input, const Point& a_in, const Point& b_in, const Point &a_out, const Point &b_out);
    Quadrant getQuadrantIn() const;
    Point operator+(const Point& p) const;
    Point operator-(const Point& p) const;
    double operator*(const Point& p) const; // dot product
    Point operator*(double s) const;
    
    //friend std::ostream& operator<<(std::ostream& os, const Point &p);
private:
    double x;
    double y;
};

const int EARTH_RADIUS = 6371000; // meters
const int EARTH_CIRCUM = 40075000; // meters
const double PI = 3.141592654; // radians
const double LAT_METER_IN_MINUTES = 0.0005389625; // minutes/meters

double degreeToRadian( const double degree, const double minute = 0 );

double radianToDegree( const double radian );

Odometry addMinToDegrees( const Odometry & current, const double lat_minutes = 0, const double lon_minutes = 0 );

double estimateNoneuclid( const Odometry& start, const Odometry& dest );

Odometry createOdom ( const Odometry & current, const double bearing, const double distance, Rover * rover );

double calcBearing( const Odometry& start, const Odometry& dest );

double mod( const double degree, const int modulus );

void throughZero( double& destinationBearing, const double currentBearing );

void clear( deque<Waypoint>& aDeque );

bool isTargetReachable( Rover* rover, const rapidjson::Document& roverConfig );

bool isLocationReachable( Rover* rover, const rapidjson::Document& roverConfig, const double locDist, const double distThresh );

bool isObstacleDetected( Rover* rover );

bool isObstacleInThreshold( Rover* rover, const rapidjson::Document& roverConfig );

#endif // NAV_UTILITES