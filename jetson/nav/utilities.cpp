#include "utilities.hpp"
#include <iostream> // remove
#include <cmath>

// Coverts the input degree (and optional minute) to radians.
double degreeToRadian( const double degree, const double minute )
{
    return ( PI / 180 ) * ( degree + minute / 60 );
} // degreeToRadian

// Converts the input radians to degrees.
double radianToDegree( const double radian )
{
    return radian * 180 / PI;
}

// create a new odom with coordinates offset from current odom by a certain lat and lon change
Odometry addMinToDegrees( const Odometry & current, const double lat_minutes, const double lon_minutes )
{
    Odometry newOdom = current;
    double total_lat_min = current.latitude_min + lat_minutes;
    int sign_lat = total_lat_min < 0 ? -1 : 1;
    newOdom.latitude_min = mod( fabs( total_lat_min ), 60 ) * sign_lat;
    newOdom.latitude_deg += ( total_lat_min ) / 60;
    double total_lon_min = current.longitude_min + lon_minutes;
    int sign_lon = total_lon_min < 0 ? -1 : 1;
    newOdom.longitude_min = mod( fabs( total_lon_min ), 60 ) * sign_lon;
    newOdom.longitude_deg += ( total_lon_min )/60;

    return newOdom;
}

// Caclulates the non-euclidean distance between the current odometry and the
// destination odometry.
double estimateNoneuclid( const Odometry& current, const Odometry& dest )
{
    double currentLat = degreeToRadian( current.latitude_deg, current.latitude_min );
    double currentLon = degreeToRadian( current.longitude_deg, current.longitude_min );
    double destLat = degreeToRadian( dest.latitude_deg, dest.latitude_min );
    double destLon = degreeToRadian( dest.longitude_deg, dest.longitude_min );

    double diffLat = ( destLat - currentLat );
    double diffLon = ( destLon - currentLon ) * cos( ( currentLat + destLat ) / 2 );
    return sqrt( diffLat * diffLat + diffLon * diffLon ) * EARTH_RADIUS;
}

// create a new Odometry point at a bearing and distance from a given odometry point
// Note this uses the absolute bearing not a bearing relative to the rover.
Odometry createOdom( const Odometry & current, double bearing, const double distance, Rover * rover )
{
    bearing = degreeToRadian( bearing );
    double latChange = distance * cos( bearing ) * LAT_METER_IN_MINUTES;
    double lonChange = distance * sin( bearing  ) * rover->longMeterInMinutes();
    Odometry newOdom = addMinToDegrees( current, latChange, lonChange );
    return newOdom;
}

// Caclulates the bearing between the current odometry and the
// destination odometry.
double calcBearing( const Odometry& start, const Odometry& dest )
{
    double currentLat = degreeToRadian( start.latitude_deg, start.latitude_min );
    double currentLon = degreeToRadian( start.longitude_deg, start.longitude_min );
    double destLat = degreeToRadian( dest.latitude_deg, dest.latitude_min );
    double destLon = degreeToRadian( dest.longitude_deg, dest.longitude_min );

    double verticleComponentDist = EARTH_RADIUS * sin( destLat - currentLat );
    double noneuclidDist = estimateNoneuclid( start, dest );

    double bearing = acos( verticleComponentDist / noneuclidDist );
    if( currentLon > destLon )
    {
        bearing = 2 * PI - bearing;
    }

    if( verticleComponentDist < 0.001 && verticleComponentDist > -0.001 )
    {
        if( currentLon < destLon )
        {
            bearing = PI / 2;
        }
        else
        {
            bearing = 3 * PI / 2;
        }
    }
    return radianToDegree( bearing );
} // calcBearing()

// // Calculates the modulo of degree with the given modulus.
double mod( const double degree, const int modulus )
{
    double mod = fmod( degree, modulus );
    if( mod < 0 )
    {
        return ( mod + modulus );
    }
    return mod;
}

// Corrects the destination bearing to account for the ability to turn
// through zero degrees.
void throughZero( double& destinationBearing, const double currentBearing )
{
    if( fabs( currentBearing - destinationBearing ) > 180 )
    {
        if( currentBearing < 180 )
        {
            destinationBearing -= 360;
        }
        else
        {
            destinationBearing += 360;
        }
    }
} // throughZero()

// Clears the queue.
void clear( deque<Waypoint>& aDeque )
{
    deque<Waypoint> emptyDeque;
    swap( aDeque, emptyDeque );
} // clear()


// Checks to see if target is reachable before hitting obstacle
// If the x component of the distance to obstacle is greater than
// half the width of the rover the obstacle if reachable
bool isTargetReachable( Rover* rover, const rapidjson::Document& roverConfig )
{
    double distToTarget = rover->roverStatus().leftCacheTarget().distance;
    double distThresh = roverConfig[ "navThresholds" ][ "targetDistance" ].GetDouble();
    return isLocationReachable( rover, roverConfig, distToTarget, distThresh );
} // istargetReachable()

// Returns true if the rover can reach the input location without hitting the obstacle.
// ASSUMPTION: There is an obstacle detected.
// ASSUMPTION: The rover is driving straight.
bool isLocationReachable( Rover* rover, const rapidjson::Document& roverConfig, const double locDist, const double distThresh )
{
    double distToObs = rover->roverStatus().obstacle().distance;
    double bearToObs = std::min( rover->roverStatus().obstacle().bearing, rover->roverStatus().obstacle().rightBearing );
    double bearToObsComplement = 90 - bearToObs;
    double xComponentOfDistToObs = distToObs * cos( bearToObsComplement );

    bool isReachable = false;

    // if location - distThresh is closer than the obstacle, it's reachable
    isReachable |= distToObs > locDist - distThresh;

    // if obstacle is farther away in "x direction" than rover's width, it's reachable
    isReachable |= xComponentOfDistToObs > roverConfig[ "roverMeasurements" ][ "width" ].GetDouble() / 2;

    return isReachable;
} // isLocationReachable()

// Returns true if an obstacle is detected, false otherwise.
bool isObstacleDetected( Rover* rover )
{
    return rover->roverStatus().obstacle().distance >= 0;
} // isObstacleDetected()

// Returns true if distance from obstacle is within user-configurable threshold
bool isObstacleInThreshold( Rover* rover, const rapidjson::Document& roverConfig )
{
    return rover->roverStatus().obstacle().distance <= roverConfig[ "navThresholds" ][ "obstacleDistanceThreshold" ].GetDouble();
} // isObstacleInThreshold()


Point Point::Map(const Point& input, const Point& a_in, const Point& b_in, const Point &a_out, const Point &b_out) {
    // Convert input from one 2d space to another.
    // This requires some linear algebra, so buckle up for some MATH 214 shenanigans.
    double scale_in = a_in.distance(b_in);
    double scale_out = a_out.distance(b_out);

    // Translate the first point to the origin
    Point bEffect = b_in - a_in;
    Point cEffect = input - a_in;
    
    Point bEffect_out = b_out - a_out;

    // Rotate b to be at the same angle as its output
    double desiredAngle = atan2(bEffect_out.getY(), bEffect_out.getX());
    double currentAngle = atan2(bEffect.getY(), bEffect.getX());
    
    double rotateAmount = desiredAngle - currentAngle;

    // Rotate both points around the origin.
    bEffect = Point((bEffect.getX() * cos(rotateAmount)) - (bEffect.getY() * sin(rotateAmount)),
                    (bEffect.getX() * sin(rotateAmount)) + (bEffect.getY() * cos(rotateAmount)));

    cEffect = Point((cEffect.getX() * cos(rotateAmount)) - (cEffect.getY() * sin(rotateAmount)),
                    (cEffect.getX() * sin(rotateAmount)) + (cEffect.getY() * cos(rotateAmount)));

    // Scale them properly.
    bEffect = bEffect * (scale_out / scale_in);
    cEffect = cEffect * (scale_out / scale_in);

    // Rotate back in place
    bEffect = bEffect + a_out;
    cEffect = cEffect + a_out;

    return cEffect;
}

Quadrant Point::getQuadrantIn() const {
    if (getX() > 0) {
        if (getY() > 0) {
            return Quadrant::TopRight;
        }
        else {
            return Quadrant::BottomRight;
        }
    }
    else {
        if (getY() > 0) {
            return Quadrant::TopLeft;
        }
        else {
            return Quadrant::BottomLeft;
        }
    }
}

Point::Point(double x_in, double y_in) : x(x_in), y(y_in) { }

Point::Point() : x(0), y(0) { }

Point::Point(const Odometry &point) :
    x(point.longitude_deg + point.longitude_min / 60.0),
    y(point.latitude_deg + point.latitude_min / 60.0) { }

double Point::operator*(const Point& p) const {
    return x * p.x + y * p.y;
}
Point Point::operator+(const Point& p) const {
    return Point(x + p.x, y + p.y);
}
Point Point::operator-(const Point& p) const {
    return Point(x - p.x, y - p.y);
}
Point Point::operator*(double s) const {
    return Point(x * s, y * s);
}

double Point::distance(const Point& other) const {
    return sqrt((other.x - x) * (other.x - x) + (other.y - y) * (other.y - y));
}


Odometry Point::toOdometry() const {
    int long_deg = (int)getX();
    double long_min = (x - long_deg) * 60.0;
    int lat_deg = (int)getY();
    double lat_min = (y - lat_deg) * 60.0;
    Odometry odomToReturn = {lat_deg, lat_min, long_deg, long_min, 0, 0};
    return odomToReturn;
}

void Point::setX(int x_in) {
    x = x_in;
}
void Point::setY(int y_in) {
    y= y_in;
}
double Point::getX() const {
    return x;
}

double Point::getY() const {
    return y;
}



