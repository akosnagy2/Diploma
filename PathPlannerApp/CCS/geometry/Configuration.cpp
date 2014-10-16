#include "Configuration.h"
#include "misc.h"
#include <cmath>

Configuration::Configuration()
: position(0, 0)
, orientation(0) {
}

Configuration::Configuration(const Point& p, double theta)
: position(p)
, orientation(theta) {
}

Configuration::Configuration(double x, double y, double theta)
: position(x, y)
, orientation(theta) {
}

/**
 * Transforms the point in the conf's coordinate system.
 * 
 * @param point Point to transform
 * @return Point in the new coordinate system
 */
Point Configuration::inverseTransform(const Point& point) const {
    double sinF = sin(orientation);
    double cosF = cos(orientation);
    double newX = (point.x - position.x) * cosF + (point.y - position.y) * sinF;
    double newY = (position.x - point.x) * sinF + (point.y - position.y) * cosF;

    return Point(newX, newY);
}

/**
 * Transforms the point into the global frame.
 * 
 * @param point Point to transform
 * @return Point int the new (global) frame
 */
Point Configuration::transform(const Point& point) const {
    double sinF = sin(orientation);
    double cosF = cos(orientation);
    double newX = point.x * cosF - point.y * sinF + position.x;
    double newY = point.x * sinF + point.y * cosF + position.y;

    return Point(newX, newY);
}

Point Configuration::translate(const Point& point, double dist) {
    return Point(point.x + cos(orientation) * dist, point.y + sin(orientation) * dist);
}

std::ostream& operator<<(std::ostream& os, const Configuration& cf) {
    os << "(" << cf.position.x << "," << cf.position.y << "," << cf.orientation << ")";
    return os;
}

Configuration& Configuration::inverseTransform(const Configuration& frame) {
    this->position = frame.inverseTransform(this->position);
    this->orientation = wrapAngle(this->orientation - frame.orientation);
    return *this;
}

Configuration& Configuration::transform(const Configuration& frame) {
    this->position = frame.transform(this->position);
    this->orientation = wrapAngle(this->orientation + frame.orientation);
    return *this;
}