#include "Point.h"
#include <cmath>

Point::Point()
: x(0)
, y(0) {
}

Point::Point(double x, double y)
: x(x)
, y(y) {
}

double Point::getX() const {
    return x;
}

double Point::getY() const {
    return y;
}

void Point::setX(double x) {
    this->x = x;
}

void Point::setY(double y) {
    this->y = y;
}

Point& Point::operator+=(const Point& rhs) {
    x += rhs.x;
    y += rhs.y;
    return *this;
}

Point& Point::operator-=(const Point& rhs) {
    x -= rhs.x;
    y -= rhs.y;
    return *this;
}

Point& Point::operator*=(const double rhs) {
    this->x *= rhs;
    this->y *= rhs;
    return *this;
}

Point& Point::operator/=(const double rhs) {
    this->x /= rhs;
    this->y /= rhs;
    return *this;
}

double Point::distance(const Point& a, const Point& b) {
    return sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
}

std::ostream& operator<<(std::ostream& os, const Point& p) {
    os << "(" << p.x << "," << p.y << ")";
    return os;
}