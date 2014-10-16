#pragma once

#include <vector>
#include <ostream>

class Point {
public:
    Point(double x, double y);
    Point();
    double getX() const;
    double getY() const;
    void setX(double x);
    void setY(double y);
    
    /* Operators */
    Point& operator+=(const Point& rhs);
    Point& operator-=(const Point& rhs);
    Point& operator*=(const double rhs);
    Point& operator/=(const double rhs);
    
    static double distance(const Point& a, const Point& b);

    double x, y;
};

inline Point operator+(Point lhs, const Point& rhs) {
    lhs += rhs;
    return lhs;
}

inline Point operator-(Point lhs, const Point& rhs) {
    lhs -= rhs;
    return lhs;
}

inline Point operator*(Point lhs, const double rhs) {
    lhs *= rhs;
    return lhs;
}

inline Point operator/(Point lhs, const double rhs) {
    lhs /= rhs;
    return lhs;
}

inline bool operator==(const Point& lhs, const Point& rhs) {
    if(lhs.x == rhs.x && lhs.y == rhs.y) {
        return true;
    } else {
        return false;
    }
}

inline bool operator!=(const Point& lhs, const Point& rhs) {
    return !operator==(lhs,rhs);
}

std::ostream& operator<<(std::ostream& os, const Point& p);

typedef std::vector<Point> pointList;