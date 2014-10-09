/* 
 * File:   Circle.cpp
 * Author: Gábor
 * 
 * Created on 2014. március 29., 14:13
 */

#include <cmath>

#include "Circle.h"
#include "Line.h"

Circle::Circle(const Point& center, double radius)
: center(center)
, radius(radius) {
}

pointList Circle::getIntersections(const Line& line) {
    pointList intersections;

    double v1 = cos(line.getAngle());
    double v2 = sin(line.getAngle());
    double k = line.getPoint().x * v2 - line.getPoint().y * v1;

    if (fabs(v2) > 0.0001) {
        double A = (v1 / v2) * (v1 / v2) + 1;
        double B = 2 * (k / v2 - center.x) * (v1 / v2) - 2 * center.y;
        double C = (k / v2 - center.x) * (k / v2 - center.x) + center.y * center.y - radius*radius;
        if ((B * B - 4 * A * C) >= 0) {
            double y = (-B + sqrt(B * B - 4 * A * C)) / (2 * A);
            double x = (k + v1 * y) / v2;
            intersections.push_back(Point(x, y));

            y = (-B - sqrt(B * B - 4 * A * C)) / (2 * A);
            x = (k + v1 * y) / v2;
            intersections.push_back(Point(x, y));
        }
    } else {
        double A = (v2 / v1) * (v2 / v1) + 1;
        double B = -2 * (k / v1 + center.y) * (v2 / v1) - 2 * center.x;
        double C = (k / v1 + center.y) * (k / v1 + center.y) + center.x * center.x - radius*radius;
        if ((B * B - 4 * A * C) >= 0) {
            double x = (-B + sqrt(B * B - 4 * A * C)) / (2 * A);
            double y = (k - v2 * x) / (-v1);
            intersections.push_back(Point(x, y));

            x = (-B - sqrt(B * B - 4 * A * C)) / (2 * A);
            y = (k - v2 * x) / (-v1);
            intersections.push_back(Point(x, y));
        }
    }
    return intersections;
}
