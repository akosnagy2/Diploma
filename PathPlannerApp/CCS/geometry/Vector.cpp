/* 
 * File:   Vector.cpp
 * Author: Gábor
 * 
 * Created on 2014. április 10., 15:32
 */

#include <math.h>

#include "Vector.h"

Vector::Vector(const Point& p)
: Point(p) {
}

Vector::Vector(double x, double y)
: Point(x,y) {
}

Vector::Vector(const Point& p1, const Point& p2)
: Point(p2-p1){
}

Vector::Vector(const Segment& seg)
: Point(seg.getB()-seg.getA()) {
}

double Vector::getScalarProduct(const Vector& v) {
    return x * v.x + y * v.y;
}

double Vector::operator *(const Vector& rhs) {
    return getScalarProduct(rhs);
}

double Vector::getFi() const {
    return atan2(y,x);
}

double Vector::getLength() const {
    return sqrt(x*x+y*y);
}
