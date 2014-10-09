/* 
 * File:   Vector.h
 * Author: Gábor
 *
 * Created on 2014. április 10., 15:32
 */

#ifndef VECTOR_H
#define	VECTOR_H

#include "Point.h"
#include "Segment.h"

class Vector : public Point {
public:
    Vector(double x, double y);
    Vector(const Point& p);
    Vector(const Point& p1, const Point& p2);
    Vector(const Segment& seg);

    double getScalarProduct(const Vector& v);
    double operator*(const Vector& rhs);
    double getFi() const;
    double getLength() const;
private:
};

#endif	/* VECTOR_H */

