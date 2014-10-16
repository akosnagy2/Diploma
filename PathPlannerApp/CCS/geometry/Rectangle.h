#pragma once
#include "Shape.h"

class Rectangle :
public Shape {
public:
    Rectangle(const Point& a, double width, double height);
    Rectangle(const Point& a, const Point& b);
    double getWidth();
    double getHeight();
};

