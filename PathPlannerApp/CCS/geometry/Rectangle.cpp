#include "Rectangle.h"

Rectangle::Rectangle(const Point& a, double width, double height) {
    pointList p;
    p.push_back(a);
    p.push_back(Point(a.getX() + width, a.getY()));
    p.push_back(Point(a.getX() + width, a.getY() + height));
    p.push_back(Point(a.getX(), a.getY() + height));
    this->setPoints(p);
}

Rectangle::Rectangle(const Point& a, const Point& b) {
    pointList p;
    p.push_back(a);
    p.push_back(Point(b.getX(), a.getY()));
    p.push_back(b);
    p.push_back(Point(a.getX(), b.getY()));
    this->setPoints(p);
}

double Rectangle::getHeight() {
    return points[2].y - points[0].y;
}

double Rectangle::getWidth() {
    return points[2].x - points[0].y;
}
