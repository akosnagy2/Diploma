#include "Segment.h"

#include <vector>
#include <algorithm>
#include <float.h>
#include <typeinfo>

#include "misc.h"
#include "Vector.h"
#include "Arc.h"
#include "Circle.h"

Segment::Segment(const Point& p1, const Point& p2, bool dir) {
    this->dir = dir;
    this->length = Point::distance(p1, p2);
	this->radius = INFINITY;
    Vector v(p1, p2);
    double angle = v.getFi();
    if (dir) {
        start = Configuration(p1, angle);
        end = Configuration(p2, angle);
    } else {
        start = Configuration(p1, wrapAngle(angle + M_PI));
        end = Configuration(p2, wrapAngle(angle + M_PI));
    }
}

Segment::Segment(const Configuration& start, const Point& end)
{
	Configuration c(start.position, atan2(end.y - start.position.y, end.x - start.position.x));
	this->start = c;
	c.position = end;
	this->end = c;
	this->dir = (abs(wrapAngle(start.orientation - c.orientation)) < M_PI_2);
	this->length = Point::distance(start.position, end);
	this->radius = INFINITY;
}

Segment::Segment(const Configuration& start, const Configuration& end, bool dir) {
    this->start = start;
    this->end = end;
    this->dir = dir;
    this->length = Point::distance(start.position, end.position);
	this->radius = INFINITY;
}

bool Segment::isIntersect(const Segment& s) const {
    Point tempPoint;
    return isIntersect(s, tempPoint);
}

bool Segment::isIntersect(const Segment& s, Point& p) const {
    Point a = start.position;
    Point b = end.position;

    Point c = s.start.position;
    Point d = s.end.position;

    double A = b.y - a.y;
    double B = b.x - a.x;
    double D = d.y - c.y;
    double E = d.x - c.x;

    if (fabs(A * E - D * B) == 0) {
        /* If parallel segemnts */
        return false;
    }

    double x = (A * E * a.x - B * E * a.y - D * B * c.x + B * E * c.y) / (A * E - D * B);
    double y = (B * D * a.y - A * D * a.x - E * A * c.y + A * D * c.x) / (B * D - E * A);

    bool c1 = (fabs(x - a.x) + fabs(x - b.x) - fabs(B)) < 0.001;
    bool c2 = (fabs(y - a.y) + fabs(y - b.y) - fabs(A)) < 0.001;
    bool c3 = (fabs(x - c.x) + fabs(x - d.x) - fabs(E)) < 0.001;
    bool c4 = (fabs(y - c.y) + fabs(y - d.y) - fabs(D)) < 0.001;

    bool c5 = (fabs(x - a.x) > 0.001) || (fabs(y - a.y) > 0.001);
    bool c6 = (fabs(x - b.x) > 0.001) || (fabs(y - b.y) > 0.001);
    bool c7 = (fabs(x - c.x) > 0.001) || (fabs(y - c.y) > 0.001);
    bool c8 = (fabs(x - d.x) > 0.001) || (fabs(y - d.y) > 0.001);

    if (c1 && c2 && c3 && c4 && c5 && c6 && c7 && c8) {
        p.x = x;
        p.y = y;
        return true;
    }

    return false;
}

bool Segment::isProjectedInside(const Point& p, Point& projection) const {
    double length = this->getLength();
    Point a = start.position;
    Point b = end.position;

    Vector v1(p - a);
    Vector v2(b - a);
    double scalar = v1*v2;
    scalar /= length;

    projection = a + (b - a) * scalar / length;

    bool insideX = false;
    if ((projection.x >= a.x && projection.x <= b.x) || (projection.x <= a.x && projection.x >= b.x))
        insideX = true;

    bool insideY = false;
    if ((projection.y >= a.y && projection.y <= b.y) || (projection.y <= a.y && projection.y >= b.y))
        insideY = true;

    if (insideX && insideY)
        return true;

    return false;
}

double Segment::getDistance(const Segment& s) const {
    if (this->isIntersect(s))
        return 0;

    std::vector<double> dist(4, 0);

    Point a = start.position;
    Point b = end.position;

    Point c = s.start.position;
    Point d = s.end.position;

    Point projection;
    /* distance between this and c */
    if (this->isProjectedInside(c, projection)) {
        dist[0] = Point::distance(c, projection);
    } else {
        dist[0] = Point::distance(c, a);
        double tmp = Point::distance(c, b);
        if (dist[0] > tmp)
            dist[0] = tmp;
    }

    /* distance between this and d */
    if (this->isProjectedInside(d, projection)) {
        dist[1] = Point::distance(d, projection);
    } else {
        dist[1] = Point::distance(d, a);
        double tmp = Point::distance(d, b);
        if (dist[1] > tmp)
            dist[1] = tmp;
    }

    /* distance between s and a */
    if (s.isProjectedInside(a, projection)) {
        dist[2] = Point::distance(a, projection);
    } else {
        dist[2] = Point::distance(a, c);
        double tmp = Point::distance(a, d);
        if (dist[2] > tmp)
            dist[2] = tmp;
    }

    /* distance between s and b */
    if (s.isProjectedInside(b, projection)) {
        dist[3] = Point::distance(b, projection);
    } else {
        dist[3] = Point::distance(b, c);
        double tmp = Point::distance(b, d);
        if (dist[3] > tmp)
            dist[3] = tmp;
    }

    return *std::min_element(dist.begin(), dist.end());
}

Point Segment::getA() const {
    return start.position;
}

Point Segment::getB() const {
    return end.position;
}

double Segment::getOrientation() const {
    return start.orientation;
}

configurationList Segment::getPoints(double dr) {
    configurationList points;
    //double conf = start.orientation;
	double conf = atan2(end.position.y - start.position.y, end.position.x - start.position.x);
    Point startPoint = start.position;
    //dr *= dir ? 1 : -1;
    double dx = dr * cos(conf);
    double dy = dr * sin(conf);
    for (int i = 0; fabs(i * dr) < length; i++) {
        Point p(i*dx, i * dy);
        points.push_back(Configuration(startPoint + p, conf));
    }
    //points.push_back(end);
	points.push_back(Configuration(end.position, conf));
    return points;
}

Segment& Segment::translateToObstacleCornerArc(const Point& pos) {
    end.position = pos - end.position + start.position;
    start.position = pos;
    return *this;
}

Segment& Segment::translateToRobotCornerArc(const Point& pos) {
    start.position = start.transform(pos);
    end.position = end.transform(pos);
    return *this;
}

Segment* Segment::copy() {
    return new Segment(*this);
}

bool Segment::isIntersect(const Arc& arc) const {
    Configuration arcStart(arc.getStartConfig());
    Configuration arcEnd(arc.getEndConfig());
    if (isinf(arc.getRadius())) {
        /* Straight segment */
        Segment seg(arcStart.position, arcEnd.position);
        return isIntersect(seg);
    } else {
        /* Circular segment */
        Point center(arc.getCenter());
        double radius = arc.getRadius();
        Circle circle(center, radius);
        pointList intersections = circle.getIntersections(Line(*this));
        bool intersect = false;
        for (unsigned i = 0; i < intersections.size() && !intersect; i++) {
            Point pTemp = intersections[i];
            double startAngle = wrapAngle(arcStart.orientation - sgn(radius) * M_PI_2);
            double isPointAngle = atan2(pTemp.y - center.y, pTemp.x - center.x);
            double checkAngle = isPointAngle - startAngle;
            double dTheta = arc.getDTheta();
            
            if (sgn(checkAngle) != sgn(dTheta))
                checkAngle = sgn(dTheta)*(2 * M_PI + sgn(dTheta) * checkAngle);

            if (fabs(dTheta) >= fabs(checkAngle)) {
                double d1 = Point::distance(start.position, pTemp);
                double d2 = Point::distance(end.position, pTemp);
                if (almostEqualRelative(d1 + d2, length, DBL_EPSILON * 10))
                    intersect = true;
            }
        }
        return intersect;
    }
}

bool Segment::isIntersect(Path& path) const {
    try {
        Arc& arc = dynamic_cast<Arc&>(path);
        return isIntersect(arc);
    } catch(std::bad_cast& bc) {
        Segment& seg = dynamic_cast<Segment&>(path);
        return isIntersect(seg);
    }   
}