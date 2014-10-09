/* 
 * File:   CCS.cpp
 * Author: Gábor
 * 
 * Created on 2014. április 13., 14:19
 */

#include "CCS.h"
#include "misc.h"

CCS::CCS(Arc& first, Arc& middle, Segment& end)
: first(first)
, middle(middle)
, end(end){
}

Arc& CCS::getFirst() {
    return first;
}

Segment& CCS::getLast() {
    return end;
}

Arc& CCS::getMiddle() {
    return middle;
}

std::ostream& operator<<(std::ostream& os, CCS& ccs) {
    Arc first = ccs.getFirst();
    Arc middle = ccs.getMiddle();
    Segment end = ccs.getLast();
    
    os << "                   C1         C2         S" << std::endl;
    os << "------------------------------------------" << std::endl;
    os << "r(1:2)     :    " << first.getRadius() << "   " << middle.getRadius() << "   " << std::endl;
    os << "dtheta(1:2): " << r2d(first.getDTheta()) << "o   " << r2d(middle.getDTheta()) << "o   " << std::endl;
    os << "dist(1:3)  :  " << first.getLength() << "    " << middle.getLength() << "  " << end.getLength() << std::endl;
    os << "dir(1:3)   :        " << first.getDirection() << "          " << middle.getDirection() << "         " << ccs.getLast().getDirection() << std::endl;
    
    return os;
}

double CCS::getLength(double reversePenalty) {
    double distance = 0;

    if (first.getDirection()) {
        distance += first.getLength();
    } else {
        distance += first.getLength() * reversePenalty;
    }

    if (middle.getDirection()) {
        distance += middle.getLength();
    } else {
        distance += middle.getLength() * reversePenalty;
    }

    if (end.getDirection()) {
        distance += end.getLength();
    } else {
        distance += end.getLength() * reversePenalty;
    }

    return distance;
}
