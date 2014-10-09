/* 
 * File:   Path.cpp
 * Author: Gábor
 * 
 * Created on 2014. április 25., 12:06
 */

#include "Path.h"

double Path::getLength() const {
    return length;
}

bool Path::getDirection() {
    return dir;
}

void Path::setDirection(bool dir) {
    this->dir = dir;
}

Configuration Path::getStartConfig() const {
    return start;
}

Configuration Path::getEndConfig() const {
    return end;
}
