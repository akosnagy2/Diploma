/* 
 * File:   CCS.h
 * Author: Gábor
 *
 * Created on 2014. április 13., 14:19
 */

#ifndef CCS_H
#define	CCS_H

#include <ostream>

#include "Arc.h"
#include "Segment.h"

class CCS {
public:
    CCS(Arc& first, Arc& middle, Segment& end);
    Arc& getFirst();
    Arc& getMiddle();
    Segment& getLast();
    double getLength(double reversePenalty = 0);
private:
    Arc first;
    Arc middle;
    Segment end;
};

std::ostream& operator<<(std::ostream& os, CCS& ccs);

#endif	/* CCS_H */

