#ifndef STATEUTIL_H_
#define STATEUTIL_H_

#include "control/State.h"

class StateUtil
{
	int stateIndexOffset;

public:
	StateUtil();
    ~StateUtil(){}
	int findIndex(double t);
    State minState();
	State maxState();
};

extern StateUtil stateUtil;

#endif /* STATEUTIL_H_ */

