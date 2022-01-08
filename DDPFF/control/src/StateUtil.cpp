#include "control/StateUtil.h"
#include "control/State.h"
#include "globals/Config.h"

StateUtil stateUtil;

StateUtil::StateUtil()
{
	stateIndexOffset = 0;
}


// Returns the floor state index for the given time t.
// This is needed to find the state history index for a real time t.
int StateUtil::findIndex(double t)
{
    int stateIndex = qBound(1, int( double(curState.size()-1) - t/config.systemIterationTime + stateIndexOffset), curState.size()-1);
    while (stateIndex < curState.size()-1 && curState[stateIndex].time > t)
	{
		stateIndex++;
		stateIndexOffset++;
	}
    while (stateIndex > 1 && curState[stateIndex-1].time <= t)
	{
		stateIndex--;
		stateIndexOffset--;
	}

	return stateIndex;
}

// Finds the minimum of all state members over the currently loaded state history.
State StateUtil::minState()
{
    State min = curState[0];
    for (int i = curState.size(); i > 0; i--){
        for (int j = 0; j < curState.memberNames.size(); j++){
            if (min.getMember(j) > curState[i].getMember(j)){
                min.setMember(j, curState[i].getMember(j));
            }
        }
    }
	return min;
}

// Finds the maximum of all state members over the currently loaded state history.
State StateUtil::maxState()
{
    State max = curState[0];
    for (int i = curState.size(); i > 0; i--){
        for (int j = 0; j < curState.memberNames.size(); j++){
            if (max.getMember(j) < curState[i].getMember(j)){
                max.setMember(j, curState[i].getMember(j));
            }
        }
    }
	return max;
}

