#ifndef _WEIGHT_CALCULATION_H_
#define _WEIGHT_CALCULATION_H_

#include "common.h"

#define SPEED (2.2)
#define DELTA 0.01

using namespace std;

class WeightCoeCal
{
    public:
        //weight coefficient come from slam speed
        int ICPWeightCoeCal(vector<COORDXYZT> &SLAMTrackTmp,vector<double> &weightCoe);
        //weight coefficient come from difference between ENUori and SLAMrotated
        int ICPWeightCoeCal(vector<COORDXYZT> &SLAMTrackTmp,vector<double> &weightCoe,vector<COORDXYZT> &ENUOriTMP, vector<COORDXYZT> &SLAMRotatedTrackTmp);

};

#endif
