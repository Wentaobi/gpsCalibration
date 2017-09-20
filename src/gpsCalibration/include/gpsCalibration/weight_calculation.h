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
        int ICPWeightCoeCal(vector<COORDXYZTW> &SLAMTrackTmp,vector<double> &weightCoe);
        //weight coefficient come from difference between ENUori and SLAMrotated
        int ICPWeightCoeCal(vector<COORDXYZTW> &SLAMTrackTmp,vector<double> &weightCoe,vector<COORDXYZTW> &ENUOriTMP, vector<COORDXYZTW> &SLAMRotatedTrackTmp);

};

#endif
