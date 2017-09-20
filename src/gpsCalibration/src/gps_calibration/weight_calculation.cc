#include "weight_calculation.h"

//speed weight coefficient
int WeightCoeCal::ICPWeightCoeCal(vector<COORDXYZTW> &SLAMTrackTmp,vector<double> &weightCoe)
{
    double weightTemp;
    double disx, disy, dis;
    for(int is= 0; is< SLAMTrackTmp.size(); is++)
    {
        if(0==is)
            weightCoe.push_back(1.0);
        else
        {
            disx= SLAMTrackTmp[is+ 1].x- SLAMTrackTmp[is].x;
            disy= SLAMTrackTmp[is+ 1].y- SLAMTrackTmp[is].y;
            dis= sqrt(disx* disx+ disy* disy);
            weightTemp= min(dis/SPEED, 1.0);
            weightCoe.push_back(weightTemp);
        }
    }

    return 1;
}

//icp weight coefficient
int WeightCoeCal::ICPWeightCoeCal(vector<COORDXYZTW> &SLAMTrackTmp, vector<double> &weightCoe, vector<COORDXYZTW> &ENUOriTMP, vector<COORDXYZTW> &SLAMRotatedTrackTmp)
{
    double weightTemp;
    double disx, disy, dis;
    
    for(int is= 0; is< SLAMTrackTmp.size(); is++)
    {
        if(0==is)
            weightCoe.push_back(1.0);
        else
        {
            disx= SLAMTrackTmp[is+ 1].x- SLAMTrackTmp[is].x;
            disy= SLAMTrackTmp[is+ 1].y- SLAMTrackTmp[is].y;
            dis= sqrt(disx* disx+ disy* disy);
            weightTemp= min(dis/SPEED, 1.0);
            weightCoe.push_back(weightTemp);
        }
    }

    double maxDis, minDis;
    maxDis= -0.1;
    minDis= 100.0;
    for(int is= 0; is< ENUOriTMP.size(); is++)
    {
        disx= ENUOriTMP[is].x- SLAMRotatedTrackTmp[is].x;
        disy= ENUOriTMP[is].y- SLAMRotatedTrackTmp[is].y;
        dis= sqrt(disx* disx+ disy* disy);
        if(dis> maxDis)
            maxDis= dis;
        if(dis< minDis)
            minDis= dis;
    }
    /*
        printf("max distance is: %lf\n", maxDis);
        printf("min distance is: %lf\n", minDis);
        printf("gps track size is: %ld\n", gpsTrack.size());
        printf("weightSpeed size is: %ld\n", weightSpeed.size());
    */
    for(int is= 0; is< ENUOriTMP.size(); is++)
    {
        disx= ENUOriTMP[is].x- SLAMRotatedTrackTmp[is].x;
        disy= ENUOriTMP[is].y- SLAMRotatedTrackTmp[is].y;
        dis= sqrt(disx* disx+ disy* disy);
        //cout << setprecision(15) <<"dis: " << dis << endl;
        weightCoe[is]= weightCoe[is]* 1.0/ max(DELTA, dis);
    }
    //cout<<"========weight size========== "<<weightCoe.size()<<endl;
    return 1;
}
