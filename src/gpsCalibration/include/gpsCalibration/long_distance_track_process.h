#ifndef __LONG_DISTANCE_TRACK_PROCESS_H__
#define __LONG_DISTANCE_TRACK_PROCESS_H__
#include<iostream>
#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include<string.h>
#include<vector>
#include "common.h"
#include "gps_process.h"
#include "gpsCalibration/IMTrack.h"
#include "weight_calculation.h"
#define SPEED (2.2)
#define DELTA 0.01
using namespace std;


//, public GPSPro, public TrackICP
class LongDisTrackPro
{

    public:
        GPSPro gps;
        WeightCoeCal wcca; 
        LongDisTrackPro(string originalGPSPath1,string method1,int type1);
        //long distance loam track
        vector<COORDXYZTW> SLAMTrackTmp;
        //backup to calculate weight
        vector<COORDXYZTW> SLAMTrackTmpbackup;
        //assign SLAMTrackTmp with messgae type IMTrack from loam 
        void trackCalibrationCallback(const gpsCalibration::IMTrack::ConstPtr& msg);
        //according different condition calculate weight coefficient
        int calcICPweightCoe(int coeSort);
        //according icp_type (weight or not) assign SLAMRotatedTrackTmp and ENUOriTMP
        int ICPmatch(int icp_type);
        //get weightcoe
        vector<double> getWeightCoe(){return this->weightCoe;}
        // set or update ENUOriTMP and SLAMRotatedTrackTmp
        void initOrupdateEnuori(vector<COORDXYZTW> ENUOriTMP,int len,int type);
        void initOrupdateSLAMRotatedTrack(vector<COORDXYZTW> SLAMRotatedTrackTmp,int len,int type);
        void prepareRecalculateWeightCoe(){this->weightCoe.clear();}
        //update SLAMTrackTmp
        void updateSLAMtrackTmp(vector<COORDXYZTW> proENUTrack,int len);
        //update weightCoe when second icp
        void updateWeightCoe(vector<float> weightCoe,int len);
        //prepare merge ENUOri with weight
        void mergeENUOriwithWeight();
        //when long loam has ended write file
        int outputData();
        void msgAssign(gpsCalibration::IMTrack & msg);
    private: 
        //weighted coefficient calculation type
        int coeSort= 1; 

        //original ENU track data
        vector<COORDXYZTW> ENUOriTMP;
        //original ENU track with weight
        vector<COORDXYZTW> ENUOriFinal;
        //rotated ICP SLAM track data
        vector<COORDXYZTW> SLAMRotatedTrackTmp;

        //weight coefficient
        vector<double> weightCoe;
};
#endif 
