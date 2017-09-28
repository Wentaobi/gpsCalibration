#include "common.h"
#include <iostream>
#include "gpsCalibration/IMTrack.h"
#include "gpsCalibration/IMLocalXYZT.h"

using namespace std;

// IMTrack type message transform to COORDXYZT type vector
vector<COORDXYZT> fromIMTracktoCOORDXYZT(const gpsCalibration::IMTrack::ConstPtr& msg)
{
    COORDXYZT temp;
    vector<COORDXYZT> ret;
    if(msg->track.size()!=0)
    {
        for(int i=0;i<msg->track.size();++i)
        {
            temp.x=msg->track[i].x;
            temp.y=msg->track[i].y;
            temp.z=msg->track[i].z;
            temp.t=msg->track[i].t;
            ret.push_back(temp);
        }
    }
    else if(msg->track.size()==0)
    {
        cerr<<"msg is null please check"<<endl;
        throw "empty msg";
    }
    return ret;
}

// IMTrack type message transform to COORDXYZT type vector
vector<COORDXYZTW> fromIMTracktoCOORDXYZTW(const gpsCalibration::IMTrack::ConstPtr& msg)
{
     COORDXYZTW temp;
     vector<COORDXYZTW> ret;
     if(msg->trackWithWeight.size()!=0)
     {
         for(int i=0;i<msg->trackWithWeight.size();++i)
         {
             temp.x=msg->trackWithWeight[i].x;
             temp.y=msg->trackWithWeight[i].y;
             temp.z=msg->trackWithWeight[i].z;
             temp.t=msg->trackWithWeight[i].t;
             temp.w=msg->trackWithWeight[i].w;
             ret.push_back(temp);
         }
     }            
     else if(msg->trackWithWeight.size()==0)
     {
        cerr<<"msg is null please check"<<endl;
        throw "empty msg";
     }
    return ret;
}

// COORDXYZTW type vector transform to IMTrack type message
gpsCalibration::IMTrack fromCOORDXYZTWtoIMTrack(vector<COORDXYZTW> arraytrack)
{
    gpsCalibration::IMLocalXYZTW temp; 
    gpsCalibration::IMTrack ret;
    if(arraytrack.size() !=0)
    {
        for(int i=0;i<arraytrack.size();++i)
        {
            temp.x=arraytrack[i].x;
            temp.y=arraytrack[i].y;
            temp.z=arraytrack[i].z;
            temp.t=arraytrack[i].t;
            temp.w=arraytrack[i].w;
            ret.trackWithWeight.push_back(temp);
        }
    }else if(arraytrack.size() == 0){
        cerr<<"array track is empty"<<endl;
        throw "empty arraytrack";
    }
    return ret;
}

// COORDXYZT type vector transform to IMTrack type message
gpsCalibration::IMTrack fromCOORDXYZTtoTrack(vector<COORDXYZT> arraytrack)
{
    gpsCalibration::IMLocalXYZT temp;
    gpsCalibration::IMTrack ret;
    if(arraytrack.size() !=0)
    {
        for(int i=0;i<arraytrack.size();++i)
        {
            temp.x=arraytrack[i].x;
            temp.y=arraytrack[i].y;
            temp.z=arraytrack[i].z;
            temp.t=arraytrack[i].t;
            ret.track.push_back(temp);
        }
    }
    else if(arraytrack.size() == 0)
    {
        cerr<<"array track is empty"<<endl;
        throw "empty arraytrack";
    }
    return ret;
}
