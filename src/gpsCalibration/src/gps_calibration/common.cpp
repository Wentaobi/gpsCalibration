#include "common.h"
#include <iostream>
#include "gpsCalibration/IMTrack.h"
#include "gpsCalibration/IMLocalXYZT.h"

using namespace std;

vector<COORDXYZTW> fromIMTracktoCOORDXYZTW(const gpsCalibration::IMTrack::ConstPtr& msg)
{
     COORDXYZTW temp;
     vector<COORDXYZTW> ret;
     if(msg->track.size()!=0)
     {
         for(int i=0;i<msg->track.size();++i)
         {
             temp.x=msg->track[i].x;
             temp.y=msg->track[i].y;
             temp.z=msg->track[i].z;
             temp.t=msg->track[i].t;
             ret.push_back(temp);
             //cout<<"i===-----> "<<i<<endl;
         }
     }            
     else if(msg->track.size()==0)
     {
        cerr<<"msg is null please check"<<endl;
        throw "empty msg";
     }
    return ret;
}

gpsCalibration::IMTrack fromCOORDXYZTWtoIMTrack(vector<COORDXYZTW> & arraytrack)
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
            temp.w=arraytrack[i].w;
            ret.track.push_back(temp);
        }
    }else if(arraytrack.size() == 0){
        cerr<<"array track is empty"<<endl;
        throw "empty arraytrack";
    }
    return ret;
}
