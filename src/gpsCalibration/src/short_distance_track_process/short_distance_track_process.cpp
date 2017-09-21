#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <queue>
#include <sys/signal.h>
#include "common.h"
#include "gps_process.h"
#include "track_calibration.h"
#include "gpsCalibration/IMTrack.h"
#include "gpsCalibration/IMLocalXYZT.h"
#include "gpsCalibration/IMMessage.h"
#include "gpsCalibration/IMGPS.h"

using namespace std;

int flag = 1;

queue<vector<COORDXYZTW> > slamTrackVector;
vector<COORDXYZTW> ENUCoorVector;
vector<COORDXYZTW> gps;

int timetodie = 0;
void sigproc(int param)
{
    signal(SIGINT,sigproc);
    timetodie = 1;
    printf("Got ctrl-c\n");
}


void getGPS(vector<COORDXYZTW> slamTrack,vector<COORDXYZTW> &slamWithGPS,vector<COORDXYZTW> &GPSWithSlam,vector<double> &weight)
{
    int i = 0;
    for(int index = 0; index < gps.size(); index ++)
    {
        if(i < slamTrack.size())
        {
            if(gps[index].t == slamTrack[i].t)
            {
                GPSWithSlam.push_back(gps[index]);
                weight.push_back(gps[index].w);
                slamWithGPS.push_back(slamTrack[i]);
            }
            else if(gps[index].t > slamTrack[i].t)
            {
                i++;
                index--;
            }
        }
        else
        {
            break;
        }
        
    }
}

void merge(vector<COORDXYZTW> tmp)
{
    if(ENUCoorVector.size() == 0)
    {
        for(int index = 0; index < tmp.size(); index ++)
        {
            ENUCoorVector.push_back(tmp[index]);
        }
    }
    else
    {
        int indexTmp = 0;
        
        int num = 1;
        double coe1 = 0.0,coe2 = 0.0;
        int smWindow = -1;
        int opNo = -1;
        for(int index = 0; index < ENUCoorVector.size(); index ++)
        {
            if(ENUCoorVector[index].t == tmp[indexTmp].t)
            {
                if(-1 == opNo)
                {
                    opNo = ENUCoorVector.size() - index;
                    smWindow = opNo / 2;
                }
                if(num <= smWindow)
                {
                    coe1 = (1.0 - num / (2.0 * smWindow));
                    coe2 = num / (2.0 * smWindow);
                }
                else if((num > smWindow) && (num <= opNo - smWindow))
                {
                    coe1 = 0.5;
                    coe2 = 0.5;
                }
                else if(num > (opNo - smWindow))
                {
                    coe1 = (opNo - num + 1) / (2.0 * smWindow);
                    coe2 = (1.0 - (opNo - num + 1) / (2.0 * smWindow));
                }
                ENUCoorVector[index].x = ENUCoorVector[index].x * coe1 + tmp[indexTmp].x * coe2;
                ENUCoorVector[index].y = ENUCoorVector[index].y * coe1 + tmp[indexTmp].y * coe2;
                ENUCoorVector[index].z = ENUCoorVector[index].z * coe1 + tmp[indexTmp].z * coe2;
                ENUCoorVector[index].w = ENUCoorVector[index].w * coe1 + tmp[indexTmp].w * coe2;
                indexTmp ++;
                num ++;
            }
        }
        for(;indexTmp < tmp.size(); indexTmp ++)
        {
            ENUCoorVector.push_back(tmp[indexTmp]);
        }
    }
}


void GPSWithWeightHandle(const gpsCalibration::IMTrackPtr& GPSWithWeight)
{
    for(int index = 0; index < GPSWithWeight->track.size(); index ++)
    {
        COORDXYZTW tmp;
        tmp.x = GPSWithWeight->track[index].x; 
        tmp.y = GPSWithWeight->track[index].y;
        tmp.z = GPSWithWeight->track[index].z;
        tmp.t = GPSWithWeight->track[index].t;
        tmp.w = GPSWithWeight->track[index].w;
        gps.push_back(tmp);
    }
}

void slamTrackHandle(const gpsCalibration::IMTrackPtr& slamTrack)
{
    if(slamTrack->track_flag == 1)
    {
        if(slamTrack->track.size() == 0)
        {
            flag = 0;
            timetodie = 1;
        }
        else
        {
            vector<COORDXYZTW> slamTrackCoor;

            for(int index = 0; index < slamTrack->track.size(); index ++)
            {
                COORDXYZTW tmp;
                tmp.x = slamTrack->track[index].x;
                tmp.y = slamTrack->track[index].y;
                tmp.z = slamTrack->track[index].z;
                tmp.t = slamTrack->track[index].t;
                slamTrackCoor.push_back(tmp);
            }   
            slamTrackVector.push(slamTrackCoor);
        }
    }
}


int main(int argc,char **argv)
{
    ros::init(argc,argv,"gpsCalibration");
    ros::NodeHandle nh;
  
    string method = argv[1];
    int type = atoi(argv[2]);
    int KMLControl = atoi(argv[3]);

    vector<pair<double,double> > WGSBL;
    vector<double> altitude;
    vector<pair<int,string> > segmentColor;
    GPSPro gpsProcess;
    gpsProcess.setMethod(method);
    gpsProcess.setType(type);

    ros::Subscriber subGPSWithWeight = nh.subscribe<>("gps_weight",1024,GPSWithWeightHandle);
    ros::Subscriber subSlamTrack = nh.subscribe<>("/slam_track",1024,slamTrackHandle);
    ros::Publisher IMCalibratedGPS= nh.advertise<gpsCalibration::IMMessage> ("/imorpheus_gps", 2);

    signal(SIGINT,sigproc);

    while(!timetodie)
    {
        while(flag)
        {
            ros::spinOnce();
            if(!gps.empty() && 1 == flag)
            {
                while(!slamTrackVector.empty())
                {
                    vector<COORDXYZTW> GPSWithSlam,ENUCoor,slamWithGPS;
                    vector<double> weight;
                    vector<COORDXYZTW> slamTrack = slamTrackVector.front();
                    slamTrackVector.pop();
                    getGPS(slamTrack,slamWithGPS,GPSWithSlam,weight);
                    trackCalibration tc(slamWithGPS,GPSWithSlam,weight);
                    tc.doICP();
                    tc.doCalibration(ENUCoor);
                    merge(ENUCoor);
                }
            }
        }

        if(1==KMLControl)
        {
            string originalKMLFileName= argv[4];
            string improveKMLFileName= argv[5];

            cout << "==================== Create original GPS KML ====================" << endl;
            gpsProcess.ENUToGPS(gps, WGSBL, altitude, segmentColor);
            gpsProcess.createKML(originalKMLFileName, WGSBL, altitude, 0, segmentColor);
            WGSBL.clear();
            altitude.clear();
            segmentColor.clear();

            cout << "==================== Create calibrated GPS KML ====================" << endl;
            gpsProcess.ENUToGPS(ENUCoorVector, WGSBL, altitude, segmentColor);
            gpsProcess.createKML(improveKMLFileName, WGSBL, altitude, 1, segmentColor);

            cout << "==================== END ====================" << endl;
        }
        else if(2==KMLControl)
        {
            gpsCalibration::IMMessage msgForUser;
            gpsCalibration::IMGPS msgTemp;

            for(int i= 0; i< WGSBL.size(); i++) 
            {
                msgTemp.b= WGSBL[i].first;
                msgTemp.l= WGSBL[i].second;
                msgTemp.w= ENUCoorVector[i].w;
                msgForUser.track.push_back(msgTemp);
            }
            cout << "==================== Start to publish calibrated gps ====================" << endl;
            IMCalibratedGPS.publish(msgForUser);
        }
    }
    return 0;
}
