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

#define IMRATE 1.0

using namespace std;

int flag = 1;

queue<vector<COORDXYZT> > slamTrackVector; // short distance track vector
vector<COORDXYZTW> ENUCoorVector;    // fininal output
vector<COORDXYZTW> gps;             // total GPS track

int timetodie = 0;
void sigproc(int param)
{
    signal(SIGINT,sigproc);
    timetodie = 1;
    //printf("Got ctrl-c\n");
}

/* get GPS track from total GPS track which timestamp is correspond to slam track timestamp*/
void getGPS(vector<COORDXYZT> slamTrack,vector<COORDXYZT> &slamWithGPS,vector<COORDXYZT> &GPSWithSlam,vector<double> &weight)
{
    int i = 0;
    for(int index = 0; index < gps.size(); index ++)
    {
        if(i < slamTrack.size())
        {
            if(gps[index].t == slamTrack[i].t)
            {
                COORDXYZT temp;
                temp.x = gps[index].x;
                temp.y = gps[index].y;
                temp.z = gps[index].z;
                temp.t = gps[index].t;
                GPSWithSlam.push_back(temp);
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

// merge all short distance track
void merge(vector<COORDXYZT> slamTrack,vector<double> weight)
{
    if(ENUCoorVector.size() == 0)
    {
        for(int index = 0; index < slamTrack.size(); index ++)
        {
            COORDXYZTW temp;
            temp.x = slamTrack[index].x;
            temp.y = slamTrack[index].y;
            temp.z = slamTrack[index].z;
            temp.t = slamTrack[index].t;
            temp.w = weight[index];
            ENUCoorVector.push_back(temp);
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
            //overlap track merge
            if(ENUCoorVector[index].t == slamTrack[indexTmp].t)
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
                ENUCoorVector[index].x = ENUCoorVector[index].x * coe1 + slamTrack[indexTmp].x * coe2;
                ENUCoorVector[index].y = ENUCoorVector[index].y * coe1 + slamTrack[indexTmp].y * coe2;
                ENUCoorVector[index].z = ENUCoorVector[index].z * coe1 + slamTrack[indexTmp].z * coe2;
                ENUCoorVector[index].w = ENUCoorVector[index].w * coe1 + weight[indexTmp] * coe2;
                indexTmp ++;
                num ++;
            }
        }
        for(;indexTmp < slamTrack.size(); indexTmp ++)
        {
            COORDXYZTW temp;
            temp.x = slamTrack[indexTmp].x;
            temp.y = slamTrack[indexTmp].y;
            temp.z = slamTrack[indexTmp].z;
            temp.t = slamTrack[indexTmp].t;
            temp.w = weight[indexTmp];
            ENUCoorVector.push_back(temp);
        }
    }
}


void GPSWithWeightHandle(const gpsCalibration::IMTrackPtr& GPSWithWeight)
{
    if(GPSWithWeight->trackWithWeight.empty())
    {
        cout << "WARN: total GPS track from long_distance_track_process_node is NULL." << endl;
        exit(0);
    }
    gps = fromIMTracktoCOORDXYZTW(GPSWithWeight);        // GPS track message convert to GPS track
    cout << "gps.size() = " << gps.size() << endl;
}

void slamTrackHandle(const gpsCalibration::IMTrackPtr& slamTrack)
{
    if(slamTrack->track_flag == 1)
    {
        if(slamTrack->track.size() == 0)
        {
            flag = 0;
        }
        else
        {
            vector<COORDXYZT> slamTrackCoor = fromIMTracktoCOORDXYZT(slamTrack);
            slamTrackVector.push(slamTrackCoor);
        }
    }
}


int main(int argc,char **argv)
{
    ros::init(argc,argv,"short_distance_track_process");
    ros::NodeHandle nh;
 
    if(strcmp(argv[1],"UTM") && strcmp(argv[1],"Gaussion"))
    {
        cout << "ERROR: argv[1]=projectmethod(UTM/Gaussion)." << endl;
        return 1;
    }
    string method = argv[1];
    if(atoi(argv[2]) != 3 && atoi(argv[2]) != 6)
    {
        cout << "ERROR: argv[2]=bandwidth(3/6)" << endl;
        return 1;
    }
    int type = atoi(argv[2]);
    int KMLControl = atoi(argv[3]);

    vector<pair<double,double> > oriWGSBL,impWGSBL;
    vector<double> oriAltitude,impAltitude;
    vector<pair<int,string> > oriSegmentColor,impSegmentColor;
    GPSPro gpsProcess;
    gpsProcess.setMethod(method);
    gpsProcess.setType(type);

    ros::Subscriber subGPSWithWeight = nh.subscribe<>("gps_weight",1024,GPSWithWeightHandle);
    ros::Subscriber subSlamTrack = nh.subscribe<>("/slam_track",1024,slamTrackHandle);
    ros::Publisher IMCalibratedGPS= nh.advertise<gpsCalibration::IMMessage> ("/imorpheus_gps", 2);

    ros::Rate rate(IMRATE);   // set update rate

    signal(SIGINT,sigproc);   // handle ctrl-c signal

    while(flag)
    {
        if(timetodie)
        {
            //cout << "got ctrl-c signal." << endl;
            return 1;
        }
        rate.sleep();
        ros::spinOnce();
        if(!gps.empty() && 1 == flag)
        {
            while(!slamTrackVector.empty())
            {
                vector<COORDXYZT> GPSWithSlam,ENUCoor,slamWithGPS;
                vector<double> weight;
                vector<COORDXYZT> slamTrack = slamTrackVector.front();
                slamTrackVector.pop();
                getGPS(slamTrack,slamWithGPS,GPSWithSlam,weight);     // get GPS track
                trackCalibration tc(slamWithGPS,GPSWithSlam,weight);   // contruct icp class object
                tc.doICP();
                tc.doCalibration(ENUCoor);         // get result
                merge(ENUCoor,weight);          // merge all short distance track
            }
        }
    }

    if(timetodie)
    {
        return 1;
    }

    gpsProcess.ENUToGPS(gps,oriWGSBL,oriAltitude,oriSegmentColor);
    gpsProcess.ENUToGPS(ENUCoorVector,impWGSBL,impAltitude,impSegmentColor);

    cout << "oriWGSBL.size() = " << oriWGSBL.size() << endl;

    if(4==KMLControl)
    {
        string oriJSONFileName = argv[4];
        string impJSONFileName = argv[5];
        vector<pair<double,double> > GCJ02;

        gpsProcess.GPSToGCJ(oriWGSBL,GCJ02);
        gpsProcess.createJSON(oriJSONFileName,GCJ02,0,oriSegmentColor);

        GCJ02.clear();

        gpsProcess.GPSToGCJ(impWGSBL,GCJ02);
        gpsProcess.createJSON(impJSONFileName,GCJ02,1,impSegmentColor);     // Gaode map
    }


    if(3==KMLControl)
    {
        string oriJSONFileName = argv[4];
        string impJSONFileName = argv[5];
        vector<pair<double,double> > GCJ02,BD09;

        gpsProcess.GPSToGCJ(oriWGSBL,GCJ02);
        gpsProcess.GCJToBD(GCJ02,BD09);
        gpsProcess.createJSON(oriJSONFileName,BD09,0,oriSegmentColor);

        GCJ02.clear();
        BD09.clear();

        gpsProcess.GPSToGCJ(impWGSBL,GCJ02);
        gpsProcess.GCJToBD(GCJ02,BD09);
        gpsProcess.createJSON(impJSONFileName,BD09,1,impSegmentColor);     //Baidu map
    }


    if(1==KMLControl)     // create KML file
    {
        string originalKMLFileName= argv[4];
        string improveKMLFileName= argv[5];

        cout << "====================  Create original GPS KML  ====================" << endl;
        gpsProcess.createKML(originalKMLFileName, oriWGSBL, oriAltitude, 0, oriSegmentColor);

        cout << "==================== Create calibrated GPS KML ====================" << endl;
        gpsProcess.createKML(improveKMLFileName, impWGSBL, impAltitude, 1, impSegmentColor);

        cout << "====================            END            ====================" << endl;
        

    }
    else if(2==KMLControl)      // publish GPS track message
    {
        gpsCalibration::IMMessage msgForUser;
        gpsCalibration::IMGPS msgTemp;

        for(int i= 0; i< impWGSBL.size(); i++) 
        {
            msgTemp.l= impWGSBL[i].first;
            msgTemp.b= impWGSBL[i].second;
            msgTemp.w= ENUCoorVector[i].w;
            msgForUser.track.push_back(msgTemp);
        }
        cout << "==================== Start to publish calibrated gps ====================" << endl;
        IMCalibratedGPS.publish(msgForUser);
    }
    
    return 0;
}
