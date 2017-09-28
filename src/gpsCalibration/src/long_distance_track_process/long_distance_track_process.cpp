#include <ros/ros.h>
#include <stdio.h>
#include <boost/function.hpp>

#include "common.h"
#include "gps_process.h"
#include "track_calibration.h"
#include "weight_calculation.h"
#include "gpsCalibration/IMTrack.h"
#include "gpsCalibration/IMLocalXYZT.h"

#define MAXITERATOR 5
#define IMRATE 1.0

ros::Publisher data_pub2;    // publish message handle
gpsCalibration::IMTrack msgWithWeight;    // total GPS track message
vector<COORDXYZTW> totalTrack;       // total GPS track
GPSPro gpsProcess;        // gps process class

// merge all GPS track
void merge(vector<COORDXYZT> localCoor,vector<double> weightCoe)
{
    if(localCoor.size() != weightCoe.size())
    {
        cout << "ERROR: ENU coordinate size is not equal to weight size" << endl;
        exit(-1);
    }
    for(int index = 0; index < localCoor.size(); index ++)    // merge slam track
    {
        COORDXYZTW tmp;
        tmp.x = localCoor[index].x;
        tmp.y = localCoor[index].y;
        tmp.z = localCoor[index].z;
        tmp.t = localCoor[index].t;
        tmp.w = weightCoe[index];
        totalTrack.push_back(tmp);
    }
    
}

void longDisTrackPro(const gpsCalibration::IMTrack::ConstPtr& msg)
{
    if(msg->track.empty())
    {
        if(totalTrack.empty())
        {
            cout << "WARN: no GPS track,please check it" << endl;
            exit(-1);
        }
        msgWithWeight = fromCOORDXYZTWtoIMTrack(totalTrack);      // convert GPS track to GPS track message
        data_pub2.publish(msgWithWeight);               // publish total GPS track to short_distance_track_process_node
    }
    else
    {
        vector<COORDXYZT> SLAMTrackTmp = fromIMTracktoCOORDXYZT(msg);   // slam track message convert to slam track
        WeightCoeCal wcca;
        vector<double> weightCoe;
        wcca.ICPWeightCoeCal(SLAMTrackTmp,weightCoe);      // calculate weight according to car speed 

        vector<COORDXYZT> localCoor = gpsProcess.GPSToENU(SLAMTrackTmp);     // return ENUgps coordinate,this will be used ICP

        //construct icp class to complete match
        trackCalibration trackICPHandle(SLAMTrackTmp,localCoor,weightCoe);
        trackICPHandle.doICP();
        //get ICP result
        vector<COORDXYZT> proENUTrack;
        trackICPHandle.doCalibration(proENUTrack);

        // iteration
        for(int i = 1; i <= MAXITERATOR; i ++)
        {
            weightCoe.clear();

            wcca.ICPWeightCoeCal(SLAMTrackTmp,weightCoe,localCoor,proENUTrack);   // update weight according to ICP

            trackCalibration trackICPHandle2(proENUTrack,localCoor,weightCoe);
            proENUTrack.clear();
            trackICPHandle2.doICP();
            trackICPHandle2.doCalibration(proENUTrack);           // get ICP result
        }
        merge(localCoor,weightCoe);       // merge slam track
        SLAMTrackTmp.clear();

    }
}

int main(int argc,char **argv)
{
    if(argc < 4)
    {
        perror("ERROR: arguments error usage argv[1]=gps_original_path argv[2]=projectmethod(UTM/Gaussion) argv[3]=bandwidth(3/6)");
        return -1;
    }

    if(strcmp(argv[2],"UTM") && strcmp(argv[2],"Gaussion"))    // UTM method or Gaussion method
    {
        perror("ERROR: argv[2]=projectmethod(UTM/Gaussion)");
        return -1;
    }

    if(atof(argv[3]) != 3 && atof(argv[3]) != 6)
    {
        perror("ERROR: argv[3]=bandwidth(3/6)");
        return -1;
    }
    string originalGPS = argv[1];
    string method = argv[2];
    int type = atof(argv[3]);

    //init gps process
    gpsProcess.setGPSPath(originalGPS);
    gpsProcess.setMethod(method);
    gpsProcess.setType(type);

    ros::init(argc,argv,"long_distance_track_process");
    ros::NodeHandle nh;

    ros::Rate rate(IMRATE);    // set update rate

    ros::Subscriber data_sub = nh.subscribe("slam_track",1000,longDisTrackPro);
    data_pub2 = nh.advertise<gpsCalibration::IMTrack>("gps_weight",2);

    //ros::spin();
    
    while(1)
    {
        rate.sleep();
        ros::spinOnce();
    }
    

    cout << "long distance track finished." << endl;
    return 0;
}
