#include <ros/ros.h>
#include <stdio.h>
#include <boost/function.hpp>
#include <iomanip>

#include "common.h"
#include "gps_process.h"
#include "track_calibration.h"
#include "long_distance_track_process.h"
#include "gpsCalibration/IMTrack.h"
#include "gpsCalibration/IMLocalXYZT.h"

#define MAXITERATOR 5

int flag = 1;
ros::Publisher data_pub2;
gpsCalibration::IMTrack msgwithweight;
void LongDisTrackPro::trackCalibrationCallback(const gpsCalibration::IMTrack::ConstPtr& msg)
{
    COORDXYZTW temp;
    ROS_INFO("I subscribe %ld, and pid is: %d",msg->track.size(), (int)getpid());
    if(this)
    {
        for(int i=0;i<msg->track.size();++i)
        {
            temp.x=msg->track[i].x;
            temp.y=msg->track[i].y;
            temp.z=msg->track[i].z;
            temp.t=msg->track[i].t;
            this->SLAMTrackTmp.push_back(temp);
        }
    }
    if(this->SLAMTrackTmp.size()>0)
    {
        this->calcICPweightCoe(1);
        // return ENUgps coordinate ,this will be used ICP
        vector<COORDXYZTW> localCoor= this->gps.GPSToENU(this->SLAMTrackTmp);
        // construct icp class to complete match
        trackCalibration* trackICPhandle=new trackCalibration(this->SLAMTrackTmp,localCoor,this->getWeightCoe());
        trackICPhandle->doICP();
        //get ICP outcome
        vector<COORDXYZTW> proENUTrack;
        trackICPhandle->doCalibration(proENUTrack);
        //init ENUOri and SLAMRotated to calculate weightCoe
        this->initOrupdateEnuori(localCoor,localCoor.size(),0);
        this->initOrupdateSLAMRotatedTrack(proENUTrack,proENUTrack.size(),0);
        //recalculate weightcoe acooding loam
        this->prepareRecalculateWeightCoe();
        //localCoor.clear();
        //iterator to get proper weight
        for(int i=1;i<=MAXITERATOR;++i)
        {
            this->calcICPweightCoe(2);
            //redo icp according new weight to get new processENUtrack
            trackCalibration* trackICPhandle2=new trackCalibration(proENUTrack,localCoor,this->getWeightCoe());
            proENUTrack.clear();
            trackICPhandle2->doICP();
            trackICPhandle2->doCalibration(proENUTrack);
            
            //update SLAMTrackrOTated to recalculate GPs weight Coe
            this->initOrupdateSLAMRotatedTrack(proENUTrack,proENUTrack.size(),1);
            //this->updateSLAMtrackTmp(this->SLAMTrackTmpbackup,this->SLAMTrackTmpbackup.size());
            if(i<MAXITERATOR) this->prepareRecalculateWeightCoe();
            delete trackICPhandle2;
            trackICPhandle2= NULL;
        }
        this->mergeENUOriwithWeight();
        this->SLAMTrackTmp.clear();
        this->SLAMTrackTmpbackup.clear();
        delete trackICPhandle;
        return;
    }else if(this->SLAMTrackTmp.size()==0){
        this->msgAssign(msgwithweight);
        flag = 0;
        data_pub2.publish(msgwithweight);
    }
    return;
}

int main(int argc,char** argv)
{  
    if(argc< 4)
    {
        perror("arguments error usage argv[1]=gps_original_path argv[2]=projectmethod(UTM/Gaussion) argv[3]=bandwidth");
        return -1;
    }
    int band=atoi(argv[3]);

    ros::init(argc, argv, "listener");
    ros::NodeHandle n;

    LongDisTrackPro *longDistanceHandle= new LongDisTrackPro(argv[1],argv[2],band);

    //tell master that this node will receive topic named slam_track
    ros::Subscriber data_sub= n.subscribe("slam_track", 1000, &LongDisTrackPro::trackCalibrationCallback, longDistanceHandle);

    //register to master that this node will publish topic named gps_weight
    data_pub2= n.advertise<gpsCalibration::IMTrack>("gps_weight",2);

    while(flag)
    {
        ros::spinOnce();
    }

    return 0;
}
