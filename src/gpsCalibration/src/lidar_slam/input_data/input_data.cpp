#include <ros/ros.h> 
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Header.h>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <sensor_msgs/PointCloud2.h>
#include <boost/foreach.hpp>
#include <queue>
#include <map>
#include <stdlib.h>
#include <signal.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <stdio.h>
#include <errno.h>
#include <cmath>
#include <nav_msgs/Odometry.h>

#include "gpsCalibration/IMTrack.h"
#include "gpsCalibration/IMLocalXYZT.h"
#include "gpsCalibration/IMControl.h"

#define foreach BOOST_FOREACH
#define IMSDLEN 512
#define IMREST 3.0
#define IMRATE 1.0

using namespace std;

long g_lPublishedMsgNum = 0; //published message number in long or short track
long g_lTotalMsgNum = 0; //all the bags message number
int times = 0;
int g_nBagIndex = 0;     // bag index
int g_nMsgIndex = 0;         // message index of each bag
static int fileNum = 0;    // track index
static double g_dSlamDistance[2];
static double g_dOverlapDistance[2];
static double g_dTotalDistane = 0; //total distance in one cycle
vector<string> bagList;   // bag path
static unsigned long nMsgSendNum = 0;    //all sent messages number in one cycle
static unsigned long nMsgLostNum = 0;    //all loss messages number in one cycle

vector<std::string> tempTopics;  //message topic
vector<sensor_msgs::PointCloud2> cloudTopics;
nav_msgs::Odometry::ConstPtr preOdometry;              //subscribe message
rosbag::Bag readBag;       //read bag handler
sensor_msgs::PointCloud2::ConstPtr pointcloud2;          //publish message
gpsCalibration::IMTrack slamTrack;
queue<gpsCalibration::IMTrack> slamTrackVector;

struct DISTANCE
{
    int nBagIndex;
    int nMsgIndex;
    double distance;
    double timestamp;
};

vector<DISTANCE> allLocation;   // all publish location
DISTANCE pubLocation;        //overlap location

// handle ctrl-c signal
int timetodie = 0;
void sigproc(int param)
{
    signal(SIGINT,sigproc);
    timetodie = 1;
    //printf("Got ctrl-c \n");
}

//subscribe handler
void subOdometryHandler(const nav_msgs::Odometry::ConstPtr& subOdometry)
{
    if(subOdometry->header.stamp.toSec()==pointcloud2->header.stamp.toSec())    //make sure publish message and receive odometry
    {
        gpsCalibration::IMLocalXYZT localCoor;
        localCoor.x = subOdometry->pose.pose.position.x;
        localCoor.y = subOdometry->pose.pose.position.y;
        localCoor.z = subOdometry->pose.pose.position.z;
        localCoor.t = subOdometry->header.stamp.toSec();
        slamTrack.track.push_back(localCoor);

        DISTANCE tmpdis = {0,0,0,0};    // init tmpdis
        tmpdis.nBagIndex = g_nBagIndex;
        tmpdis.nMsgIndex = g_nMsgIndex;
        if(NULL == preOdometry)
        {
            tmpdis.distance = 0;
        }
        else
        {
            tmpdis.distance = sqrt(pow(subOdometry->pose.pose.position.x - preOdometry->pose.pose.position.x,2)
                              + pow(subOdometry->pose.pose.position.y - preOdometry->pose.pose.position.y,2)
                              + pow(subOdometry->pose.pose.position.z - preOdometry->pose.pose.position.z,2))
                              + g_dTotalDistane;
        }
        tmpdis.timestamp = (double)subOdometry->header.stamp.toSec();
        preOdometry = subOdometry;    // update preOdometry
    
        if(tmpdis.distance <= g_dSlamDistance[times] - g_dOverlapDistance[times])    //next publish message location
        {
            pubLocation = tmpdis;
        }
        else
        {
            if(allLocation[allLocation.size()-1].timestamp != pubLocation.timestamp) 
            {
                allLocation.push_back(pubLocation);                  // push next publish message location into vector allLocation
            }
        }
        g_dTotalDistane = tmpdis.distance;
    }
    else
    {
        nMsgLostNum++;
        cout<<" WARN: publish message to loam,but not receive odometry"<<endl;
    }
}

//read baglist.txt to get bag location
void readBagList(char *fileName,vector<string> &bagList)
{
    //open baglist.txt
    ifstream ifile;
    ifile.open(fileName);
    if(!ifile.is_open())
    {
        printf("ERROR: open %s error,please check it\n",fileName);
        exit(0);
    }
    //read file
    char buf[IMSDLEN];
    while(ifile.getline(buf,IMSDLEN))
    {
        if(strcmp("",buf))
        {
            bagList.push_back(buf);
        }
    }
    ifile.close();

    if(bagList.empty())
    {
         //published message number in long or short trackprintf("WARN:%s is NULL,please check it.\n",fileName);
        exit(0);
    }
}

/******************************************************************************
*Description: Get bag list infomation,include all bags message number and each
*             bag message number
*return:      all bags message number
******************************************************************************/
long getBagMessageInfo(vector<string> bagList,vector<long> &messageNumber)
{
    long totalNum = 0;             // total message number
    long messageNumPerBag = 0;

    tempTopics.push_back("velodyne_points");
    
    try
    {
        for(int index = 0;index < bagList.size();index ++)
        {
            messageNumPerBag = 0;        // message number per bag
            readBag.open(bagList[index].c_str());
            rosbag::View view(readBag,rosbag::TopicQuery(tempTopics));
            foreach(rosbag::MessageInstance const m,view)
            {
                totalNum ++;
                messageNumPerBag ++;
            }
            readBag.close();
            messageNumber.push_back(messageNumPerBag);
        }
        cout<<"Total Message Num Is "<<totalNum<<endl;
    }
    catch(rosbag::BagIOException)
    {
        cout<<"ERROR: Cannot find bag path, Please check bag_list"<<endl;
        exit(0);
    }
    return totalNum;
}

/*
*show message sent number,message loss number,message loss rate infomation
*/
void showMessageHandleResult()
{
    if (0 != nMsgSendNum)
    {
        float rate = (nMsgLostNum * 100) / nMsgSendNum;
        if (rate < 0.01)
        {
            printf("[INFO]In one cycle,send message:%lu,lost message:%lu,message loss rate < 0.01%%\n", nMsgSendNum, nMsgLostNum);
        }
        else
        {
            printf("[INFO]In one cycle,send message:%lu,lost message:%lu,message loss rate:%.2f%%\n", nMsgSendNum, nMsgLostNum, rate);
        }
    }
}

/*
*show message publish infomation
*/
void showMessagePublishInfo()
{
    system("clear");
    cout<<"***********************publish message***********************"<<endl;
    printf("published %ld message,%ld message left,maybe %.1lf minites to publish\n",g_lPublishedMsgNum,g_lTotalMsgNum - g_lPublishedMsgNum,(g_lTotalMsgNum - g_lPublishedMsgNum) / IMRATE / 60.0);
}

/*
argv[1]: baglist.txt path
argv[2]: slam odometry path
argv[3]: slam distance
argv[4]: overlap slam distance
*/
int main(int argc, char **argv)  
{
    if(6 > argc)
    {
        printf("ERROR: parameter error\n");
        return 1;
    }
    char *bagListPath = argv[1];
    if(atof(argv[2]) > atof(argv[3]) && atof(argv[3]) > atof(argv[4]) && atof(argv[4]) > 0)
    {
        g_dSlamDistance[0] = atof(argv[2]);
        g_dOverlapDistance[0] = 0;
        g_dSlamDistance[1] = atof(argv[3]);
        g_dOverlapDistance[1] = atof(argv[4]);
    }
    else
    {
        printf(" WARN: please make sure long distance is larger than short distance and short distance is larger than overlap distance.\n");
        return 1;
    }
    char buf[IMSDLEN];

    ros::init(argc,argv,"input_data");
    ros::NodeHandle nh;
    ros::Rate rate(IMRATE);    // set update rate
    rate.sleep();

    readBagList(bagListPath,bagList);

    vector<long> messageNumber;             // message number per bag
    g_lTotalMsgNum = getBagMessageInfo(bagList,messageNumber);   // total message number

    ros::Publisher pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points",1024);      // publish pointcloud message
	ros::Publisher slamTrackPub = nh.advertise<gpsCalibration::IMTrack>("/slam_track",2);
    ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry>("/true_odometry_to_init",1024,subOdometryHandler);      //receive slam odometry
    ros::Publisher pubControl = nh.advertise<gpsCalibration::IMControl>("/control_command",2);

    signal(SIGINT,sigproc);       // handle ctrl-c signal

    for(times = 0; times < 2; times ++)
    {
        // init publocation
        pubLocation.nBagIndex = 0;
        pubLocation.nMsgIndex = 0;
        pubLocation.distance = 0;
        pubLocation.timestamp = 0;
        allLocation.push_back(pubLocation);
        
        g_nBagIndex = 0;
        g_dTotalDistane = 0;
        nMsgSendNum = 0;
        nMsgLostNum = 0;

        //init IMControl message
        gpsCalibration::IMControl controlMsg;
        controlMsg.systemInited = false;
        pubControl.publish(controlMsg);
        rate.sleep();

	    while(g_nBagIndex < bagList.size())
	    {
            int end = 0;   // flag for end publish 
            char bagName[IMSDLEN];     // bag path

            //firstMessage = 1;  // init firstMessage
            g_nBagIndex = allLocation[allLocation.size()-1].nBagIndex;   // init g_nBagIndex

            g_lPublishedMsgNum = 0;
            for(int i = 0;i< g_nBagIndex;i++)
            {
                g_lPublishedMsgNum += messageNumber[i];
            }
            g_lPublishedMsgNum += pubLocation.nMsgIndex;    // init message index

            showMessagePublishInfo();

            while(g_nBagIndex < bagList.size())
            {
                sprintf(bagName,"%s",bagList[g_nBagIndex].c_str());
                readBag.open(bagName,rosbag::bagmode::Read);           //open bag
                tempTopics.push_back("velodyne_points");
                rosbag::View view(readBag,rosbag::TopicQuery(tempTopics));
 
                g_nMsgIndex = 0;          // init messag index
                //publish message
                foreach(rosbag::MessageInstance const m,view)
                {
                    g_nMsgIndex++;
                    pointcloud2 = m.instantiate<sensor_msgs::PointCloud2>();
                    
                    if(timetodie == 1)   // get ctrl-c signal,return;
                    {
                        readBag.close();
                        //cout << "got ctrl-c signal." << endl;
                        return 1; 
                    }
                 
                    if(pubLocation.nMsgIndex < g_nMsgIndex || pubLocation.nBagIndex < g_nBagIndex)    // find location to publish messge
                    {
                        pubLaserCloud.publish(pointcloud2);        //publish message
                        g_lPublishedMsgNum ++;
                        nMsgSendNum++;
                        if(g_lPublishedMsgNum % 50 == 0)
                        {
                            showMessagePublishInfo();
                        }
                        rate.sleep();                // publish rate
                        ros::spinOnce();       //calculate track distance
                        if(g_dTotalDistane > g_dSlamDistance[times])    // stop publish message
                        {
                            g_dTotalDistane = 0;
                            showMessageHandleResult();
                            nMsgSendNum = 0;
                            nMsgLostNum = 0;
                            end = 1;
                            break;
                        }
                    }
                }
                readBag.close();
                if(end == 1)
                {
                    pubControl.publish(controlMsg);
                    rate.sleep();
                    break;
                }
                ++g_nBagIndex;
            }
            slamTrackVector.push(slamTrack);
            slamTrack.track.clear();
            preOdometry = NULL;
            if(3 == slamTrackVector.size())
            {
                slamTrackPub.publish(slamTrackVector.front());
                slamTrackVector.pop();
            }
        }       

        //if rest distance is too short,then add to previous file
        if(allLocation.size() > 1 && g_dTotalDistane < g_dSlamDistance[times] / IMREST)
        {
            DISTANCE tmp = allLocation[allLocation.size()-2];      //publish location
            
            g_lPublishedMsgNum = 0;
            for(int i = 0;i < tmp.nBagIndex;i++)
            {
                g_lPublishedMsgNum += messageNumber[i];
            }
            g_lPublishedMsgNum += tmp.nMsgIndex;

            while(!slamTrackVector.empty())
            {
                slamTrackVector.pop();
            }
            slamTrack.track.clear();

            pubControl.publish(controlMsg);
            rate.sleep();
            showMessagePublishInfo();
        
            for(g_nBagIndex = tmp.nBagIndex ; g_nBagIndex < bagList.size();g_nBagIndex++)       //publish message
            {
                char bagName[IMSDLEN];
                sprintf(bagName,"%s",bagList[g_nBagIndex].c_str());
                readBag.open(bagName,rosbag::bagmode::Read);           //open bag
                tempTopics.push_back("velodyne_points");
                rosbag::View view(readBag,rosbag::TopicQuery(tempTopics));

                g_nMsgIndex = 0;          // message index
                //publish message
                foreach(rosbag::MessageInstance const m,view)
                {
                    if(timetodie == 1)
                    {
                        readBag.close();
                        //cout << "got ctrl-c signal." << endl;
                        return 1;
                    }
                    g_nMsgIndex++;
                    pointcloud2 = m.instantiate<sensor_msgs::PointCloud2>();

                    if(tmp.nMsgIndex < g_nMsgIndex || tmp.nBagIndex < g_nBagIndex)    // find location to publish messge
                    {
                        pubLaserCloud.publish(pointcloud2);    // publish message
                        g_lPublishedMsgNum++;
                        nMsgSendNum++;
                        if(g_lPublishedMsgNum % 50 == 0)
                        {
                            showMessagePublishInfo();
                        }
                        rate.sleep();
                        ros::spinOnce();
                    }
                }
                readBag.close();
            }
        }
        showMessageHandleResult();
        rate.sleep();
     
        if(!slamTrack.track.empty())
        {
            slamTrack.track_flag = times;
            slamTrackVector.push(slamTrack);
            slamTrack.track.clear();
        }
        
        while(!slamTrackVector.empty())
        {
            slamTrackPub.publish(slamTrackVector.front());
            rate.sleep();
            slamTrackVector.pop();
         }
         slamTrackPub.publish(slamTrack);
         rate.sleep();
         allLocation.clear();
    }
   
    cout << "SLAM Track Calculation Over" << endl;
    return 0;
}
