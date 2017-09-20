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
#define IMRATE 3.0

using namespace std;

static int times = 0;
int firstMessage = 1;     //handle the first message odometry,first odometry has no distance
int rowNum = 0;         // message index
static int bagIndex = 0;     // bag index
static int fileNum = 0;    // track index
static double totalDistance[2];
static double overlapDistance[2];
static double totalDis = 0;
vector<string> bagList;

string commandLoam, commandKill;     // run loam and kill loam
vector<std::string> tempTopics;  //message topic
vector<sensor_msgs::PointCloud2> cloudTopics;
nav_msgs::Odometry preOdometry;              //subscribe message
rosbag::Bag readBag;       //read bag handler
sensor_msgs::PointCloud2::ConstPtr pointcloud2;          //publish message
gpsCalibration::IMTrack slamTrack;
queue<gpsCalibration::IMTrack> slamTrackVector;

struct DISTANCE
{
    int bagNum;
    int row;
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

    printf("Got ctrl-c \n");
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

        DISTANCE tmpdis;
        if(firstMessage == 1)    //first odomtry
        {
            preOdometry = (*subOdometry);
            tmpdis.bagNum = bagIndex;
            tmpdis.row = rowNum;
            tmpdis.distance = 0;
            tmpdis.timestamp = (double)subOdometry->header.stamp.toSec();
            firstMessage = 0;
        }
        else                         // the rest odometry
        {
            tmpdis.bagNum = bagIndex;
            tmpdis.row = rowNum;
            tmpdis.distance = sqrt(pow(subOdometry->pose.pose.position.x - preOdometry.pose.pose.position.x,2)      // calculate distance between two odometry points
                            + pow(subOdometry->pose.pose.position.y - preOdometry.pose.pose.position.y,2))
                            + totalDis;
            tmpdis.timestamp = (double)subOdometry->header.stamp.toSec();

            preOdometry = (*subOdometry);
        }
    
        if(tmpdis.distance <= totalDistance[times] - overlapDistance[times])    //next publish message location
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
        totalDis = tmpdis.distance;
        //printf("totaldis = %lf   pubLocation.distance = %lf\n",totaldis,pubLocation.distance);
    }
    else
    {
        cout<<"publish message to loam,but not receive odometry"<<endl;
    }
}

//read baglist.txt to get bag location
int readBagList(char *fileName,vector<string> &bagList)
{
    //open baglist.txt
    ifstream ifile;
    ifile.open(fileName);
    if(NULL==ifile)
    {
        printf("open %s error\n",fileName);
        return 1;
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
    return 0;
}


long totalMessageNumber(vector<string> bagList,vector<long> &messageNumber)
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
        cout<<"total message is "<<totalNum<<endl;
    }
    catch(rosbag::BagIOException)
    {
        cout<<"cannot find bag path,please check bag_list"<<endl;
        timetodie = 1;
    }
    return totalNum;
}


int main(int argc, char **argv)  
{
    /*
        argv[1]: baglist.txt path
        argv[2]: slam odometry path
        argv[3]: slam distance
        argv[4]: overlap slam distance
    */
    
    if(6 > argc)
    {
        printf("parameter error\n");
        return 1;
    }
    
    char *bagListPath = argv[1];
    totalDistance[0] = atof(argv[2]);
    overlapDistance[0] = 0;
    totalDistance[1] = atof(argv[3]);
    overlapDistance[1] = atof(argv[4]);

    char buf[IMSDLEN];
    
    /*
    int fd[2];     // pipe,father process send fileNum to child process
    if(pipe(fd)<0)
    {
        printf("create pipe error\n");
    }
    */

    //bool iskill = false;
    ros::init(argc, argv, "input_data_node");

    if(1 == readBagList(bagListPath,bagList))
    {
        printf("bag location not found\n");
        return 1;
    }

    if(0 == bagList.size())
    {
        cout<<"bag path file is null."<<endl;
        return 0;
    }

    vector<long> messageNumber;
    long totalMessage = totalMessageNumber(bagList,messageNumber);

	commandKill= "killall transformMaintenance; killall laserMapping; killall laserOdometry; killall scanRegistration;";     // kill loam
    
    ros::NodeHandle nh;
    ros::Publisher pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points",1024);      // publish pointcloud message
	ros::Publisher slamTrackPub = nh.advertise<gpsCalibration::IMTrack>("/slam_track",2);
    ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry>("/true_odometry_to_init",1024,subOdometryHandler);      //receive slam odometry
    ros::Publisher pubControl = nh.advertise<gpsCalibration::IMControl>("/control_command",2);
    /*
    pid_t child_pid = fork();    // fork child process,running loam
    if(child_pid == -1) {
        perror("fork error");
        exit(EXIT_FAILURE);
    }
    */

    signal(SIGINT,sigproc);       // handle ctrl-c signal

    //if(child_pid != 0)         // falther process: publish message and receive track
    //{
        for(times = 0; times < 2; times ++)
        {
            // init publocation
            pubLocation.bagNum = 0;
            pubLocation.row = 0;
            pubLocation.distance = 0;
            pubLocation.timestamp = 0;
            allLocation.push_back(pubLocation);
        
            long messageIndex = 0;
            bagIndex = 0;
            totalDis = 0;

            gpsCalibration::IMControl controlMsg;
            controlMsg.systemInited = false;
            pubControl.publish(controlMsg);
            sleep(1);

	        while(!timetodie && bagIndex < bagList.size())
	        {
                int end = 0;   // flag for end publish 
                char bagName[IMSDLEN];     // bag path

                firstMessage = 1;  // init firstMessage
                bagIndex = allLocation[allLocation.size()-1].bagNum;   // init bagIndex

                //send filenum to child process
                /*
                fileNum++;
                sprintf(buf,"%d",fileNum);
                write(fd[1],buf,sizeof(buf));
                sleep(4);              // wait child process reply

                bzero(buf,sizeof(buf));    // clear buf
                read(fd[0],buf,IMSDLEN);   // get child reply
                sleep(1);      //sleep 1 second then publish message
                system("clear");
                */

                messageIndex = 0;
                for(int i = 0;i< bagIndex;i++)
                {
                    messageIndex += messageNumber[i];
                }
                messageIndex += pubLocation.row;
                system("clear");            
                cout<<"***********************publish message***********************"<<endl;
                printf("published %ld message,%ld message left,maybe %.1lf minites to publish\n",messageIndex,totalMessage-messageIndex,(totalMessage-messageIndex)/60.0);
                while(bagIndex < bagList.size() && timetodie != 1)
                {
                    sprintf(bagName,"%s",bagList[bagIndex].c_str());
                    readBag.open(bagName,rosbag::bagmode::Read);           //open bag
                    tempTopics.push_back("velodyne_points");
                    rosbag::View view(readBag,rosbag::TopicQuery(tempTopics));
 
                    rowNum = 0;          // init messag index
                    //publish message
                    foreach(rosbag::MessageInstance const m,view)
                    {
                        rowNum++;
                        pointcloud2 = m.instantiate<sensor_msgs::PointCloud2>();
                    
                        if(timetodie == 1) { break; }      // get ctrl-c signal,return;
                 
                        if(pubLocation.row < rowNum || pubLocation.bagNum < bagIndex)    // find location to publish messge
                        {
                            pubLaserCloud.publish(pointcloud2);        //publish message
                            messageIndex ++;
                            if(messageIndex % 50 == 0)
                            {
                                system("clear");
                                cout<<"***********************publish message***********************"<<endl;
                                printf("published %ld message,%ld message left,maybe %.1lf minites to publish\n",messageIndex,totalMessage-messageIndex,(totalMessage-messageIndex)/60.0);
                            }
                            sleep(1);                // publish rate
                            ros::spinOnce();       //calculate track distance
                            if(totalDis > totalDistance[times])    // stop publish message
                            {
                                totalDis = 0;
                                end = 1;
                                break;
                            }
                        }
                    }
                    readBag.close();
                    if(end == 1)
                    {
                        pubControl.publish(controlMsg);
                        sleep(1);
                        break;
                    }
                    ++bagIndex;
                }
                slamTrack.track_flag = times;
                slamTrackVector.push(slamTrack);
                slamTrack.track.clear();
                if(3 == slamTrackVector.size())
                {
                    slamTrackPub.publish(slamTrackVector.front());
                    slamTrackVector.pop();
                }
                sleep(1);
                //system(commandKill.c_str());    // close loam

            }       

            if(!timetodie)
            {
                //if rest distance is too short,then add to previous file
                if(allLocation.size() > 1 && totalDis < totalDistance[times] / IMRATE)
                {

                    //send filenum to child process
                    /*
                    fileNum --;
                    sprintf(buf,"%d",fileNum);
                    write(fd[1],buf,sizeof(buf));
                    sleep(4);

                    read(fd[0],buf,IMSDLEN);  // get child process reply
                    sleep(1);
                    system("clear");
                    */

                    system("clear");
                    DISTANCE tmp = allLocation[allLocation.size()-2];      //publish location

                    messageIndex = 0;
                    for(int i = 0;i < tmp.bagNum;i++)
                    {
                        messageIndex += messageNumber[i];
                    }
                    messageIndex += tmp.row;

                    while(!slamTrackVector.empty())
                    {
                        slamTrackVector.pop();
                    }
                    slamTrack.track.clear();
                    pubControl.publish(controlMsg);
                    sleep(1);
                    cout<<"***********************publish message***********************"<<endl;
                    printf("published %ld message,%ld message left,maybe %.1lf minites to publish\n",messageIndex,totalMessage-messageIndex,(totalMessage-messageIndex)/60.0);
        
                    for(bagIndex = tmp.bagNum ; bagIndex < bagList.size() && timetodie != 1;bagIndex++)       //publish message
                    {
                        char bagName[IMSDLEN];
                        sprintf(bagName,"%s",bagList[bagIndex].c_str());
                        readBag.open(bagName,rosbag::bagmode::Read);           //open bag
                        tempTopics.push_back("velodyne_points");
                        rosbag::View view(readBag,rosbag::TopicQuery(tempTopics));

                        rowNum = 0;          // message index
                        //publish message
                        foreach(rosbag::MessageInstance const m,view)
                        {
                            if(timetodie == 1)
                            {
                                break;
                            }
                            rowNum++;
                            pointcloud2 = m.instantiate<sensor_msgs::PointCloud2>();

                            if(tmp.row < rowNum || tmp.bagNum < bagIndex)    // find location to publish messge
                            {
                                pubLaserCloud.publish(pointcloud2);    // publish message
                                messageIndex++;
                                if(messageIndex % 50 == 0)
                                {
                                    system("clear");
                                    cout<<"***********************publish message***********************"<<endl;
                                    printf("published %ld message,%ld message left,maybe %.1lf minites to publish\n",messageIndex,totalMessage-messageIndex,(totalMessage-messageIndex)/60.0);
                                }
                                sleep(1);
                                ros::spinOnce();
                            }
                        }
                        readBag.close();
                    }
                }
                sleep(1);
                slamTrack.track_flag = times;
                slamTrackVector.push(slamTrack);
                slamTrack.track.clear();
                //system(commandKill.c_str());     // kill loam

            }

            //system(commandKill.c_str());

            while(!slamTrackVector.empty())
            {
                slamTrackPub.publish(slamTrackVector.front());
                sleep(1);
                slamTrackVector.pop();
            }
            slamTrackPub.publish(slamTrack);
            sleep(2);
            allLocation.clear();
        }
    //}
    /*
    else       // child process: run loam
    {
        while(1)
        {
          //open loam-2 steps
          //1. track name 
          //receive track filenum
          sleep(2);            // wait father process fileNum
          read(fd[0],buf,IMSDLEN);    // get fileNum
          fileNum = atoi(buf);
          
          //running loam
          //2. loam command executed
          commandLoam= "roslaunch loam loam.launch loam_track_file:="+ loamTrackPath+ std::to_string(fileNum);
          cout << "The Command Loam Is: " << commandLoam << endl;
          write(fd[1],"ok",3);    // reply to father process
          system(commandLoam.c_str());     // run loam
        }
    }
    kill(child_pid,SIGKILL);
    cout<<"kill child pid"<<endl;
    */
    cout << "finish" << endl;
    return 0;
}
