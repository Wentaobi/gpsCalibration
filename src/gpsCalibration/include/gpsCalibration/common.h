#ifndef __COMMON_H__
#define __COMMON_H__
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cstdio>
#include <math.h>
#include <vector>
#include <string.h>
#include <stdlib.h>
#include "gpsCalibration/IMTrack.h"
#include "gpsCalibration/IMLocalXYZT.h"
#include <pcl/point_types.h>
 
#define POINTSNUM 60000
#define HEIGHT 10
#define PI 3.141592653589
#define IMLDLEN 512
#define IMSDLEN 512
#define IMDP 15
#define IMTHREEBANDS 3
#define IMSIXBANDS 6

using namespace std;

typedef struct
{
    double x;
    double y;
    double z;
    double t;
}COORDXYZT;

typedef struct
{
    double x;
    double y;
    double z;
    double t;
    double w;
}COORDXYZTW;
typedef pcl::PointXYZI PointType;
//if you want to use these follow function please define __message_transform__ to 1
vector<COORDXYZTW> fromIMTracktoCOORDXYZTW(const gpsCalibration::IMTrack::ConstPtr& msg);

gpsCalibration::IMTrack fromCOORDXYZTWtoIMTrack(vector<COORDXYZTW> & arraytrack); 
 
 
 inline double rad2deg(double radians)
 {
   return radians * 180.0 / M_PI;
 }
 
 inline double deg2rad(double degrees)
 {
   return degrees * M_PI / 180.0;
 }
#endif
