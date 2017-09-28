#ifndef _GPS_PROCESS_H_
#define _GPS_PROCESS_H_

#include <libxml/tree.h>
#include <libxml/xpath.h>
#include <libxml/parser.h>
#include <libxml/xmlmemory.h>
#include <libxml/xmlstring.h>
#include "common.h"


class WGSParameter
{
    public:
        double longAxle;
        double shortAxle;
        double E1;    // E1 = sqrt(pow(longAxle,2)-pow(shortAxle,2))/longAxle;
        double E2;    // E2 = sqrt(pow(longAxle,2)-pow(shortAxle,2))/shortAxle;
        double C;    // C = pow(longAxle,2)/shortAxle
        WGSParameter();
        ~WGSParameter();
};

class GPSPro
{
    public:
        GPSPro();
        GPSPro(string &originalGPSPath,string &method,int &type);
        int getType();    // get type value
        void setType(int type);  // set type value
        string getMethod();    // get transform method
        void setMethod(string method);    // set transform method
        string getGPSPath();      // get original gps file path
        void setGPSPath(string originalGPSPath);    // set original gps file path
        int ENUToGPS(vector<COORDXYZTW> enuCoor,vector<pair<double,double> > &WGSBL,vector<double> &altitude,vector<pair<int,string> > &segmentColor);   // ENU coordinate transform to GPS coordinate
        vector<COORDXYZT> GPSToENU(vector<COORDXYZT> slamTrack);   // GPS coordinate transform to ENU coordinate
        int createKML(string KMLFileName,vector<pair<double,double> > WGSBL,vector<double> altitude,int flag,vector<pair<int,string> > segmentColor);   // write KML file
        ~GPSPro();
    private:
        int type;              // type 3 or 6
        string method;         // method gaussion or UTM
        string originalGPSPath;    // original gps file
        WGSParameter parameter;    // WGS84 parameter

        int gpsProcess(vector<pair<double,double> > &WGSBL,vector<double> GPSTime);  // gps process
        int interPolate(vector<pair<double,double> > WGSBL,vector<double> GPSTime,vector<double> slamTrackTime,vector<pair<double,double> > &iterWGSBL);    // linear interpolation
        double arcLength(double latitude);
        int getGPS(string originalGPSPath,vector<double> slamTrackTime,vector<pair<double,double> > &WGSBL,vector<double> &GPSTime);  // get GPS from gps driver log
        int UTMTransform(vector<pair<double,double> > WGSBL,vector<pair<double,double> > &LocalXY);     // transform GPS coordinate into ENU coordinate through UTM method
        int UTMReverseTransform(vector<COORDXYZTW> localCoor,vector<pair<double,double> > &WGSBL,vector<double> &altitude);  // transform ENU coordinate into GPS coordinate through UTM method
        int GaussionTransform(vector<pair<double,double> > WGSBL,vector<pair<double,double> > &LocalXY);     // transform GPS coordinate into ENU coordinate through Gaussion method
        int GaussionReverseTransform(vector<COORDXYZTW> localCoor,vector<pair<double,double> > &WGSBL,vector<double> &altitude);   // transform ENU coordinate into GPS coordinate through Gaussion method
        vector<string> readKMLParameter();   // read KML config parameter
        vector<pair<int,string> > segment(vector<COORDXYZTW> enuCoor);   // GPS segment
        string rgbColor(double w,double distance);   // calculate RGB color
};
#endif
