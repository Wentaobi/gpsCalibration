#include "gps_process.h"
#define SEGMENTLEN 50

using namespace std;

GPSPro::GPSPro()
{
    method = "UTM";
    type = 3;
}

GPSPro::GPSPro(string &originalGPSPath,string &method,int &type)
{
    originalGPSPath = originalGPSPath;

    if(method != "UTM" && method != "Gaussion")
    {
        printf("method value is \"UTM\" or \"Gaussion\",the default is \"UTM\"\n");
        this->method = "UTM";
    }
    else
    {
        method = method;
    }

    if(type != 3 && type != 6)
    {
        printf("type value is 3 or 6,the default value is 3\n");
        type = 3;
    }
    else
    {
        type = type;
    }
}

/*calcute the Arc Length of central meridian*/
double GPSPro::arcLength(double latitude)
{
    double m0 = parameter.longAxle * (1 - pow(parameter.E1,2));
    double m2 = 3.0 / 2.0 * pow(parameter.E1,2) * m0;
    double m4 = 5.0 / 4.0 * pow(parameter.E1,2) * m2;
    double m6 = 7.0 / 6.0 * pow(parameter.E1,2) * m4;
    double m8 = 9.0 / 8.0 * pow(parameter.E1,2) * m6;

    double a0 = m0 + 1.0 / 2.0 * m2 + 3.0 / 8.0 * m4 + 5.0 / 16.0 * m6 + 35.0 / 128.0 * m8;
    double a2 = 1.0 / 2.0 * m2 + 1.0 / 2.0 * m4 + 15.0 / 32.0 * m6 + 7.0 / 16.0 * m8;
    double a4 = 1.0 / 8.0 * m4 + 3.0 / 16.0 * m6 + 7.0 / 32.0 * m8;
    double a6 = 1.0 / 32.0 * m6 + 1.0 / 16.0 * m8;
    double a8 = 1.0 / 128.0 * m8;

    double rB = latitude * PI / 180.0;                  //Angle is converted to radian

    double meridianLength = a0 * rB - a2 / 2.0 * sin(2 * rB) + a4 / 4.0 * sin(4 * rB) - a6 / 6.0 *sin(6 * rB) + a8 / 8.0 * sin(8 * rB);
    return meridianLength;
}

/*linear interpolate*/
int GPSPro::interPolate(vector<pair<double,double> > LocalXY,vector<double> GPSTime,vector<double> slamTrackTime,vector<pair<double,double> > &interLocalXY)
{
    vector<double> dataX,dataY;

    int nSize = LocalXY.size();
    for(int index = 0; index < nSize; index ++)
    {
        dataX.push_back(LocalXY[index].first);
        dataY.push_back(LocalXY[index].second);
    }

    int totalNumSource = GPSTime.size();

    int totalNumRefer = slamTrackTime.size();

    //Input GPS track coordinates
    /* Do the interpolation
        1. interpolate gps track at each slam track time
        2. interpolation method- linear interpolation
    */

    double s1,s2,s3;
    double x1,x2,y1,y2,coe1,coe2;
    int iCount = 0;

    int iSource,iRefer;
    for(iSource = 0; iSource < totalNumSource - 1; iSource ++)
    {
        s1 = GPSTime[iSource];
        s2 = GPSTime[iSource + 1];
        s3 = s2 - s1;

        x1 = dataX[iSource];
        x2 = dataX[iSource + 1];
        y1 = dataY[iSource];
        y2 = dataY[iSource + 1];

        //interpolation
        for(iRefer = iCount; iRefer < totalNumRefer; iRefer ++)
        {
            if(slamTrackTime[iRefer] > s2)
                break;
            coe1 = (slamTrackTime[iRefer] - s1) / s3;
            coe2 = 1.0 - coe1;
            
            interLocalXY.push_back(pair<double,double>(coe1 * x2 + coe2 * x1,coe1 * y2 + coe2 * y1));
            iCount ++ ;
        }
    }

    return 0;
}

/*get GPS from gps file*/
int GPSPro::getGPS(string originalGPSPath,vector<double> slamTrackTime,vector<pair<double,double> > &WGSBL,vector<double> &GPSTime)
{
    char buf[IMSDLEN];
	
    // open GPS text
    ifstream ifile;
    ifile.open(originalGPSPath.c_str());
    if(!ifile.is_open())
    {
        printf("open %s error\n",originalGPSPath.c_str());
        return 1;
    }

    //read GPS text and get GPS coordinate and GPS time

	
    ifile.getline(buf,IMSDLEN);
	char tmpBuf[IMSDLEN];
	memset (tmpBuf, 0, IMSDLEN);
	strncpy (tmpBuf, buf, sizeof (buf));
	char *p = strtok (tmpBuf, ",");
	p = strtok (NULL, ",");

	cout << "=====================================" << endl;
	cout << p << endl;
	cout << "=====================================" << endl;
	if (0 == strcmp ("$GPRMC", p))
	{
		getGPRMCFormat (buf, ifile, slamTrackTime, WGSBL, GPSTime);
	}
	else if (0 == strcmp ("$GPGGA", p))
	{
		getGPGGAFormat (buf, ifile, slamTrackTime, WGSBL, GPSTime);
	}
	else if (0 == strcmp ("$GPGLL", p))
	{
		getGPGLLFormat (buf, ifile, slamTrackTime, WGSBL, GPSTime);
	}
	else 
	{
		cout << "The current version does not support the current GPS format" << endl;
	}



    ifile.close();
    return 0;
}

void GPSPro::getGPRMCFormat (char* buf, ifstream &ifile, vector<double> slamTrackTime, vector<pair<double, double> >&WGSBL, vector<double> &GPSTime)
{
    double timestamp = 0;
    double startTime = slamTrackTime[0]; 
    double endTime = slamTrackTime[slamTrackTime.size()-1];

    do {
        int column = 0;
        double latitude = 90,longitude = 180;
        timestamp = 0;
        char *splite = strtok(buf,",");
        while(splite)
        {
            column ++ ;
            
			if(4 == column && !strcmp("V",splite))      // the forth is 'A' or 'V'; if 'V', no GPS signal
            {
                break;
            }


            switch(column)
            {
                case 1:                              // the first column is timestamp
                {
                    timestamp = atof(splite);   
                    break;
                }
                case 5:                         // the fifth column is latitude(ddmm.mmmm)
                {
                    int tmp = atof(splite) / 100;
                    latitude = tmp + (atof(splite) - tmp * 100) / 60.0;
                    break;
                }
                case 6:                         // the sixth column is 'N' or 'S'; if 'S', latitude is negative
                {
                    if(!strcmp("S",splite))
                    {
                        latitude = 0 - latitude;
                    }
                    break;
                }
                case 7:                         // the seventh column is longitude(dddmm.mmmm)
                {
                    int tmp = atof(splite) / 100;
                    longitude = tmp + (atof(splite) - tmp * 100) / 60.0;
                    break;
                }
                case 8:                         // the eighth column is 'W' or 'E'; if 'W', longitude is negative
                {
                    if(!strcmp("W",splite))
                    {
                        longitude = 0 - longitude;
                    }
                    break;
                }
            }
            splite = strtok(NULL,",");
        }
        splite = NULL;

        if((long)timestamp >= (long)(startTime - 1) && (long)timestamp <= (long)(endTime + 1))
        {
            WGSBL.push_back(pair<double,double>(latitude,longitude));
            GPSTime.push_back(timestamp);
        }
    }while(ifile.getline(buf,IMSDLEN) && timestamp < endTime + 1);
	
}

void GPSPro::getGPGGAFormat (char* buf, ifstream &ifile, vector<double> slamTrackTime, vector<pair<double, double> >&WGSBL, vector<double> &GPSTime)
{
    double timestamp = 0;
    double startTime = slamTrackTime[0]; 
    double endTime = slamTrackTime[slamTrackTime.size()-1];


    do {
        int column = 0;
        double latitude = 90,longitude = 180;
        timestamp = 0;
        char *splite = strtok(buf,",");
        while(splite)
        {
            column ++ ;

			if (5 == column && 0 != strcmp ("N", splite) && 0 != strcmp ("S", splite))
			{
				break;
			}
			if (7 == column && 0 != strcmp ("W", splite) && 0 != strcmp("E", splite))
			{
				break;
			}

            switch(column)
            {
                case 1:                              // the first column is timestamp
                {
                    timestamp = atof(splite);   
                    break;
                }
                case 4:                         // the fifth column is latitude(ddmm.mmmm)
                {
                    int tmp = atof(splite) / 100;
                    latitude = tmp + (atof(splite) - tmp * 100) / 60.0;
                    break;
                }
                case 5:                         // the sixth column is 'N' or 'S'; if 'S', latitude is negative
                {
                    if(!strcmp("S",splite))
                    {
                        latitude = 0 - latitude;
                    }
                    break;
                }
                case 6:                         // the seventh column is longitude(dddmm.mmmm)
                {
                    int tmp = atof(splite) / 100;
                    longitude = tmp + (atof(splite) - tmp * 100) / 60.0;
                    break;
                }
                case 7:                         // the eighth column is 'W' or 'E'; if 'W', longitude is negative
                {
                    if(!strcmp("W",splite))
                    {
                        longitude = 0 - longitude;
                    }
                    break;
                }
            }
            splite = strtok(NULL,",");
        }
        splite = NULL;

        if((long)timestamp >= (long)(startTime - 1) && (long)timestamp <= (long)(endTime + 1) && 90 != latitude && 180 != longitude)
        {
            WGSBL.push_back(pair<double,double>(latitude,longitude));
            GPSTime.push_back(timestamp);
        }
    }while(ifile.getline(buf,IMSDLEN) && timestamp < endTime + 1);
	
}
void GPSPro::getGPGLLFormat (char* buf, ifstream &ifile, vector<double> slamTrackTime, vector<pair<double, double> >&WGSBL, vector<double> &GPSTime)
{
    double timestamp = 0;
    double startTime = slamTrackTime[0]; 
    double endTime = slamTrackTime[slamTrackTime.size()-1];

    do {
        int column = 0;
        double latitude = 90,longitude = 180;
        timestamp = 0;
        char *splite = strtok(buf,",");
        while(splite)
        {
            column ++ ;
            
			if(8 == column && !strcmp("V",splite))      // the forth is 'A' or 'V'; if 'V', no GPS signal
            {
                break;
            }


            switch(column)
            {
                case 1:                              // the first column is timestamp
                {
                    timestamp = atof(splite);   
                    break;
                }
                case 3:                         // the fifth column is latitude(ddmm.mmmm)
                {
                    int tmp = atof(splite) / 100;
                    latitude = tmp + (atof(splite) - tmp * 100) / 60.0;
                    break;
                }
                case 4:                         // the sixth column is 'N' or 'S'; if 'S', latitude is negative
                {
                    if(!strcmp("S",splite))
                    {
                        latitude = 0 - latitude;
                    }
                    break;
                }
                case 5:                         // the seventh column is longitude(dddmm.mmmm)
                {
                    int tmp = atof(splite) / 100;
                    longitude = tmp + (atof(splite) - tmp * 100) / 60.0;
                    break;
                }
                case 6:                         // the eighth column is 'W' or 'E'; if 'W', longitude is negative
                {
                    if(!strcmp("W",splite))
                    {
                        longitude = 0 - longitude;
                    }
                    break;
                }
            }
            splite = strtok(NULL,",");
        }
        splite = NULL;

        if((long)timestamp >= (long)(startTime - 1) && (long)timestamp <= (long)(endTime + 1))
        {
            WGSBL.push_back(pair<double,double>(latitude,longitude));
            GPSTime.push_back(timestamp);
        }
    }while(ifile.getline(buf,IMSDLEN) && timestamp < endTime + 1);
	
}
/* ENU coordinate transform to GPS coordinate*/
int GPSPro::ENUToGPS(vector<COORDXYZTW> enuCoor,vector<pair<double,double> > &WGSBL,vector<double> &altitude,vector<pair<int,string> > &segmentColor)
{
    segmentColor = segment(enuCoor);

    if(!strcmp("UTM",method.c_str()))
    {
        UTMReverseTransform(enuCoor,WGSBL,altitude);
    }
    else
    {
        GaussionReverseTransform(enuCoor,WGSBL,altitude);
    }
}

/* GPS process */
int GPSPro::gpsProcess(vector<pair<double,double> > &WGSBL,vector<double> GPSTime)
{
    int index = 0;
    while(index < WGSBL.size())
    {
        int beginIndex = -2,endIndex = -2;
        int flag = 0;

        // find gps point exception location
        for(;index < WGSBL.size(); index ++)
        {
            if(90 == WGSBL[index].first && 180 == WGSBL[index].second)  // gps point exception
            {
                if(0 == flag)
                {
                    beginIndex = index - 1;
                    flag = 1;
                }
            }
            else if(1 == flag)
            {
                endIndex = index;
                break;
            }
        }

        if(-2 == beginIndex && -2 == endIndex)     // no gps exception
        {
            return 0;
        }

        // linear interpolation
        if(-1 == beginIndex)
        {
            if(-2 == endIndex || WGSBL.size() - 1 == endIndex)  // only have one end gps point,cannot do linear interpolation
            {
                return 1;
            }
            else                                              // linear front interpolation
            {
                double deltaT = GPSTime[endIndex + 1] - GPSTime[endIndex];

                double deltaB = (WGSBL[endIndex + 1].first - WGSBL[endIndex].first) / deltaT;

                double deltaL = (WGSBL[endIndex + 1].second - WGSBL[endIndex].second) / deltaT;

                for(int index = endIndex - 1; index > beginIndex; index --)
                {
                    WGSBL[index].first = WGSBL[index + 1].first - deltaB * (GPSTime[index + 1] - GPSTime[index]);
                    WGSBL[index].second = WGSBL[index + 1].second - deltaL * (GPSTime[index + 1] - GPSTime[index]);
                }
            }
        }
        else
        {
            if(0 == beginIndex && -2 == endIndex)     // only have one begin gps point,cannot do linear interpolation
            {
                return 1;
            }
            if(0 < beginIndex && -2 == endIndex)     // linear after interpolation
            {
                double deltaT = GPSTime[beginIndex] - GPSTime[beginIndex - 1];
                double deltaB = (WGSBL[beginIndex].first - WGSBL[beginIndex - 1].first) / deltaT;
                double deltaL = (WGSBL[beginIndex].second - WGSBL[beginIndex - 1].second) / deltaT;
                for(int index = beginIndex + 1;index < WGSBL.size(); index ++)
                {
                    WGSBL[index].first = WGSBL[index - 1].first + deltaB * (GPSTime[index] - GPSTime[index - 1]);
                    WGSBL[index].second = WGSBL[index - 1].second + deltaL * (GPSTime[index] - GPSTime[index - 1]);
                }
            }
            else                    // linear middle interpolation
            {
                double deltaT = GPSTime[endIndex] - GPSTime[beginIndex];
                double deltaB = (WGSBL[endIndex].first - WGSBL[beginIndex].first) / deltaT;
                double deltaL = (WGSBL[endIndex].second - WGSBL[beginIndex].second) / deltaT;
                for(int index = beginIndex + 1; index < endIndex; index ++)
                {
                    WGSBL[index].first = WGSBL[index - 1].first + deltaB * (GPSTime[index] - GPSTime[index - 1]);
                    WGSBL[index].second = WGSBL[index - 1].second + deltaL * (GPSTime[index] - GPSTime[index - 1]);
                }
            }
        }
    }
    return 0;
}

/* GPS coordinate transform to ENU coordinate */
vector<COORDXYZT> GPSPro::GPSToENU(vector<COORDXYZT> slamTrack)
{
    vector<pair<double,double> > WGSBL,LocalXY,interLocalXY;
    vector<double> GPSTime,slamTrackTime;

    int nSlamTrackSize = slamTrack.size();
    for(int index = 0; index < nSlamTrackSize; index ++)
    {
        slamTrackTime.push_back(slamTrack[index].t);
    }

    if(1 == getGPS(originalGPSPath,slamTrackTime,WGSBL,GPSTime))
    {
        exit(0);
    }
    if(GPSTime.empty() && WGSBL.empty())
    {
        cout << "WARN: cannot find GPS information corresponding to slam track time,please check GPS original file." << endl;
        exit(0);
    }
    gpsProcess(WGSBL,GPSTime);

    if(!strcmp("UTM",method.c_str()))
    {
        UTMTransform(WGSBL,LocalXY);
    }
    else
    {
        GaussionTransform(WGSBL,LocalXY);
    }
    interPolate(LocalXY,GPSTime,slamTrackTime,interLocalXY);
   
    vector<COORDXYZT> localCoor;
    int nInterLocalXYSize = interLocalXY.size();
    for(int index = 0; index < nInterLocalXYSize; index ++)
    {
        COORDXYZT tmp;
        tmp.x = interLocalXY[index].first;
        tmp.y = interLocalXY[index].second;
        tmp.z = slamTrack[index].z;
        tmp.t = slamTrackTime[index];
        localCoor.push_back(tmp);
    }

    return localCoor;
}

/**
 *  GPS ==> GCJ02
  */
int GPSPro::GPSToGCJ (vector<pair<double, double> > vecGpsCoor, vector<pair<double, double> >& vecGCJ)
{
    if (0 >= vecGpsCoor.size ())
    {
        cout << "GPS data NULL" << endl;
        return -1;
    }

    double lGCJLat = 0;
    double lGCJLon = 0;

    for (int i=0; i<vecGpsCoor.size(); ++i)
    {
        transform2Mars (vecGpsCoor[i].second, vecGpsCoor[i].first, lGCJLon, lGCJLat);
        vecGCJ.push_back(pair<double, double>(lGCJLat, lGCJLon));
    }

    return 0;
}





/**
  *   GCJ02 ==> BD09
  */
int GPSPro::GCJToBD (vector<pair<double, double> > vecGCJCoor, vector<pair<double, double> >&vecBD)
{
    if (0 >= vecGCJCoor.size ())
    {
        cout << "GCJ02 data NULL " << endl;
        return -1;
    }

    double lBDLat = 0;
    double lBDLon = 0;
    for (int i=0; i<vecGCJCoor.size (); ++i)
    {
        bd_encrypt (vecGCJCoor[i].second, vecGCJCoor[i].first, lBDLon, lBDLat);
        vecBD.push_back (pair<double, double>(lBDLat, lBDLon));
    }
    return 0;
}




/**
  *  BD09 ==> GCJ02
  */
int GPSPro::BDToGCJ (vector<pair<double, double> > vecBDCoor, vector<pair<double, double> >&vecGCJ)
{
    if (0 >= vecBDCoor.size ())
    {
        cout << "BD09 data NULL" << endl;
        return -1;
    }

    double lGCJLat = 0;
    double lGCJLon = 0;

    for (int i=0; i<vecBDCoor.size (); ++i)
    {
        bd_decrypt (vecBDCoor[i].second, vecBDCoor[i].first, lGCJLon, lGCJLat);

        vecGCJ.push_back(pair<double, double>(lGCJLat, lGCJLon));
    }
    return 0;
}



/*GPS segment*/
vector<pair<int,string> > GPSPro::segment(vector<COORDXYZTW> enuCoor)
{
    vector<pair<int,string> > segmentColor;

    double distance = 0;
    double weightSum = 0;
    if(0 == enuCoor.size())
    {
        exit(0);
    }
    weightSum = enuCoor[0].w;
    for(int index = 1;index < enuCoor.size(); index ++)
    {
        double deltaX = enuCoor[index].x - enuCoor[index - 1].x;
        double deltaY = enuCoor[index].y - enuCoor[index - 1].y;
        weightSum += enuCoor[index].w;
        distance += sqrt(deltaX * deltaX + deltaY * deltaY);
        if(distance > SEGMENTLEN || index == enuCoor.size() - 1)
        {
            string color = rgbColor(weightSum,distance);
            segmentColor.push_back(pair<int,string>(index,color));
            distance = 0;
            weightSum = 0;
        }
    }
    return segmentColor;
}

/*read parameter*/
vector<string> GPSPro::readKMLParameter()
{
    vector<string> config;
    xmlDocPtr pDoc=xmlReadFile("src/gpsCalibration/config/kml_config.xml","UTF-8",XML_PARSE_RECOVER); //get xml pointer
    if(NULL==pDoc)
    {
        printf("open config.xml error\n");
        exit(0);
    }

    xmlNodePtr pRoot=xmlDocGetRootElement(pDoc);     //get xml root
            
    if(NULL==pRoot)
    {
        printf("get config.xml root error\n");
        exit(0);
    }

    xmlNodePtr pFirst=pRoot->children;

    while(NULL!=pFirst)
    {
        xmlChar *value=NULL;
        if(!xmlStrcmp(pFirst->name,(const xmlChar *)("style")))
        {
            xmlNodePtr pStyle=pFirst->children;
            while(NULL!=pStyle)
            {
                value=xmlNodeGetContent(pStyle);
                if(xmlStrcmp(pStyle->name,(const xmlChar *)("text")))
                {
                    config.push_back((char *)value);
                }
                pStyle=pStyle->next;
            }
        }
        else if(!xmlStrcmp(pFirst->name,(const xmlChar *)("Placemark")))
        {
            xmlNodePtr pPlacemark=pFirst->children;
            while(NULL!=pPlacemark)
            {
                value=xmlNodeGetContent(pPlacemark);
                if(xmlStrcmp(pPlacemark->name,(const xmlChar *)("text")))
                {
                    config.push_back((char *)value);
                }
                pPlacemark=pPlacemark->next;
            }
        }
        else
        {
            value=xmlNodeGetContent(pFirst);
            if(xmlStrcmp(pFirst->name,(const xmlChar *)("text")))
            {
                config.push_back((char *)value);
            }
        }
        pFirst=pFirst->next;
    }
    return config;
}

/* calculate color */
string GPSPro::rgbColor(double w,double distance)
{
    float a;
    int x,y,r,g,b;
    char buf[IMSDLEN];

    w = w / distance;
    w = min(w / 0.667,1.0);
    a = (1 - w) / 0.25;
    x = floor(a);
    y = floor(255 * (a - x));

    switch(x)
    {
        case 0:
        {
            r = 255;
            g = y;
            b = 0;
            break;
        }
        case 1:
        {
            r = 255 - y;
            g = 255;
            b = 0;
            break;
        }
        case 2:
        {
            r = 0;
            g = 255;
            b = y;
            break;
        }
        case 3:
        {
            r = 0;
            g = 255 - y;
            b = 255;
            break;
        }
        case 4:
        {
            r = 0;
            g = 0;
            b = 255;
            break;
        }
    }

    sprintf(buf,"%02X",r);
    string rString(buf);
    bzero(buf,sizeof(buf));

    sprintf(buf,"%02X",g);
    string gString(buf);
    bzero(buf,sizeof(buf));

    sprintf(buf,"%02X",b);
    string bString(buf);
    bzero(buf,sizeof(buf));
    
    return rString + gString + bString;
}

/*write KML file*/
int GPSPro::createKML(string KMLFileName,vector<pair<double,double> > WGSBL,vector<double> altitude,int flag,vector<pair<int,string> > segmentColor)
{
    vector<string> configParameter = readKMLParameter();
    ofstream ofile;
    ofile.open(KMLFileName.c_str());
    if(!ofile.is_open())
    {
        printf("open %s error\n",KMLFileName.c_str());
        return 1;
    }
    ofile.precision(IMDP);

    int index = 0;
    if(0 == flag)
    {
        ofile<<"<?xml version=\"1.0\" encoding=\"UTF-8\"?>"<<endl;
        ofile<<"<kml xmlns=\"http://www.opengis.net/kml/2.2\">"<<endl;
        ofile<<"<Document>"<<endl;
        ofile<<"<name>"<<"original GPS"<<"</name>"<<endl;
        ofile<<"<description>"<<"original GPS"<<"</description>"<<endl;
        ofile<<"<Style id=\""<<configParameter[index ++]<<"\">"<<endl;
        ofile<<"<LineStyle>"<<endl;
        ofile<<"<color>"<<"7fFF00FF"<<"</color>"<<endl;
        ofile<<"<width>"<<configParameter[index++]<<"</width>"<<endl;
        ofile<<"</LineStyle>"<<endl;
        ofile<<"<PolyStyle>"<<endl;
        ofile<<"<color>"<<"7fFF00FF"<<"</color>"<<endl;
        ofile<<"</PolyStyle>"<<endl;
        ofile<<"</Style>"<<endl;
        ofile<<"<Placemark>"<<endl;
        ofile<<"<styleUrl>"<<configParameter[index++]<<"</styleUrl>"<<endl;
        ofile<<"<LineString>"<<endl;
        ofile<<"<extrude>"<<configParameter[index++]<<"</extrude>"<<endl;
        ofile<<"<tessellate>"<<configParameter[index++]<<"</tessellate>"<<endl;
        ofile<<"<altitudeMode>"<<configParameter[index++]<<"</altitudeMode>"<<endl;
        ofile<<"<coordinates>"<<endl;
        for(int i = 0; i < WGSBL.size() && i < altitude.size(); i++)
        {
            ofile<<WGSBL[i].first<<','<<WGSBL[i].second<<','<<altitude[i]<<endl;
        }
        ofile<<"</coordinates>"<<endl;
        ofile<<"</LineString></Placemark>"<<endl;
        ofile<<"</Document></kml>"<<endl;
    }
    else
    {
        ofile<<"<?xml version=\"1.0\" encoding=\"UTF-8\"?>"<<endl;
        ofile<<"<kml xmlns=\"http://www.opengis.net/kml/2.2\">"<<endl;
        ofile<<"<Document>"<<endl;
        ofile<<"<name>"<<"calibrated GPS"<<"</name>"<<endl;
        ofile<<"<description>"<<"calibrated GPS"<<"</description>"<<endl;

        int indexCoor = 0;

        for(int i = 0; i < segmentColor.size(); i ++)
        {
            index = 0;
            ofile<<"<Style id=\""<<configParameter[index++]<<"\">"<<endl;
            ofile<<"<LineStyle>"<<endl;
            ofile<<"<color>"<< ("7f" + segmentColor[i].second ) <<"</color>"<<endl;
            ofile<<"<width>"<<configParameter[index++]<<"</width>"<<endl;
            ofile<<"</LineStyle>"<<endl;
            ofile<<"<PolyStyle>"<<endl;
            ofile<<"<color>"<<segmentColor[i].second<<"</color>"<<endl;
            ofile<<"</PolyStyle>"<<endl;
            ofile<<"</Style>"<<endl;
            ofile<<"<Placemark>"<<endl;
            ofile<<"<styleUrl>"<<configParameter[index++]<<"</styleUrl>"<<endl;
            ofile<<"<LineString>"<<endl;
            ofile<<"<extrude>"<<configParameter[index++]<<"</extrude>"<<endl;
            ofile<<"<tessellate>"<<configParameter[index++]<<"</tessellate>"<<endl;
            ofile<<"<altitudeMode>"<<configParameter[index++]<<"</altitudeMode>"<<endl;
            ofile<<"<coordinates>"<<endl;
            for(;indexCoor < segmentColor[i].first && index < altitude.size(); indexCoor ++)
            {
                ofile<<WGSBL[indexCoor].first<<','<<WGSBL[indexCoor].second<<','<<altitude[indexCoor]<<endl;
            }
            ofile<<"</coordinates>"<<endl;
            ofile<<"</LineString></Placemark>"<<endl;

        }

        ofile<<"</Document></kml>"<<endl;

    }

    ofile.close();
    return 0;
}


/* UTM transform */
int GPSPro::UTMTransform(vector<pair<double,double> > WGSBL,vector<pair<double,double> > &LocalXY)
{
  
    if (WGSBL.empty())
    {
        return 1;
    }

    double localX = 0,localY = 0;
    int bandNum = 0;
    double meridian = 0;

    int nSize = WGSBL.size();
    for(int index = 0;index < nSize; index ++)
    {
        /* calculate the central meridian */
        if(type == IMTHREEBANDS)
        {
            if(0 == bandNum)
            {
                bandNum = WGSBL[index].second / IMTHREEBANDS;
                double tmp = WGSBL[index].second / IMTHREEBANDS;
                if(tmp - bandNum > 0.5)
                {
                    bandNum += 1;
                }
            }
            meridian = IMTHREEBANDS * bandNum;
        }
        else if(type == IMSIXBANDS)
        {
            if(0 == bandNum)
            {
                bandNum = (int)WGSBL[index].second / IMSIXBANDS + 1;
            }
            meridian = IMSIXBANDS * bandNum - IMSIXBANDS / 2;
        }
        
        /* calculate related parameters */
        double k0 = 0.9996;
        double rB = WGSBL[index].first * PI / 180.0;     // angle is converted to radian
        double t = tan(rB) * tan(rB);
        double c = pow(parameter.E2,2) * pow(cos(rB),2);
        double A = (WGSBL[index].second - meridian) * PI / 180.0 * cos(rB); 
        double N = parameter.longAxle / sqrt(1 - parameter.E1 * parameter.E1 * sin(rB) * sin(rB));
        double M = parameter.longAxle * ((1 - pow(parameter.E1,2) / 4.0 - 3.0 * pow(parameter.E1,4) / 64.0 - 5.0 * pow(parameter.E1,6) / 256.0) * rB - (3.0 * pow(parameter.E1,2) / 8.0 + 3.0 * pow(parameter.E1,4) / 32.0 + 45.0 * pow(parameter.E1,6) / 1024.0) * sin(2 * rB) + (15.0 * pow(parameter.E1,4) / 256.0 + 45.0 * pow(parameter.E1,6) / 1024.0) * sin(4 * rB) - 35.0 * pow(parameter.E1,6) / 3072.0 * sin(6 * rB));

        /* calculate the coordinate (x,y)*/
        localX =  k0 * (M + N * tan(rB) * (A * A / 2.0 + (5 - t + 9 * c + 4 * c * c) * pow(A,4) / 24.0) + (61 - 58 * t + t * t + 600 * c - 330 * parameter.E2 * parameter.E2) * pow(A,6) / 720.0);
        localY = k0 * N * (A + (1 - t + c) * pow(A,3) / 6.0 + (5 - 18 * t + t * t + 72 * c - 58 * parameter.E2 * parameter.E2) * pow(A,5) /120.0) + 500000;

        localY += bandNum * 10000000;

        LocalXY.push_back(pair<double,double>(localX,localY));
    }

    return 0;
}

/* transform ENU coordinate into GPS coordinate through Gaussion method */
int GPSPro::GaussionReverseTransform(vector<COORDXYZTW> localCoor,vector<pair<double,double> > &WGSBL,vector<double> &altitude)
{
    int bandNum = 0;
    int meridian = 0;
    double latitude = 0,longitude = 0;
    for(int index = 0; index < localCoor.size(); index ++)
    {
        bandNum = localCoor[index].y / 10000000;    // head of y coordinate is band number
        if(IMTHREEBANDS == type)
        {
            meridian = IMTHREEBANDS * bandNum;
        }
        else if(IMSIXBANDS == type)
        {
            meridian = IMSIXBANDS * bandNum - IMSIXBANDS / 2;
        }

        double localY = localCoor[index].y - bandNum * 10000000 - 500000;   // true y coordinate

        /* calculate parameters */
        double X = localCoor[index].x;
        double fi = X / (parameter.longAxle * (1 - pow(parameter.E1,2) / 4 - 3 * pow(parameter.E1,4) / 64 - 5 * pow(parameter.E1,6) / 256));
        double e = (1 - parameter.shortAxle / parameter.longAxle) / (1 + parameter.shortAxle / parameter.longAxle);
        double Bf = fi + (3 * e / 2 - 27 * pow(e,3) / 32) * sin(2 * fi) + (21 * e * e / 16 - 55 * pow(e,4) / 32) * sin(4 * fi) + 151 * pow(e,3) / 96 * sin(6 * fi);
        double Nf = parameter.longAxle / sqrt(1 - parameter.E1 * parameter.E1 * pow(sin(Bf),2));
        double Rf = parameter.longAxle * (1 - parameter.E1 * parameter.E1) / pow((1 - parameter.E1 * parameter.E1 * pow(sin(Bf),2)),1.5);
        double D = localY / (Nf);
        double Cf = parameter.E2 * parameter.E2 * cos(Bf) * cos(Bf);
        double Tf = tan(Bf) * tan(Bf);

        /* calculate GPS coordinate */
        latitude = Bf - Nf * tan(Bf) / Rf * (D * D / 2 - (5 + 3 * Tf + Cf - 9 * Tf * Cf) * pow(D,4) / 24 + (61 + 90 * Tf + 45 * Tf * Tf) * pow(D,6) / 720);
        longitude = meridian + (1.0 / cos(Bf) * (D - (1 + 2 * Tf + Cf) * pow(D,3) / 6 + (5 + 28 * Tf + 6 * Cf + 8 * Tf * Cf + 24 * Tf * Tf) * pow(D,5) / 120)) * 180 / PI;

        latitude = latitude * 180 / PI;     //The radians are converted to angles

        WGSBL.push_back(pair<double,double>(longitude,latitude));
        altitude.push_back(localCoor[index].z);
    }
}

/* transform GPS coordinate into ENU coordinate through Gaussion method */
int GPSPro::GaussionTransform(vector<pair<double,double> > WGSBL,vector<pair<double,double> > &LocalXY)
{
    if(0 == WGSBL.size())
    {
        return 1;
    }

    double localX = 0,localY = 0;
    int bandNum = 0;
    double meridian = 0;
    for(int index = 0; index < WGSBL.size(); index ++)
    {
        /* calculate the central meridian */
        if(IMTHREEBANDS == type)
        {
            if(0 == bandNum)
            {
                bandNum = WGSBL[index].second / IMTHREEBANDS;
                double tmp = WGSBL[index].second / IMTHREEBANDS;
                if(tmp - bandNum > 0.5)
                {
                    bandNum += 1;
                }
            }
            meridian = IMTHREEBANDS * bandNum;
        }
        else if(IMSIXBANDS == type)
        {
            if(0 == bandNum)
            {
                bandNum = (int)WGSBL[index].second / IMSIXBANDS + 1;
            }
            meridian = IMSIXBANDS * bandNum - IMSIXBANDS / 2;
        }

        /* calculate related parameters */
        double rB = WGSBL[index].first * PI / 180.0;      //angle is converted to radian
        double t = tan(rB);
        double ng2 = pow(parameter.E2,2) * pow(cos(rB),2);
        double N = parameter.C / sqrt(1 + ng2);
        double m = cos(rB) * PI / 180.0 * (WGSBL[index].second - meridian);

        double meridianLength = arcLength(WGSBL[index].first); // the length of central meridian

        /*calcute the coordinate (x,y)*/
        localX = meridianLength + N * t * (1.0 / 2.0 * m * m + 1.0 / 24.0 * (5 - t * t + 9 * ng2 + 4 * ng2 * ng2) * pow(m,4) + 1.0 / 720.0 * (61 - 58 * t * t + pow(t,4) + 270 * ng2 - 330 * ng2 * t * t) * pow(m,6));
        localY = N * (m + 1.0 / 6.0 * (1 - t * t + ng2) * pow(m,3) + 1.0 / 120.0 * (5 - 18 * t * t + pow(t,4) + 14 * ng2 - 58 * ng2 * t * t) * pow(m,5)) + 500000;

        localY += bandNum * 10000000;           // head of y coordinate is Band number

        LocalXY.push_back(pair<double,double>(localX,localY));

    }
    return 0;
}

/* transform ENU coordinate into GPS coordinate through UTM method*/
int GPSPro::UTMReverseTransform(vector<COORDXYZTW> localCoor,vector<pair<double,double> > &WGSBL,vector<double> &altitude)
{

    if(0 == localCoor.size())
    {
        return 1;
    }

    double latitude  = 0,longitude = 0;
    int bandNum = 0;
    double meridian = 0;
    for(int index = 0; index < localCoor.size(); index ++)
    {
        bandNum = localCoor[index].y / 10000000;      // head of y coordinate is band number
        if(IMTHREEBANDS == type)
        {
            meridian = IMTHREEBANDS * bandNum;
        }
        else if(IMSIXBANDS == type)
        {
            meridian = IMSIXBANDS * bandNum - IMSIXBANDS / 2;
        }

        localCoor[index].y = localCoor[index].y - bandNum * 10000000 - 500000;   // true y coordinate

        /*calcute parameter*/
        double k0 = 0.9996;
        double X = localCoor[index].x / k0;
        double fi = X / (parameter.longAxle * (1 - pow(parameter.E1,2) / 4 - 3 * pow(parameter.E1,4) / 64 - 5 * pow(parameter.E1,6) / 256));
        double e = (1 - parameter.shortAxle / parameter.longAxle) / (1 + parameter.shortAxle / parameter.longAxle);
        double Bf = fi + (3 * e / 2 - 27 * pow(e,3) / 32) * sin(2 * fi) + (21 * e * e / 16 - 55 * pow(e,4) / 32) * sin(4 * fi) + 151 * pow(e,3) / 96 * sin(6 * fi);
        double Nf = parameter.longAxle / sqrt(1 - parameter.E1 * parameter.E1 * pow(sin(Bf),2));
        double Rf = parameter.longAxle * (1 - parameter.E1 * parameter.E1) / pow((1 - parameter.E1 * parameter.E1 * pow(sin(Bf),2)),1.5);
        double D = localCoor[index].y / (k0 * Nf);
        double Cf = parameter.E2 * parameter.E2 *cos(Bf) * cos(Bf);
        double Tf = tan(Bf) * tan(Bf);

        /*calcute GPS coordinate*/
        latitude = Bf - Nf * tan(Bf) / Rf * (D * D / 2 - (5 + 3 * Tf + 10 * Cf - 4 * Cf * Cf - 9 * parameter.E2 * parameter.E2) * pow(D,4) / 24.0 + (61 + 90 * Tf + 298 * Cf + 45 * Tf * Tf - 252 * parameter.E2 * parameter.E2 - 3 * Cf * Cf) * pow(D,6) / 720);
        longitude = meridian + (1.0 / cos(Bf) * (D - (1 + 2 * Tf + Cf) * pow(D,3) / 6.0 + (5 - 2 * Cf + 28 * Tf - 3 * Cf * Cf + 8 * parameter.E2 * parameter.E2 + 24 * Tf * Tf) * pow(D,5) / 120.0)) * 180 / PI;

        latitude = latitude * 180 / PI;              //The radians are converted to angles

        WGSBL.push_back(pair<double,double>(longitude,latitude));
        altitude.push_back(localCoor[index].z);
    }

    return 0;
}

int GPSPro::getType()
{
    return this->type;
}

void GPSPro::setType(int type)
{
    if(type != 3 && type != 6)
    {
        printf("type value is 3 or 6,the default value is 3\n");
        this->type = 3;
    }
    else
    {
        this->type = type;
    }
}

string GPSPro::getMethod()
{
    return this->method;
}

void GPSPro::setMethod(string method)
{
    if(method != "UTM" && method != "Gaussion")
    {
        printf("method value is \"UTM\" or \"Gaussion\",the default is \"UTM\"\n");
        this->method = "UTM";
    }
    else
    {
        this->method = method;
    }
}

string GPSPro::getGPSPath()
{
    return this->originalGPSPath;
}

void GPSPro::setGPSPath(string originalGPSPath)
{
    this->originalGPSPath = originalGPSPath;
}

GPSPro::~GPSPro()
{
}


WGSParameter::WGSParameter()
{
    longAxle = 6378137;
    shortAxle = 6356752.314;
    E1 = sqrt(pow(longAxle,2)-pow(shortAxle,2))/longAxle;
    E2 = sqrt(pow(longAxle,2)-pow(shortAxle,2))/shortAxle;
    C =  pow(longAxle,2)/shortAxle;
}


WGSParameter::~WGSParameter()
{
}



bool GPSPro::outOfChina (double lat, double lon)
{
    if (lon < 72.004 || lon > 137.8347)
    {
        return true;
    }
    if (lat < 0.8293 || lat > 55.8271)
    {
        return true;
    }
        return false;
}

// transform lat
double GPSPro::transformLat (double x, double y)
{
    double ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * sqrt(abs(x));
    ret += (20.0 * sin(6.0 * x * PI) + 20.0 * sin(2.0 * x * PI)) * 2.0 / 3.0;
    ret += (20.0 * sin(y * PI) + 40.0 * sin(y / 3.0 * PI)) * 2.0 / 3.0;
    ret += (160.0 * sin(y / 12.0 * PI) + 320 * sin(y * PI / 30.0)) * 2.0 / 3.0;
    return ret;
}

// transform lon
double GPSPro::transformLon (double x, double y)
{
    double ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x* y + 0.1 * sqrt(abs(x));
    ret += (20.0 * sin(6.0 * x * PI) + 20.0 * sin(2.0 * x * PI)) * 2.0 / 3.0;
    ret += (20.0 * sin(x * PI) + 40.0 * sin(x / 3.0 * PI)) * 2.0 / 3.0;
    ret += (150.0 * sin(x / 12.0 * PI) + 300.0 * sin(x / 30.0 * PI)) * 2.0 / 3.0;
        return ret;
}

// GPS ==> GCJ02
void GPSPro::transform2Mars (double lGpsLat, double lGpsLon, double &lGcjLat, double &lGcjLon)
{
    if (outOfChina (lGpsLat, lGpsLon))
    {
        lGcjLat = lGpsLat;
        lGcjLon = lGpsLon;
        return;
    }
    double dLat = transformLat(lGpsLon - 105.0, lGpsLat - 35.0);
    double dLon = transformLon(lGpsLon - 105.0, lGpsLat - 35.0);
    double radLat = lGpsLat / 180.0 * PI;
    double magic = sin (radLat);
    magic = 1 - ee * magic * magic;
    double sqrtMagic = sqrt (magic);
    dLat = (dLat * 180.0) / ((LongAxis * (1-ee)) / (magic * sqrtMagic) * PI);
    dLon = (dLon * 180.0) / (LongAxis / sqrtMagic * cos(radLat) * PI);
    lGcjLat = lGpsLat + dLat;
    lGcjLon = lGpsLon + dLon;
}


// GCJ02 ==> BD09
void GPSPro::bd_encrypt (double lGcjLat, double lGcjLon, double& lBdLat, double& lBdLon)
{
    double x = lGcjLon;
    double y = lGcjLat;
    double z = sqrt(x * x + y * y) + 0.00002 * sin (y * X_PI);

    double theta = atan2 (y, x) + 0.000003 * cos (x * X_PI);

    lBdLon = z * cos(theta) + 0.0065;
    lBdLat = z * sin(theta) + 0.006;
    
}

// BD09 ==> GCJ02
void GPSPro::bd_decrypt (double lBdLat, double lBdLon, double& lGcjLat, double& lGcjLon)
{
    double x = lBdLon - 0.0065;
    double y = lBdLat - 0.006;
    double z = sqrt (x * x + y * y) - 0.00002 * sin (y * X_PI);

    double theta = atan2 (y, x) - 0.000003 * cos (x * X_PI);

    lGcjLon = z * cos (theta);
    lGcjLat = z * sin (theta);
}


void GPSPro::createJSON(string fileName,vector<pair<double,double> > GPSValue,int flag,vector<pair<int,string> > segmentColor)
{
    ofstream ofile;
    ofile.open(fileName.c_str());
    if(!ofile.is_open())
    {
        printf("ERROR: open %s error.\n",fileName.c_str());
        exit(0);
    }
    ofile.precision(15);

    int index = 0;

    if(0 == flag)
    {
        ofile << "[";
        ofile << "{\"line\":[";
        for(;index < GPSValue.size(); index ++)
        {
            ofile << "[" << GPSValue[index].first << "," << GPSValue[index].second << "],";
        }
        ofile << "],\"color\":\"" << "FF00FF" << "\"}";
        ofile << "]";
    }
    else
    {
        ofile << "[";
        for(int colorIndex = 0; colorIndex < segmentColor.size(); colorIndex ++)
        {
            ofile << "{\"line\":[";
            for(;index <= segmentColor[colorIndex].first; index ++)
            {
                ofile << "[" << GPSValue[index].first << "," << GPSValue[index].second << "],";
            }
            ofile << "],\"color\":\"" << segmentColor[colorIndex].second << "\"},";
        }
        ofile << "]";
    }
    ofile.close();
    cout << "finished map" << endl;
}
