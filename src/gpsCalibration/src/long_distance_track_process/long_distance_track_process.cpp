#include <fstream>
#include <iostream>
#include <stdio.h>
#include <iomanip>

#include "weight_calculation.h"
#include "long_distance_track_process.h"

using namespace std;

int LongDisTrackPro::calcICPweightCoe(int coeSort)
{
    this->coeSort=coeSort;
    if(coeSort==1)
    {
        wcca.ICPWeightCoeCal(this->SLAMTrackTmp,this->weightCoe);
        return 1;
    }
    else if(coeSort==2)
    {
        wcca.ICPWeightCoeCal(this->SLAMTrackTmp,this->weightCoe, this->ENUOriTMP, this->SLAMRotatedTrackTmp);
        return 2;
    }else{
        std::cerr<<"parameter error need 1 or 2"<<std::endl;
        return -1;
    }
}
//type 0 is init 1 is update 
void LongDisTrackPro::initOrupdateEnuori(vector<COORDXYZTW> ENUOriTMP,int len,int type)
{
    if(type==0)
    {
        for(int i=0;i<len;++i)
        {
            this->ENUOriTMP.push_back(ENUOriTMP[i]);
        }
    }
    else if(type==1)
    {
        for(int i=0;i<len;++i)
        {
            this->ENUOriTMP[i]=ENUOriTMP[i];
        }
    }
}
void LongDisTrackPro::initOrupdateSLAMRotatedTrack(vector<COORDXYZTW> SLAMRotatedTrackTmp,int len,int type)
{
    if(type==0)
    {
        for(int i=0;i<len;++i)
        {
            this->SLAMRotatedTrackTmp.push_back(SLAMRotatedTrackTmp[i]);
        }
    }
    else if(type==1)
    {
        for(int i=0;i<len;++i)
        {
            this->SLAMRotatedTrackTmp[i]=SLAMRotatedTrackTmp[i];
        }
    }
}
void LongDisTrackPro::updateSLAMtrackTmp(vector<COORDXYZTW> proENUTrack,int len)
{
    for(int i;i<len;++i)
    {
        this->SLAMTrackTmp[i]=proENUTrack[i];
    }
}
void LongDisTrackPro::updateWeightCoe(vector<float> weightCoe,int len)
{
    for(int i=0;i<len;++i)
    {
        this->weightCoe[i]=weightCoe[i];
    }
}
void LongDisTrackPro::mergeENUOriwithWeight()
{
    COORDXYZTW temp;
    if(this->ENUOriTMP.size() != this->weightCoe.size())
    {
        cerr<<"can not merge because they have not same size"<<endl;
        cout<<"ENUOriTMPsize= "<<this->ENUOriTMP.size()<<" weightCoesize= "<<this->weightCoe.size()<<endl;
        exit(-1);
    }
    for(int i=0;i<this->ENUOriTMP.size();++i)
    {
        temp.x=ENUOriTMP[i].x;
        temp.y=ENUOriTMP[i].y;
        temp.z=ENUOriTMP[i].z;
        temp.t=ENUOriTMP[i].t;
        temp.w=this->weightCoe[i];
        ENUOriFinal.push_back(temp);
    }
    this->weightCoe.clear();
    this->SLAMRotatedTrackTmp.clear();
    this->ENUOriTMP.clear();
}
LongDisTrackPro::LongDisTrackPro(string originalGPSPath1,string method1,int type1)                                                                               
{
    gps.setGPSPath(originalGPSPath1);
    gps.setMethod(method1);
    gps.setType(type1);
}
int LongDisTrackPro::outputData()
{
     ofstream ifile;
     ifile.open("mergefile.txt",fstream::out); 
     if(NULL==ifile)
     {
         printf("open  error\n");
         return 1;
     }
     vector<COORDXYZTW>::iterator p;
     for(p= this->ENUOriFinal.begin(); p!= this->ENUOriFinal.end();p++)
     {   
         ifile<<setprecision(15)<< p->x<<" "<< p->y<< " "<< p->z<<" "<< p->t<<" "<<p->w<<endl;
     }
     ifile.close();
     return 1;
}
void LongDisTrackPro::msgAssign(gpsCalibration::IMTrack & msg)
{
    gpsCalibration::IMLocalXYZT temp;
    for(int i=0;i<ENUOriFinal.size();++i)
    {
        temp.x=ENUOriFinal[i].x;
        temp.y=ENUOriFinal[i].y;
        temp.z=ENUOriFinal[i].z;
        temp.t=ENUOriFinal[i].t;
        temp.w=ENUOriFinal[i].w;
        msg.track.push_back(temp);
    }
}
