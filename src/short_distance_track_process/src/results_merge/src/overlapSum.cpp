#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

using namespace std;

typedef struct
{
    double x;
    double y;
    double z;
    double t;
    double w;
}TRACK;

int getNo(vector<TRACK> &a, vector<TRACK> &b, int *noStart, int *noEnd)
{
    //find the start index of b
    for(int i= 0; i< b.size(); i++)
    {
        if(fabs(b[i].t- a[0].t)< 0.0001)
            *noStart= i;
    }

    //find the end index of a
    for(int i= 0; i< a.size(); i++)
    {
        if(fabs(a[i].t- b[b.size()- 1].t)< 0.0001)
            *noEnd= i;
    }
    
    return 0;
}

int overlapSum(vector<TRACK> &a, vector<TRACK> &b)
{
    vector<TRACK>::iterator tmp;
    TRACK trackTmp;
    int noStart, noEnd;
    
    //first time
    if(0==b.size())
    {
        for(tmp= a.begin(); tmp!= a.end(); tmp++)
        {
            trackTmp.x= tmp->x;
            trackTmp.y= tmp->y;
            trackTmp.z= tmp->z;
            trackTmp.t= tmp->t;

            b.push_back(trackTmp);
        }
    }
    else
    {
        //find the start no of b and end no of a
        getNo(a, b, &noStart, &noEnd);
  
        float coe1, coe2;
        int num= 0;
        int smWindow= noEnd/2;
        //add
        for(int i= 0; i< a.size(); ++i)
        {
            if(i<= noEnd)
            {
                //1st coe part
                if(i <= smWindow)
                {
                    coe1= (1.0- num/(2.0* smWindow));
                    coe2= num/(2.0* smWindow);
                }

                //2nd coe part
                else if((num> smWindow)&&(num<= noEnd- smWindow))
                {
                    coe1= 0.5;
                    coe2= 0.5;
                }

                //3rd coe part
                else if(num> (noEnd- smWindow))
                {
                    coe1= (noEnd- num+ 1)/(2.0* smWindow);
                    coe2= (1.0- (noEnd- num+ 1)/(2.0* smWindow));
                }

                int bIndex= i+ noStart;
                b[bIndex].x= a[i].x* coe1+ b[bIndex].x* coe2;
                b[bIndex].y= a[i].y* coe1+ b[bIndex].y* coe2;
                b[bIndex].z= a[i].z* coe1+ b[bIndex].z* coe2;
                b[bIndex].w= a[i].w* coe1+ b[bIndex].w* coe2;

                num++;
            }
            else
            {
                trackTmp.x= a[i].x;
                trackTmp.y= a[i].y;
                trackTmp.z= a[i].z;
                trackTmp.t= a[i].t;

                b.push_back(trackTmp);
            }
        }
    }
    
    return 0;
}
