#ifndef _TRACK_CALIBRATION_H_ 
#define _TRACK_CALIBRATION_H_ 

#include "common.h"
#include <algorithm>
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

using namespace Eigen;

class trackCalibration
{
    public:
        //track ICP, initial the private variables
        trackCalibration(vector<COORDXYZTW> &SLAMTrackTmp, vector<COORDXYZTW> &ENUTrackTmp,vector<double> weightCoeTmp);

        //do the icp
        int doICP();

        //do the calibration
        int doCalibration(vector<COORDXYZTW> &calENUTrack);

    private:
        //Define the numPoint to get the Point's number
        int numPoint;

        //Define the numPoint to get the Point's number
        bool isAddWeight;

        //Define the array to get slam track and GPS track
        double ENUX0;
        double ENUY0;

        //temp of calibrated ENU track
        vector<COORDXYZTW> calENUTrackTmp;

        //Define the matrix
        MatrixXd SLAMCoord;
        MatrixXd ENUCoord;
        MatrixXd weightCoord;

        MatrixXd SLAMRotatedCoord;

        //initial the vector date temp
        int dataInitial(vector<COORDXYZTW> &SLAMTrackTmp, vector<COORDXYZTW> &ENUTrackTmp, vector<double> &weightCoeTmp);

        //icp
        int icp(MatrixXd *outT,MatrixXd *outD);

        //best fit for no weight icp
        int bestFitTransform(MatrixXd A,MatrixXd B,MatrixXd *outputT);

        //best fit for weighted icp
        int BFTWithWeight(MatrixXd A,MatrixXd B,MatrixXd *outputT);

        //find the nearest distance
        int nearestNeighbor(MatrixXd inputSrc, MatrixXd inputDst, MatrixXd *outputDistances, MatrixXd *outputIndices);

        //rotate the SLAM track into ENU coordinate
        int coordRotated(MatrixXd transMatrix);

        //calibrate the ENU track by rotated SLAM track
        int calibrateGPSWithSLAMTrack(vector<COORDXYZTW>& track);
};
#endif
