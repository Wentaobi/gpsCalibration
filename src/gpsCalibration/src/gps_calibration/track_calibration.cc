#include "track_calibration.h"

//track data initial
trackCalibration::trackCalibration(vector<COORDXYZT> &SLAMTrackTmp, vector<COORDXYZT> &ENUTrackTmp,vector<double> weightCoeTmp)
{
    //initial the track, weight matrix
    dataInitial(SLAMTrackTmp, ENUTrackTmp, weightCoeTmp);

    //weighted ICP at present
    isAddWeight= true;

}

//track ICP
int trackCalibration::doICP()
{
    
    MatrixXd transformT;
    MatrixXd distancesD;

    //icp
    icp(&transformT, &distancesD);

    //rotated;
    coordRotated(transformT);

    return 1;
}

//track calibration
int trackCalibration::doCalibration(vector<COORDXYZT> &calENUTrack)
{
    //calibration
    calibrateGPSWithSLAMTrack(calENUTrack);

    return 1;
}

//initial the vector date temp
int trackCalibration::dataInitial(vector<COORDXYZT> &SLAMTrackTmp, vector<COORDXYZT> &ENUTrackTmp, vector<double> &weightCoeTmp)
{
    int SLAMLineNo= SLAMTrackTmp.size(); 
    int ENULineNo= ENUTrackTmp.size(); 
    int weightLineNo= weightCoeTmp.size(); 

    if((SLAMLineNo!=ENULineNo)||(SLAMLineNo!=weightLineNo)||(ENULineNo!=weightLineNo))
    {
        cout << "there's something wrong in icp data no, please check it out" << endl;
        exit(1);
    }

    //SLAM track coord
    SLAMCoord.setOnes(SLAMLineNo, 4);
    for(int i= 0; i< SLAMLineNo; i++) 
    {    
        SLAMCoord(i, 0)= SLAMTrackTmp[i].x- SLAMTrackTmp[0].x;
        SLAMCoord(i, 1)= SLAMTrackTmp[i].y- SLAMTrackTmp[0].y;
    }   

    //ENU track coord
    ENUCoord.setOnes(ENULineNo, 4);
    for(int i= 0; i< ENULineNo; i++) 
    {    
        if(0==i)
        {
            ENUX0= ENUTrackTmp[i].x;
            ENUY0= ENUTrackTmp[i].y;
        }
        ENUCoord(i, 0)= ENUTrackTmp[i].x- ENUX0;
        ENUCoord(i, 1)= ENUTrackTmp[i].y- ENUY0;
    }   
    
    //weight coefficient
    weightCoord.setOnes(weightLineNo, 1);

    for(int i= 0; i< weightLineNo; i++) 
    {    
        weightCoord(i)= weightCoeTmp[i];
    }

    //calibrated track
    COORDXYZT trackTmp;
    for(int i= 0; i< ENULineNo; i++)
    {
        trackTmp.x= 0.0;
        trackTmp.y= 0.0;
        trackTmp.z= ENUTrackTmp[i].z;
        trackTmp.t= ENUTrackTmp[i].t;
        //trackTmp.w= weightCoeTmp[i];

        calENUTrackTmp.push_back(trackTmp);
    }

    numPoint= SLAMLineNo;

    return 1;
}

//icp
int trackCalibration::icp(MatrixXd *outT,MatrixXd *outD)
{
    int iNumStart = 0;
    int iNumEnd = SLAMCoord.rows();
    int numPointICP = iNumEnd - iNumStart;

    //get slamCoordAll's rows and cols
    int slamTrackRows = SLAMCoord.rows() ;
    int slamTrackCols = SLAMCoord.cols() ;

    //get gpsCoordAll's rows and cols
    int gpsTrackRows = ENUCoord.rows() ;
    int gpsTrackCols = ENUCoord.cols() ;

    //N*3 matrix of source 3D points
    MatrixXd src;
    //N*3 matrix of destination 3D point
    MatrixXd dst;

    //N*1 Euclidean distances (errors) of the nearest neighbor
    MatrixXd tempDistance;
    MatrixXd tempIndice;

    MatrixXd _T;
    
    //src:N*4,dst:N*4，use "1" to fill the matrix
    src.setOnes(slamTrackRows, slamTrackCols);
    dst.setOnes(gpsTrackRows, gpsTrackCols);
    _T.setOnes(4, 4);
    
    //get the slam track 
    for(int i= 0; i< slamTrackRows; i++)
    {
       for(int j= 0; j< slamTrackCols- 1; j++)
       {
           src(i,j)= SLAMCoord(i,j);	
       }
    }

    //get the GPS track
    for(int i= 0; i< gpsTrackRows; i++)
    {
       for(int j= 0; j< gpsTrackCols- 1; j++)
       {
           dst(i, j)= ENUCoord(i, j);
       }
    }
    
    int maxIterations= 2;

    double prevError= 0;
    
    for(int i= 0; i< maxIterations; i++)
    {
        //find the nearest neighbours between the current source and destination points
        nearestNeighbor(src, dst, &tempDistance, &tempIndice);

        //compute the transformation between the current source and nearest destination points
        if(isAddWeight)
        {
            BFTWithWeight(src, dst, &_T);
        }
        else
        {
            bestFitTransform(src, dst, &_T);
        }
        
        //update the current source
        src =  src * _T.transpose();
        
        //check error
        double meanError = 0.0;
        
        for(int j=0;j<tempDistance.rows();j++)
        {
            meanError += tempDistance(j,0);
        }
        meanError = meanError/(tempDistance.rows());
        
        if(abs(prevError-meanError)< 0.003)
        {     
            break;
        }
        prevError = meanError;
    }
    
    //final homogeneous transformation
    MatrixXd tempTransformT;

    //calculate final transformation
    if(isAddWeight)
    {
        BFTWithWeight(SLAMCoord, src, &tempTransformT);
    }
    else
    {
        bestFitTransform(SLAMCoord, src, &tempTransformT);
    }


    *outT = tempTransformT;
    *outD = tempDistance;

    return 1;
}

/*Calculates the least-squares best-fit transform between corresponding 3D points A->B
    Input:
        A: Nx3 numpy array of corresponding 3D points
        B: Nx3 numpy array of corresponding 3D points
    Returns:
        T: 4x4 homogeneous transformation matrix
        R: 3x3 rotation m)atrix
        t: 3x1 column vector
*/
int trackCalibration::bestFitTransform(MatrixXd A,MatrixXd B,MatrixXd *outputT)
{
    int ALength= A.rows();
    int BLength= B.rows();

    double sumA_X = 0.0;
    double sumA_Y = 0.0;
    double sumA_Z = 0.0;

    double sumB_X = 0.0;
    double sumB_Y = 0.0;
    double sumB_Z = 0.0;

    //N*3,translate points to their centroids
    MatrixXd centroid_A;
    MatrixXd centroid_B;
    
    //N*3
    MatrixXd AA;
    MatrixXd BB;

    //N*3
    MatrixXd tempA;
    MatrixXd tempB;

    //1*3
    MatrixXd tempAA;
    MatrixXd tempBB;

    centroid_A.setOnes(ALength, 3);
    centroid_B.setOnes(BLength, 3);

    AA.setOnes(ALength, 3);
    BB.setOnes(BLength, 3);

    tempA.setOnes(ALength, 3);
    tempB.setOnes(BLength, 3);

    tempAA.setOnes(1, 3);
    tempBB.setOnes(1, 3);
    
    for(int i= 0; i< ALength; i++)
    {
        for(int j= 0; j< 3; j++)
        {
            if(j== 0)
            {
                sumA_X+= A(i,j);
                sumB_X+= B(i,j);
            }
            else if(j== 1)
            {
                sumA_Y+= A(i,j);
                sumB_Y+= B(i,j);
            }
            else
            {
                sumA_Z+= A(i,j);
                sumB_Z+= B(i,j);
            }   
        }
    }
    
    for(int i= 0; i< ALength; i++)
    {
       for(int j= 0; j< 3; j++)
        {
            tempA(i, j)= A(i, j);
            tempB(i, j)= B(i, j);
        }
    }
    
    sumA_X= sumA_X/ ALength;
    sumA_Y= sumA_Y/ ALength;
    sumA_Z= sumA_Z/ ALength;
    
    sumB_X= sumB_X/ BLength;
    sumB_Y= sumB_Y/ BLength;
    sumB_Z= sumB_Z/ BLength;
    
    for(int i= 0; i< 3; i++)
    {
        if(i== 0)
        {
            tempAA(0, i)= sumA_X;
            tempBB(0, i)= sumB_X;
        }
        else if(i==1)
        {
            tempAA(0, i)= sumA_Y;
            tempBB(0, i)= sumB_Y;
        }
        else
        {
            tempAA(0, i)= sumA_Z;
            tempBB(0, i)= sumB_Z;
        }
    }
    
    for(int i= 0; i< ALength; i++)
    {
        centroid_A(i, 0)= sumA_X;
        centroid_A(i, 1)= sumA_Y;
        centroid_A(i, 2)= sumA_Z;

        centroid_B(i, 0)= sumB_X;
        centroid_B(i, 1)= sumB_Y;
        centroid_B(i, 2)= sumB_Z;
    }
    
    //translate points to their centroids
    AA= tempA- centroid_A;
    BB= tempB- centroid_B;

    //rotation matrix
    MatrixXd H= (AA.transpose())*(BB);

    JacobiSVD<Eigen::MatrixXd> svd(H,ComputeThinU | ComputeThinV );
    MatrixXd U= svd.matrixU();
    MatrixXd Vt= svd.matrixV();
    MatrixXd S= svd.singularValues();

    MatrixXd R= Vt*U.transpose();
    
    //special reflection case
    if(R.determinant()< 0)
    {
        for(int i= 0; i< Vt.cols(); i++)
        {
             Vt(i, 2) = (-1) * Vt(i, 2);
        }
        R = Vt* U.transpose();
    }

    //translation
    MatrixXd t= tempBB.transpose()- ((R)*(tempAA.transpose()));

    //homogeneous transformation
    MatrixXd T;
    T.setIdentity(4, 4);

    for(int i= 0; i< 3;i++)
    {
        for(int j= 0; j< 3; j++)
        {  
            T(i, j)= R(i, j);
        }
        double temp= t(i, 0);
        T(i,3) = temp ;
    }
    *outputT= T;
    
    return 1;
}

/*Calculates the least-squares best-fit transform between corresponding 3D points A->B
    Input:
        A: N*3 the matrix of corresponding 3D points
        B: Nx3 the matrix of corresponding 3D points
    Returns:
        T: 4x4 homogeneous transformation matrix
        R: 3x3 rotation matrix
        t: 3x1 column vector
*/
int trackCalibration::BFTWithWeight(MatrixXd A,MatrixXd B,MatrixXd *outputT)
{
    int ALength= A.rows();
    int BLength= B.rows();

    double sumA_X= 0.0;
    double sumA_Y= 0.0;
    double sumA_Z= 0.0;

    double sumB_X= 0.0;
    double sumB_Y= 0.0;
    double sumB_Z= 0.0;
    
    double sumW = 0.0;

    //N*3,translate points to their centroids
    MatrixXd centroid_A;
    MatrixXd centroid_B;
    
    //N*3
    MatrixXd A1;
    MatrixXd B1;   

    //N*3
    MatrixXd AA;
    MatrixXd BB;

    //N*3
    MatrixXd tempA;
    MatrixXd tempB;

    //1*3
    MatrixXd tempAA;
    MatrixXd tempBB;

    A1.setOnes(ALength,3);
    B1.setOnes(BLength,3);

    centroid_A.setOnes(ALength, 3);
    centroid_B.setOnes(BLength, 3);

    AA.setOnes(ALength, 3);
    BB.setOnes(BLength, 3);

    tempA.setOnes(ALength, 3);
    tempB.setOnes(BLength, 3);

    tempAA.setOnes(1, 3);
    tempBB.setOnes(1, 3);
    
    for(int i= 0; i< ALength; i++)
    {
        for(int j= 0; j< 3; j++)
        {
            if(j== 0)
            {
                A1(i, j)= A(i, j)* weightCoord(i);
                B1(i, j)= B(i, j)* weightCoord(i);
            }
            else if(j==1)
            {
                A1(i, j)= A(i, j)* weightCoord(i);
                B1(i, j)= B(i, j)* weightCoord(i);
            }
            else
            {
                A1(i, j)= A(i, j)* weightCoord(i);
                B1(i, j)= B(i, j)* weightCoord(i);
            }
        }
    }
   
    for(int i= 0; i< ALength; i++)
    {
        for(int j= 0; j< 3; j++)
        {
            if(j==0)
            {
                sumA_X+= A1(i, j);
                sumB_X+= B1(i, j);
            }
            else if(j== 1)
            {
                sumA_Y+= A1(i, j);
                sumB_Y+= B1(i, j);
            }
            else
            {
                sumA_Z+= A1(i, j);
                sumB_Z+= B1(i, j);
            }  
        }
        sumW+= weightCoord(i);
    }
   
    for(int i= 0; i< ALength; i++)
    {
        for(int j= 0; j< 3; j++)
        {
            tempA(i, j)= A(i, j);
            tempB(i, j)= B(i, j);
        }
    }
    
    sumA_X= sumA_X/ sumW;
    sumA_Y= sumA_Y/ sumW;
    sumA_Z= sumA_Z/ sumW;
    
    sumB_X= sumB_X/ sumW;
    sumB_Y= sumB_Y/ sumW;
    sumB_Z= sumB_Z/ sumW;
    
    for(int i= 0; i< 3; i++)
    {
        if(i==0)
        {
            tempAA(0, i)= sumA_X;
            tempBB(0, i)= sumB_X;
        }
        else if(i==1)
        {
            tempAA(0, i)= sumA_Y;
            tempBB(0, i)= sumB_Y;
         }
        else
        {
            tempAA(0,i)= sumA_Z;
            tempBB(0,i)= sumB_Z;
        }
    }
    
    for(int i= 0; i< ALength; i++)
    {
        centroid_A(i, 0)= sumA_X;
        centroid_A(i, 1)= sumA_Y;
        centroid_A(i, 2)= sumA_Z;

        centroid_B(i, 0)= sumB_X;
        centroid_B(i, 1)= sumB_Y;
        centroid_B(i, 2)= sumB_Z;
    }
    
    //translate points to their centroids
    AA= tempA- centroid_A;
    BB= tempB- centroid_B;
    
    for(int i= 0; i< ALength; i++)
    {
        for(int j= 0; j< 3; j++)
        {
            if(j==0)
            {
                 AA(i, j)= AA(i, j)* weightCoord(i);
                 BB(i, j)= BB(i, j)* weightCoord(i);
            }
            else if(j==1)
            {
                 AA(i, j)= AA(i, j)* weightCoord(i);
                 BB(i, j)= BB(i, j)* weightCoord(i);
            }
            else
            {    
                 AA(i, j)= AA(i, j)* weightCoord(i);
		         BB(i, j)= BB(i, j)* weightCoord(i);
            }
        }
    }
   
    //rotation matrix
    MatrixXd H= (AA.transpose())*(BB);

    JacobiSVD<Eigen::MatrixXd> svd(H,ComputeThinU | ComputeThinV );
    MatrixXd U= svd.matrixU();
    MatrixXd Vt= svd.matrixV();
    MatrixXd S= svd.singularValues();
   
    MatrixXd R= Vt*U.transpose();

    //special reflection case
    if(R.determinant()< 0)
    {
        for(int i= 0; i< Vt.cols(); i++)
        {
            Vt(i, 2)= (-1) * Vt(i, 2);
        }
        R= Vt* U.transpose();
    }

    //translation
    MatrixXd t= tempBB.transpose()- ((R)* (tempAA.transpose()));
    
    //homogeneous transformation
    MatrixXd T;
    T.setIdentity(4, 4);

    for(int i= 0; i< 3; i++)
    {
        for(int j= 0; j< 3; j++)
        {
           
            T(i, j)= R(i, j);
        }
        double temp= t(i, 0);
        T(i,3)= temp ;
    }
    *outputT= T;

    return 1;
}

/*Find the nearest (Euclidean) neighbor in dst for each point in src
    Input:
        src: Nx3 array of points
        dst: Nx3 array of points
    Output:
        distances: Euclidean distances of the nearest neighbor
        indices: dst indices of the right neighbor 
*/
int trackCalibration::nearestNeighbor(MatrixXd inputSrc, MatrixXd inputDst, MatrixXd *outputDistances, MatrixXd *outputIndices)
{
    //the matrix of inputSrc's rows;
    int srcLength= inputSrc.rows();

    //count 
    int jNum= 0;
    
    //N*1, index:0~N
    MatrixXd tempIndices;
    tempIndices.setOnes(srcLength, 1);
    
    //N*N, euclidean distance 
    MatrixXd tempDistance;
    tempDistance.setOnes(srcLength, srcLength);
    
    //N*1, euclidean distances of the nearest neighbor
    MatrixXd tempDst;
    tempDst.setOnes(srcLength, 1);

    for(int i= 0; i< srcLength; i++)
    {
        tempIndices(i,0)= i;
    }
    
    *outputIndices= tempIndices;

    double disx= 0.0;
    double disy= 0.0;

    for(int i= 0; i< srcLength; i++)
    {   
        for(int j= 0; j< srcLength; j++)
        {
            disx= inputSrc(i, 0)- inputDst(j, 0);
            disy= inputSrc(i, 1)- inputDst(j, 1);
            tempDistance(i, j)= sqrt(disx* disx+ disy* disy);
        }
    }
    
    for(int i= 0; i< srcLength; i++)
    {
        tempDst(i,0)= tempDistance(i, jNum);
        jNum++;
    }

    *outputDistances= tempDst;  

    return 1;
}

//rotate the SLAM track into ENU coordinate
int trackCalibration::coordRotated(MatrixXd transformT)
{
    //N*3
    MatrixXd tempSLAMCoord;
    MatrixXd tempT;
    MatrixXd tempR;
    
    tempSLAMCoord.setOnes(SLAMCoord.rows(),3);
    tempT.setOnes(SLAMCoord.rows(), 3);
    tempR.setOnes(3, 3);
    
    //Get t from transformT
    for(int i= 0; i< SLAMCoord.rows(); i++)
    {
        for(int j= 0; j< 3; j++)
        {
            tempSLAMCoord(i, j)= SLAMCoord(i, j);
            tempT(i, j)= transformT(j, 3);
        }
    }

    //Get r from transformT
    for(int i= 0; i< 3; i++)
    {
        for(int j= 0; j< 3; j++)
        {
            tempR(i, j)= transformT(i, j);
        }
    }

    //Get the rotation of slam track
    SLAMRotatedCoord= tempSLAMCoord*(tempR.transpose()) + tempT;

    return 1;
}

/*
    Calibrate the GPS track by rotated slam track
        outPut: the average of whole regressed GPS track and rotated slam track
*/
int trackCalibration::calibrateGPSWithSLAMTrack(vector<COORDXYZT> &calENUTrack)
{ 
    int iNum;
    int iCoord;

    double averageX;
    double averageY;
    double disX;
    double disY;

    MatrixXd tempGPSReCoord;
    MatrixXd tempGPSReAverCoord;

    tempGPSReCoord.setOnes(numPoint, 2);
    tempGPSReAverCoord.setOnes(numPoint, 2);

    iNum= 0;
    while(iNum< numPoint)
    {
        averageX= 0.0;
        averageY= 0.0;
        iCoord= 0;
        
        while(iCoord< numPoint)
        {
            //calculate the distance between slam track points
            disX= SLAMRotatedCoord(iCoord, 0)- SLAMRotatedCoord(iNum, 0);
            disY= SLAMRotatedCoord(iCoord, 1)- SLAMRotatedCoord(iNum, 1);
        
            //regressed GPS dots dopend on iNum Slam Track dot
            averageX+= ENUCoord(iCoord, 0)- disX;
            averageY+= ENUCoord(iCoord, 1)- disY;
            iCoord++;
        }
        
        averageX/= numPoint;
        averageY/= numPoint;

        //regressed points, get the average
        tempGPSReAverCoord(iNum, 0)= (averageX+ SLAMRotatedCoord(iNum, 0))/ 2.0;
        tempGPSReAverCoord(iNum, 1)= (averageY+ SLAMRotatedCoord(iNum, 1))/ 2.0; 
        
        iNum++;
    }

    //the results
    COORDXYZT trackTmp;
    for(int i= 0; i< numPoint; i++)
    {
        trackTmp.x= tempGPSReAverCoord(i, 0)+ ENUX0;
        trackTmp.y= tempGPSReAverCoord(i, 1)+ ENUY0;
        trackTmp.z= calENUTrackTmp[i].z;
        trackTmp.t= calENUTrackTmp[i].t;
        //trackTmp.w= calENUTrackTmp[i].w;
        calENUTrack.push_back(trackTmp);
    }

    return 1;
}
