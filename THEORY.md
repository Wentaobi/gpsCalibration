## Module description
### 1. Modules
gpsCalibration has three modules include GPS, LOAM and Calibration.

#### 1.1 GPS module
This module processes GPS data and transforms them into local coordinate system.

#### 1.2 LOAM module
LOAM(Laser Odometry and Mapping) is a real-time method for state estimation and mapping using 3D lidar. The program contains two major threads running in parallel. 
The "odometry" thread computes motion of the lidar between two sweeps, at a higher frame rate. It also removes distortion in the point cloud caused by motion of the lidar. The "mapping" thread takes the undistorted point cloud and incrementally builds a map, while simultaneously computes pose of the lidar on the map at a lower frame rate. The lidar state estimation is combination of the outputs from the two threads.

To learn more about LOAM, please refer to [paper](http://www.frc.ri.cmu.edu/%7Ejizhang03/Publications/RSS_2014.pdf).

This package is a simple modified copy of loam_velodyne GitHub repository from laboshinl, https://github.com/laboshinl/loam_velodyne

We modified laserOdometry.cpp and scanRegistration.cpp to enhance lidar io, and added some code on transformMaintenance.cpp to get lidar's track with timestamp and map.

#### 1.3 Calibration module
This module calls GPS module and LOAM module and use their data as input.

We use timestamped point set registration to match GPS and LOAM tracks in two steps. First, we calculate long LOAM segments and find their least absolute deviations with GPS track by means of iteratively re-weighted least squares. Second, the weights from the first step are used to register overlapping short LOAM segments with GPS track by means of weighted least squares.

The final output is calibrated GPS track. We only work on 2D GPS tracks in this version. Altitudes are set to a constant for the purpose of demonstration in Google Earth.

#### 1.4 Flowchart
![image](https://github.com/iMorpheusAI/gpsCalibration/raw/master/demo/flowchart.jpg)
