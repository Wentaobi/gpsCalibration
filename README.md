# iMorpheus.ai - high availability sub-meter precise GPS
Through algorithm fusion of multiple data sources from different sensors such as lidar, radar, camera, gps, imu and point cloud, iMorpheus.ai brings about an high availability precision GPS measurement to outdoor robotics developer. iMorpheus integrate a number of advanced algorithm such as slam, kalman filter, ICP, feature selection and Gaussian Process. 
#### We believe precise GPS signal should obtained by computing from cloud rather than measure the satellite, and soly software and cloud can solve the problem rather than expensive hardware. So that we committed into only software to solve the problem. 
![image](https://github.com/iMorpheusAI/gpsCalibration/raw/develop/demo/demo.gif)
## Copyrights
![License](https://img.shields.io/badge/License-Apache2.0-blue.svg)

## Current Version - alpha
The alpha version is a software package operate under off-line mode and using point clould and GPS to give you accurate GPS. Each GPS signal produced also comes with confidence level. 

## Installation Environment

### 1. Operating System
Ubuntu 14.04, 16.04

### 2. Robot Operating System - ROS
ROS provides libraries and tools to help software developers create robot applications. It provides hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more. ROS is licensed under an open source, BSD license.
##### [Learn More](http://wiki.ros.org/ROS/Tutorials)

### 3. Point Cloud Library - PCL
PCL is a large scale, open source project for 2D/3D image and point cloud processing.
##### [Learn More](http://pointclouds.org/documentation/)

### 4. EIGEN
Eigen is a C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms.
##### [Learn More](http://eigen.tuxfamily.org/index.php?title=Main_Page)

## Quick Installation
### Download 
- Download by GIT
```
$ git clone https://github.com/iMorpheusAI/gpsCalibration.git
```
- Download by browser - [click to download](https://github.com/iMorpheusAI/gpsCalibration/archive/master.zip)<br/>
  After download, you can type command to decompress:
  ```
  $ unzip gpsCalibration-master.zip -d gpsCalibration
  ```

### Intallation
Under 'gpsCalibration/install' directory, we provide install scripts for beginners and developers. The scripts include installing ROS, PCL and EIGEN. <br/>
a) Basic version (Recommended for general users.) - one hour installation time.<br/>
b) Professional version (Recommended for developers.) 

- First download this repertory from github to your local system. 
- For ubuntu 14.04 basic version install, type commands:
```
$ cd gpsCalibration
$ cd install
$ sudo ./install_u1404_basic.sh 
```

## How to compile and run the project
### 1. Compile
In directory gpsCalibration, run command:
```
$ cd gpsCalibration
$ catkin_make 
```
### 2. Run
1.*Make sure you have the message bag. it includes follow message types
  sensor_msgs/PointCloud2 and GPS coordinates recording your run trail, even 
  GPS is not accurate and not continuous in time.
  If you don’t have lidar or GPS data, don’t worry, we have sample data for your trial.*
##### small size demo data -> [[download-191MB]](http://www.imorpheus.ai/download/dataForDemo/smallSizeDemoData)
##### large size demo data -> [[download-2.6GB]](http://www.imorpheus.ai/download/dataForDemo/largeSizeDemoData)
*Download the compressed demo data, put it into gpsCalibration/data and type commands to decompress:*
```
$ tar -zvxf small_size_demo_data.tar.gz
$ tar -zvxf large_size_demo_data.tar.gz 
```
*After decomproession of small_size_demo_data.tar.gz, you will see:*
```
$ miniDemo/
   ├── bag_0
   └── original_gps_data.txt
```
2.*Open the run.sh in directory "gpsCalibration/" and set needed file directory correctly.*
```
    User Parameters:
    Input filenames:
    1. bag_input_filename:
       input point cloud bag file list
       bag_input_filename= "./data/bag_list.txt"

    2. gps_input_filename:
       original GPS data with type GPRMC
       gps_input_filename= "./data/original_gps_data.txt"
       
    Output filenames:
    3. gps_original_filename:
       original GPS track type with KML format
       gps_original_filename=  "./data/original_gps_track.kml"
       
    4. gps_improved_filename:
       imporved accurate GPS track type with KML format
       gps_improved_filename=  "./data/calibrated_gps_track.kml"
```
3.*In directory gpsCalibration, run commands:*
```
$ source setup.sh
$ cd data
$ vi bag_list.txt
  modify the point cloud data bag path: ./data/miniDemo/bag_0
$ cp ./miniDemo/original_gps_data.txt ./
```
4.*Okay, you can run command:*
```
$ ./run.sh
```
5.*Finally, you get a global position system coordinates matched with your run trail. It is accurate and reliable!*

### 3. Example
  We show calibrated results of large size demo data. 
#### 3.1 demo results
##### [Results Download](http://www.imorpheus.ai/download/dataForDemo/largeSizeDemoResult)
Download compressed demo results and type commands to decompress: 
```
$ tar -zvxf large_size_demo_result.tar.gz
$ tree large_size_demo_result
  ├── calibrated_gps_track.kml
  └── original_gps_track.kml
```
Open these KML files in google earth, you can check your results.
##### [See More](http://www.imorpheus.ai/demo/)
## About system input and output
### 1. Input
#### 1.1 .bag
A bag is a file format in ROS for storing ROS message data.
 
#### 1.2 GPS
The GPSRMC is protocol of GPSRMC's communication:
$GPRMC,085223.136,A,3957.6286,N,11619.2078,E,0.06,36.81,180908,,,A\*57

### 2. Output
The results are stored in gpsCalibration/data. We provide calibrated GPS in KML formats.<br/>
You can download Google Earth [here](https://www.google.com/earth/download/ge/) and open KML files.

## Questions
  You can ask any question [here](https://github.com/iMorpheusAI/gpsCalibration/issues) or send us [emails](product@imorpheus.ai).
