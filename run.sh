#!/bin/bash

<<!
    User Parameters:
    
    Input filenames:
    1. bag_input_filename:
       input point cloud bag file list

    2. gps_input_filename:
       original GPS data with type GPRMC

    3. result_control:
       output map files or publish messages to other nodes
       1- output Google Earth KML files(.kml or .KML must be the suffix)
       2- output Baidu map .bmap files(.bmap is suggested to be used as suffix)
       3- output Gaode map .amap files(.amap is suggested to be used as suffix)
       4- publish messages to other nodes

    Output filenames, if needed:
    3. gps_original_filename:
       original GPS track type with KML format

    4. gps_improved_filename:
       imporved accurate GPS track type with KML format
!
bag_input_filename="./data/bag_list.txt"
gps_input_filename="./data/original_gps_data.txt"
result_control=1
gps_original_filename="./data/original_gps_file.kml"
gps_improved_filename="./data/calibration_gps_file.kml"

<<!
    Project parameters:
    1. total_long_distance:
        length of long distance lidar SLAM
        (600m-1000m, recommended)

    2. total_short_distance:
        length of short distance lidar SLAM
        (200m-300m, recommended)

    3. overlap_distance:
        length of overlapped short distance lidar SLAM
!
total_long_distance=1000
total_short_distance=300
overlap_distance=100

<<!
    Default Parameters:
    1. ctm:
       WGS-84 coordinate to ENU local coordinates method:
       UTM, Gaussian- UTM recommended

    2. gdt:
       GPS divide_type refers to longitude:
       3, 6- 3 recommended
!
ctm="UTM"
gdt="3"

bd="./data/bd.json"

#command
roslaunch gpsCalibration gpsCalibration.launch bag_input_filename:=${bag_input_filename} total_long_distance:=${total_long_distance} total_short_distance:=${total_short_distance} \
gps_input_filename:=${gps_input_filename} ctm:=${ctm} gdt:=${gdt} result_control:=${result_control} overlap_distance:=${overlap_distance} gps_original_filename:=${gps_original_filename} \
gps_improved_filename:=${gps_improved_filename} bd:=${bd}
