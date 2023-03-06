#include "RosTranslation.h"
#include <iostream>

RosTranslation::RosTranslation(ros::NodeHandle& nh, QWidget *parent) : 
    nh_m(nh) ,
    QWidget(parent)
{
    // create the subscriber
    // lscan_sub = nh_.subscribe("base_scan",3,&RosTranslation::updateScanData,this);
    ROS_INFO("created ros translation");   
}

RosTranslation::~RosTranslation()
{
}

void RosTranslation::updateScanData(const sensor_msgs::LaserScan& laser_scan_msg){
    /*
    Description:
        convert the laser scan to xyz datapoints, then emit the data as a signal.
    */

    std::vector<float> scan_data_vec = laser_scan_msg.ranges;
    float angle_inc = laser_scan_msg.angle_increment;
    float angle_min = laser_scan_msg.angle_min;
    float angle_max = laser_scan_msg.angle_max;
    float scan_range_min = laser_scan_msg.range_min;
    float scan_range_max = laser_scan_msg.range_max;

    std::vector<std::vector<float>> out_scan;
    for(int i=0; i<3; i++){std::vector<float> v; 
        out_scan.push_back(v);}

    float angle = angle_min;
    float x,y,z;
    for(int i=0; i<scan_data_vec.size(); i++){
        if ((scan_data_vec[i]<angle_max) & (scan_data_vec[i]>angle_min)){
            // throw out out of range data points
            x = scan_data_vec[i] * std::cos(angle);
            y = scan_data_vec[i] * std::sin(angle);
            z = 0;
            out_scan[0].push_back(x);
            out_scan[1].push_back(y);
            out_scan[2].push_back(z);
        }
        angle += angle_inc;
    }
    emit laserScanValueChanged(out_scan);
}

void RosTranslation::addSubscriber(std::string& sub){
    lscan_sub_m = nh_m.subscribe(sub,3,&RosTranslation::updateScanData,this);
    // subscriber_watchdog = new QTimer(this)
}