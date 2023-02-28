#pragma once
#include <ros/ros.h>
#include "sensor_msgs/PointCloud.h"
#include <QtWidgets>
#include <QtDataVisualization>

class PointCloudViewer_W : public QWidget{
    Q_OBJECT

    public: 
        PointCloudViewer_W(ros::NodeHandle& nh);
    private:
        void pointCloudCallback(const sensor_msgs::PointCloudConstPtr&);

        QtDataVisualization::Q3DScatter *scatter_plot;
    protected:
        ros::NodeHandle nh_;
        sensor_msgs::PointCloud curr_cloud;
        ros::Subscriber pcl_sub;
};