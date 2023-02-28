#include "PCViewerWidget.h"

PointCloudViewer_W::PointCloudViewer_W(ros::NodeHandle& nh) : scatter_plot(new QtDataVisualization::Q3DScatter){
    pcl_sub = nh.subscribe("scan",3,&PointCloudViewer_W::pointCloudCallback, this);
   // setCentralWidget(scatter_plot);
    ROS_INFO("Started Point Cloud Viewer");
}

void PointCloudViewer_W::pointCloudCallback(const sensor_msgs::PointCloudConstPtr& in_cloud_ptr){
    curr_cloud = *in_cloud_ptr;
    ROS_INFO("Point Cloud Received");
}