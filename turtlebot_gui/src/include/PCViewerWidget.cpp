#include "PCViewerWidget.h"
// #include <QtCharts/QScatterSeries>
#include <QtCharts>

PointCloudViewer_W::PointCloudViewer_W(ros::NodeHandle& nh,QWidget* parent) : 
    QWidget(parent) 
    {
    pcl_sub = nh.subscribe("/scan",3,&PointCloudViewer_W::pointCloudCallback, this);
    // QtDataVisualization::Q3DScatter *scatter_plot = new QtDataVisualization::Q3DScatter;
    QChart *scatter_plot = new QChart();
    QScatterSeries *pcl_data = new QScatterSeries(); //tmp
    pcl_data->setName("Point Cloud Data");
    pcl_data->setMarkerShape(QScatterSeries::MarkerShapeCircle);
    pcl_data->setMarkerSize(10.0);

    //temp
    pcl_data->append(0, 6);
    pcl_data->append(2, 4);
    pcl_data->append(3, 8);
    pcl_data->append(7, 4);
    pcl_data->append(10, 5);

    scatter_plot->addSeries(pcl_data);
    QChartView *viewer = new QChartView(scatter_plot);
    // QWidget *container = QWidget::createWindowContainer(scatter_plot);
    // QWidget *widget = new QWidget;
    QHBoxLayout *hLayout = new QHBoxLayout(this);
    hLayout->addWidget(viewer,1);

    ROS_INFO("Started Point Cloud Viewer");
    rosSpin();
}

void PointCloudViewer_W::pointCloudCallback(const sensor_msgs::PointCloudConstPtr& in_cloud_ptr){
    curr_cloud = *in_cloud_ptr;
    ROS_INFO("Point Cloud Received");
}

void PointCloudViewer_W::rosSpin(){
    ros::Rate r(500);
    while (ros::ok())
    {
        ros::spinOnce();                   // Handle ROS events
        r.sleep();
    }
}