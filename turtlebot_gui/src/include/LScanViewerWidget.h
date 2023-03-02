#pragma once
#include <ros/ros.h>
#include <QtWidgets>
#include "sensor_msgs/LaserScan.h"
#include <QtCharts>
#include <cmath>
#include <QTimer>



class LaserScanViewer_W : public QWidget{
    Q_OBJECT

    public: 
        LaserScanViewer_W(ros::NodeHandle& nh, QWidget* parent = nullptr);
        ~LaserScanViewer_W();
    private:
        void LaserScanCallback(const sensor_msgs::LaserScan&);
        void repaintScan();
        void updatePlotArea();

        // Plot variables
        QChart *scatter_plot_m;
        QChartView *viewer;
    
        QTimer *repaintTimer;

        // Laser Scan data: updated in callback
        std::vector<float> scan_data_vec_m;
        float angle_inc_m;
        float angle_min_m;
        float angle_max_m;
        float scan_range_min_m;
        float scan_range_max_m;

    protected:
        // Ros related variables
        ros::NodeHandle nh_;
        ros::Subscriber lscan_sub;

};