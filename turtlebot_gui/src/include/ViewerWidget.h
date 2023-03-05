#pragma once
#include <ros/ros.h>
#include <QtWidgets>
#include "sensor_msgs/LaserScan.h"
#include <QtCharts>
#include <cmath>
#include <QTimer>

#include <iostream>

class Viewer_W : public QWidget{
    Q_OBJECT

    public: 
        explicit Viewer_W(QWidget* parent = nullptr);
        ~Viewer_W();

        void setMarkerSize(float);
        void setMarkerColor();

    public slots:
        void updateData(std::vector<std::vector<float>>&);

    private:
        // void LaserScanCallback(const sensor_msgs::LaserScan&);
        void repaintScan();
        void updatePlotArea();
        void addDataToSeries();

        // Plot variables
        QChart *scatter_plot_m;
        QChartView *viewer;
        QTimer *repaintTimer;

        float marker_size_m=5.0f;
        QScatterSeries *scan_data_m;
        std::vector<std::vector<float>> raw_data_m;
        // Laser Scan data: updated in callback


    // protected:
    //     // Ros related variables
    //     ros::NodeHandle nh_;
    //     ros::Subscriber lscan_sub;

};