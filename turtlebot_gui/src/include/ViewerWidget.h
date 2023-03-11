/*=========================================
  copyright [2023] Rebecca Miles
 ==========================================*/
#pragma once

#include <ros/ros.h>
// #include <QtWidgets>
#include <QList>
#include <QtCharts>
#include <QTimer>
// #include <QString>
// #include <cmath>
#include <vector>
// #include "sensor_msgs/LaserScan.h"

class Viewer_W : public QWidget {
    Q_OBJECT

 public:
    explicit Viewer_W(QWidget* parent = nullptr);
    ~Viewer_W();

    void setMarkerSize(float);
    void setMarkerColor(void);

 public slots:
    void updateData(std::vector<std::vector<float>>&);

 private:
    void repaintScan(void);
    void updatePlotArea(void);
    void addDataToSeries(void);

    // Plot variables
    QChart *scatter_plot_m;
    QChartView *viewer;
    QTimer *repaintTimer;

    float marker_size_m = 5.0f;
    QScatterSeries *scan_data_m = nullptr;
    std::vector<std::vector<float>> raw_data_m;
};
