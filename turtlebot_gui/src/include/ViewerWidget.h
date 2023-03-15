/*=========================================
  copyright [2023] Rebecca Miles
 ==========================================*/
#pragma once

#include <ros/ros.h>
#include <QList>
#include <QtCharts>
#include <QTimer>
#include <QGridLayout>
#include <QPushButton>
#include <vector>

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
    void zoomIn(void);
    void zoomOut(void);
    void zoomHome(void);

    // Plot variables
    QGridLayout *layout_m;
    QChart *scatter_plot_m;
    QChartView *viewer_m;
    QTimer *repaint_timer_m;
    // Plot Settings/data
    float marker_size_m = 5.0f;
    QScatterSeries *scan_data_m = nullptr;
    std::vector<std::vector<float>> raw_data_m;

    // Buttons
    QPushButton *zoom_in_btn_m;
    QPushButton *zoom_out_btn_m;
    QPushButton *zoom_home_btn_m;

    // Button Visuals
    int button_height = 50;
    QPixmap zoom_in_pix_m = QPixmap(":/zoomIn.png").scaledToHeight(button_height);
    QPixmap zoom_out_pix_m = QPixmap(":/zoomOut.png").scaledToHeight(button_height);
    QPixmap zoom_home_pix_m = QPixmap(":/zoomHome.png").scaledToHeight(button_height);
};
