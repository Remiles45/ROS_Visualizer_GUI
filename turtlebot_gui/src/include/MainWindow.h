/*=========================================
  copyright [2023] Rebecca Miles
 ==========================================*/
#pragma once

#include <QMainWindow>
#include <QDesktopWidget>
#include <ros/ros.h>
#include <QTimer>
#include <vector>
#include <string>
#include "ViewerWidget.h"
#include "AddSubscriberWidget.h"
#include "RosTranslation.h"
#include "TopicStatusWidget.h"

class MainWindow : public QMainWindow {
    // enable use of signals and slots
    Q_OBJECT

 public:
    explicit MainWindow(ros::NodeHandle&, QWidget* parent = nullptr);
    ~MainWindow();

 private:
    void createWidgets();
    // window sizing
    int windowWidth;
    int windowHeight;
    QDesktopWidget *displayScreen;
    // timer that spins ros
    QTimer *spinTimer = nullptr;

 protected:
    ros::NodeHandle nh_;
};
