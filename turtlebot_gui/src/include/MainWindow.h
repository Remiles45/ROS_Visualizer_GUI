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
    // std::unique_ptr is a smart pointer that owns and manages
    // another object through a pointer and disposes of that
    // object when the unique_ptr goes out of scope.
    // std::unique_ptr<MainWindow> ui;
    // void createActions();
    // void createMenus();
    void createWidgets();

    int windowWidth;
    int windowHeight;
    QDesktopWidget *displayScreen;
    QMenu *fileMenu;
    QTimer *spinTimer = nullptr;

 protected:
    ros::NodeHandle nh_;
};
