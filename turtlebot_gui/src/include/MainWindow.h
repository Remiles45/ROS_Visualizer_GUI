#pragma once

#include <QMainWindow>
#include <QDesktopWidget>
#include <ros/ros.h>
#include "PCViewerWidget.h"

class MainWindow : public QMainWindow{
    Q_OBJECT //enables use of signals and slots

    public:
        MainWindow(ros::NodeHandle& nh);

    private:
        // void createActions();
        // void createMenus();
        // void createDockWindows();
        
        int windowWidth;
        int windowHeight;
        QDesktopWidget *displayScreen;
        QMenu *fileMenu;

        PointCloudViewer_W pcl_viewer_widget(ros::NodeHandle& nh);
    protected: 
        ros::NodeHandle nh_;
};