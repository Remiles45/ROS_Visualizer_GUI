#pragma once

#include <QMainWindow>
#include <QDesktopWidget>
#include <ros/ros.h>
#include "PCViewerWidget.h"
#include <QDockWidget>

class MainWindow : public QMainWindow{
    Q_OBJECT //enables use of signals and slots

    public:
        MainWindow(ros::NodeHandle& nh, QWidget* parent = nullptr);

    private:
        //std::unique_ptr is a smart pointer that owns and manages
        //another object through a pointer and disposes of that 
        //object when the unique_ptr goes out of scope.
        // std::unique_ptr<MainWindow> ui; 
        // void createActions();
        // void createMenus();
        void createWidgets(ros::NodeHandle& nh);
        
        int windowWidth;
        int windowHeight;
        QDesktopWidget *displayScreen;
        QMenu *fileMenu;

        
    protected: 
        ros::NodeHandle nh_;
};