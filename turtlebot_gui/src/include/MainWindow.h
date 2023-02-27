#pragma once

#include <QMainWindow>
#include <QDesktopWidget>
#include <Q3DScatter>
#include <ros/ros.h>

class MainWindow : public QMainWindow{
    Q_OBJECT //enables use of signals and slots

    public:
        MainWindow(ros::NodeHandle& nh);

    private slots: //QT slots (functions that are connected to)


    private:
        // void createActions();
        // void createMenus();
        // void createDockWindows();
        
        int windowWidth;
        int windowHeight;
        QDesktopWidget *displayScreen;
        QMenu *fileMenu;

};