#pragma once

#include <QMainWindow>
#include <QDesktopWidget>
#include <ros/ros.h>
#include "ViewerWidget.h"
#include "AddSubscriberWidget.h"
#include "RosTranslation.h"
#include "TopicStatusWidget.h"
#include <QTimer>

class MainWindow : public QMainWindow{
    Q_OBJECT //enables use of signals and slots

    public:
        MainWindow(ros::NodeHandle& nh, QWidget* parent = nullptr);
        ~MainWindow();

    private:
        //std::unique_ptr is a smart pointer that owns and manages
        //another object through a pointer and disposes of that 
        //object when the unique_ptr goes out of scope.
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