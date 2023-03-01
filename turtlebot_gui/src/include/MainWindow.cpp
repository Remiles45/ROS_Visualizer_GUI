// #include <QtWidgets>
#include "MainWindow.h"
// #include <ros/ros.h>

MainWindow::MainWindow(ros::NodeHandle& nh, QWidget* parent) : 
    QMainWindow(parent),
    nh_(nh)// do I need to pass node handle into here?
{    
    /*
    ===========================================================
    Description: 
        Main window constructor. the : textEdit(new QTextEdit)
        passes ownership of the text edit to the main window 

        This constructor sets up the entire window and places 
        in all of the corresponding widgets.
    ===========================================================
    */
    // set the default size of the window to 25% of the available screen width 
    // and 50% of the available screen height
    QRect screenSize = displayScreen->availableGeometry(this);
    this->setFixedSize(QSize(screenSize.width()*0.25f,screenSize.height()*0.5f));

    setWindowTitle("Turtlebot Data Viewer");
    // pcl_viewer_widget;//(ros::NodeHandle& nh);
    // setCentralWidget(pcl_viewer_widget);
    createWidgets(nh);
}

void MainWindow::createWidgets(ros::NodeHandle& nh){
    // QDockWidget *dock = new QDockWidget(PointCloudViewer_W(nh));
    auto pcl_viewer_w = new PointCloudViewer_W(nh,this);
    setCentralWidget(pcl_viewer_w);
}
