#include "include/MainWindow.h"
#include <QApplication>
#include "ros/ros.h"

int main(int argc, char *argv[])
{   
    // Initialize the ros node
    ros::init(argc, argv, "listener");
    ros::NodeHandle nh;

    // Create the Application and display the Gui window
    QApplication app(argc, argv);
    MainWindow turtle_gui(nh);

    turtle_gui.show();
    ros::spinOnce();
    return app.exec();
}