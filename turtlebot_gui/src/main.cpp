/*=========================================
  copyright [2023] Rebecca Miles
 ==========================================*/

#include "include/main.h"

int main(int argc, char *argv[]) {
    // Initialize the ros node
    ros::init(argc, argv, "turtlebot_gui");
    ros::NodeHandle nh;

    // Create the Application and display the Gui window
    QApplication app(argc, argv);
    MainWindow turtle_gui(nh);

    turtle_gui.show();
    return app.exec();
}
