/*=========================================
  copyright [2023] Rebecca Miles
 ==========================================*/

#include "include/main.h"

int main(int argc, char *argv[]) {
/**
 * DESCRIPTION: 
 *  sets up the application and displays the GUI window
*/
    // Initialize the ros node
    ros::init(argc, argv, "turtlebot_gui");
    ros::NodeHandle nh;

    // Create the Application and display the Gui window
    QApplication app(argc, argv);
    MainWindow turtle_gui(nh);

    app.setWindowIcon(QIcon(":/window.png"));

    turtle_gui.show();
    return app.exec();
}
