#include "MainWindow.h"

MainWindow::MainWindow(ros::NodeHandle& nh, QWidget* parent) : 
    QMainWindow(parent),
    nh_(nh)
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

    spinTimer = new QTimer(this);
    connect(spinTimer, &QTimer::timeout, this, &ros::spinOnce);
    spinTimer->start(1); // add msec interval into start declaration if desired. with 0 interval it will call as soon as all events in the window systems event queue have been processed.
    createWidgets();
}

MainWindow::~MainWindow(){
    spinTimer->stop();
}

void MainWindow::createWidgets(){
    QHBoxLayout *main_layout = new QHBoxLayout();
    QWidget *main_widget = new QWidget();

    RosTranslation *ros_translation = new RosTranslation(nh_,this);
    Viewer_W *lscan_viewer_w = new Viewer_W(this);

    main_layout->addWidget(lscan_viewer_w);
    // main_layout->addWidget(options_w);
    main_widget->setLayout(main_layout);
    setCentralWidget(main_widget);


    QObject::connect(ros_translation, SIGNAL(laserScanValueChanged(std::vector<std::vector<float>>&)), lscan_viewer_w, SLOT(updateData(std::vector<std::vector<float>>&)));
 
}
