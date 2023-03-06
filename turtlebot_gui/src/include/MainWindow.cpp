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
    // set the default size of the window to 50% of the available screen width 
    // and 75% of the available screen height
    QRect screenSize = displayScreen->availableGeometry(this);
    this->setFixedSize(QSize(screenSize.width()*0.5f,screenSize.height()*0.75f));

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
    QGridLayout *main_layout = new QGridLayout();
    QWidget *main_widget = new QWidget();

    RosTranslation *ros_translation = new RosTranslation(nh_,this);
    Viewer_W *lscan_viewer_w = new Viewer_W(this);
    AddSubscriber_W *add_sub_w = new AddSubscriber_W(this);
    // QSpacerItem *spacer = new QSpacerItem(1,2);
    main_layout->addWidget(lscan_viewer_w,1,0);
    main_layout->addWidget(add_sub_w,0,0);
    main_widget->setLayout(main_layout);
    setCentralWidget(main_widget);

    // Connect signals and slots
    QObject::connect(ros_translation, SIGNAL(laserScanValueChanged(std::vector<std::vector<float>>&)), lscan_viewer_w, SLOT(updateData(std::vector<std::vector<float>>&)));
    QObject::connect(add_sub_w, SIGNAL(addSubSignal(std::string&)), ros_translation, SLOT(addSubscriber(std::string&)));
 
}
