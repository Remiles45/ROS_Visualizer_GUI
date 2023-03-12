/*=========================================
  copyright [2023] Rebecca Miles
 ==========================================*/
#include "include/MainWindow.h"

MainWindow::MainWindow(ros::NodeHandle& nh, QWidget* parent) :
    QMainWindow(parent),
    nh_(nh) {
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
    this->setFixedSize(QSize(screenSize.width()*0.5f,
                             screenSize.height()*0.75f));

    setWindowTitle("Turtlebot Data Viewer");

    // Start a timer for ros spin
    spinTimer = new QTimer(this);
    connect(spinTimer, &QTimer::timeout, this, &ros::spinOnce);
    spinTimer->start(1);

    createWidgets();

    ROS_INFO("Main Window Setup Succeeded");
}

MainWindow::~MainWindow() {
    /*
    Description:
        Destructor, shuts down timers
    */
    spinTimer->stop();
}

void MainWindow::createWidgets() {
    /*
    Description:
        set up main window, create sub widgets, set up layout, connect signals.
    */
    QGridLayout *main_layout = new QGridLayout();
    QWidget *main_widget = new QWidget();

    // create widget objects.
    RosTranslation *ros_translation = new RosTranslation(nh_, this);
    Viewer_W *lscan_viewer_w = new Viewer_W(this);
    AddSubscriber_W *add_sub_w = new AddSubscriber_W(this);
    TopicStatus_W *topic_status_w = new TopicStatus_W(this);

    // add widgets to the layout
    main_layout->addWidget(add_sub_w, 0, 0);
    main_layout->addWidget(topic_status_w, 1, 0);
    main_layout->addWidget(lscan_viewer_w, 2, 0);
    main_widget->setLayout(main_layout);
    setCentralWidget(main_widget);

    // Connect signals and slots
    QObject::connect(ros_translation,
                SIGNAL(laserScanValueChanged(std::vector<std::vector<float>>&)),
                lscan_viewer_w,
                SLOT(updateData(std::vector<std::vector<float>>&)));
    QObject::connect(add_sub_w,
                SIGNAL(addSubSignal(std::string&)),
                ros_translation,
                SLOT(addSubscriber(std::string&)));
    QObject::connect(ros_translation,
                SIGNAL(subscriberStatus(msg_type&)),
                topic_status_w,
                SLOT(updateStatusMsg(msg_type&)));
    QObject::connect(topic_status_w,
                SIGNAL(unsubTrigger()),
                ros_translation,
                SLOT(triggerUnsubscribe()));
}
