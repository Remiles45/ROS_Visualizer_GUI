#include "LScanViewerWidget.h"

LaserScanViewer_W::LaserScanViewer_W(ros::NodeHandle& nh,QWidget* parent) : 
    QWidget(parent),
    nh_(nh) 
    {
    /*
    Description:
        Constructor, sets up a subscriber to the turtlebot laserscan,
        creates a plot in the GUI which will display the collected laserscan, updating periodically.
    */
    // create the subscriber
    lscan_sub = nh_.subscribe("scan",3,&LaserScanViewer_W::LaserScanCallback,this);
    // create a plot and add it to the widget
    scatter_plot_m = new QChart();
    viewer = new QChartView(scatter_plot_m);
    QHBoxLayout *hLayout = new QHBoxLayout(this);
    hLayout->addWidget(viewer,1);
    
    // start a timer to repaint the plot every 250 mS
    repaintTimer = new QTimer(this);
    connect(repaintTimer, &QTimer::timeout, this, &LaserScanViewer_W::repaintScan);
    repaintTimer->start(250); 
}
LaserScanViewer_W::~LaserScanViewer_W(){
    /*
    Description:
        Destructor, shuts down timers.
    */
    repaintTimer->stop();
}

void LaserScanViewer_W::LaserScanCallback(const sensor_msgs::LaserScan& laser_scan_msg){
    /*
    Description:
        Callback for the lscan subscriber. Updates member veriables to most recent scan values
    */
    scan_data_vec_m = laser_scan_msg.ranges;
    angle_inc_m = laser_scan_msg.angle_increment;
    angle_min_m = laser_scan_msg.angle_min;
    angle_max_m = laser_scan_msg.angle_max;
    scan_range_min_m = laser_scan_msg.range_min;
    scan_range_max_m = laser_scan_msg.range_max;

    // repaintScan();
}

void LaserScanViewer_W::repaintScan(){
    /*
    Description: 
        takes the most recent laser scan and updates the plot 
        being displayed in the GUI
    */
    //clear the plot
    scatter_plot_m->removeAllSeries(); 

    // Create a scatter series in which we will store all of the datapoints that 
    // the laser scan has collected. 
    QScatterSeries *scan_data = new QScatterSeries(); 
    scan_data->setMarkerShape(QScatterSeries::MarkerShapeCircle);
    scan_data->setMarkerSize(10.0);

    // Convert data vector into x and y points to plot
    float angle = angle_min_m;
    for(int i=0; i<scan_data_vec_m.size(); i++){
        // throw out out of range data points
       if ((scan_data_vec_m[i]<scan_range_max_m) & (scan_data_vec_m[i]>scan_range_min_m)){
            float x,y;
            x = scan_data_vec_m[i] * std::cos(angle);
            y = scan_data_vec_m[i] * std::sin(angle);
            scan_data->append(x,y);
        }
        angle += angle_inc_m;
    }
    // add the data to the plot
    scatter_plot_m->addSeries(scan_data);
    // update plot size according to the data that was received
    // may want to make this smarter in the future
    scatter_plot_m->createDefaultAxes();
    viewer->update();//repaint();
}

// LaserScanViewer_W::