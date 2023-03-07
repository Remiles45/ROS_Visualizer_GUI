#include "RosTranslation.h"
#include <iostream>

RosTranslation::RosTranslation(ros::NodeHandle& nh, QWidget *parent) : 
    nh_m(nh) ,
    QWidget(parent)
{
    /*
    Description:
        Constructor, creates basic widget elements that will be used. 
        In this case that is a MessageBox which will inform the user 
        if connecting to the chosen topic appears to have failed and
        a timer to continually report the status of the topic connection.
    Inputs:
        ros::NodeHandle& nh - the ros node
        QWidget *parent - the parent widget (main window)
    */
    // setup dialog box if subscriber setup fails.
    sub_failed_popup_m = new QMessageBox(this);
    confirm_name_btn = sub_failed_popup_m->addButton("Confirm Topic Name",QMessageBox::YesRole); 
    delete_sub_btn   = sub_failed_popup_m->addButton("Delete Topic",QMessageBox::DestructiveRole);

    // setup status reporting timer
    status_update_timer_m = new QTimer(this);
    connect(status_update_timer_m, &QTimer::timeout, this, &RosTranslation::reportStatus);
    status_update_timer_m->start(status_update_interval_m);
}

RosTranslation::~RosTranslation()
{
    /*
    Description:
        Destructor, shuts down timers.
    */
   if (subscriber_watchdog_m != nullptr){subscriber_watchdog_m->stop();}
   if (status_update_timer_m != nullptr){status_update_timer_m->stop();}
}

void RosTranslation::updateScanData(const sensor_msgs::LaserScan& laser_scan_msg){
    /*
    Description:
        convert the laser scan to xyz datapoints, then emit the data as a signal.
    Inputs:
        sensor_msgs::LaserScan& - incoming laser scan ros message
    */
    // confirm callback is being called for the watchdog / status reporter
    if (!first_msg_received_flag_m){
        subscriber_watchdog_m->setInterval(watchdog_time_interval_m);
        first_msg_received_flag_m = true;
    }
    responding_flag_m = true;
    curr_status = Connected;

    // collect incoming data
    std::vector<float> scan_data_vec = laser_scan_msg.ranges;
    float angle_inc = laser_scan_msg.angle_increment;
    float angle_min = laser_scan_msg.angle_min;
    float angle_max = laser_scan_msg.angle_max;
    float scan_range_min = laser_scan_msg.range_min;
    float scan_range_max = laser_scan_msg.range_max;

    // create a 3xn output vector 
    std::vector<std::vector<float>> out_scan;
    for(int i=0; i<3; i++){
        std::vector<float> v; 
        out_scan.push_back(v);
        }
    

    float angle = angle_min;
    float x,y,z;
    for(int i=0; i<scan_data_vec.size(); i++){
        // throw out out of range data points
        if ((scan_data_vec[i]<angle_max) & (scan_data_vec[i]>angle_min)){
            // convert range to xy datapoint
            x = scan_data_vec[i] * std::cos(angle);
            y = scan_data_vec[i] * std::sin(angle);
            z = 0;
            // add data to output vector
            out_scan[0].push_back(x);
            out_scan[1].push_back(y);
            out_scan[2].push_back(z);
        }
        angle += angle_inc;
    }
    emit laserScanValueChanged(out_scan);
}

void RosTranslation::addSubscriber(std::string& sub){
    /*
    Description: 
        create a subscriber and set up the watchdog timer.
    Inputs:
        std::string& sub - name of the topic to subscriber to
    */
    sub_name = sub;
    lscan_sub_m = nh_m.subscribe(sub_name,3,&RosTranslation::updateScanData,this);
    if (subscriber_watchdog_m == nullptr){
        // initialize watchdog if not already created
        subscriber_watchdog_m = new QTimer(this);
        connect(subscriber_watchdog_m, &QTimer::timeout, this, &RosTranslation::checkSubscribeResponding);
        subscriber_watchdog_m->start(new_sub_watchdog_time_interval_m);
    }
    else{
        // watchdog timeout 5 seconds after a new subscriber is added to ensure 
        // the subscriber exists/ is the correct type and is responding.
        subscriber_watchdog_m->setInterval(new_sub_watchdog_time_interval_m); 
    }

    first_msg_received_flag_m = false;
    responding_flag_m = false;
    curr_status = TryingToConnect;
}

void RosTranslation::checkSubscribeResponding(){
    /*
    Description:
        Check if subscriber is sending data. 
        If no data has ever been received, check if subscriber name
        is correct, if not, destroy the subscriber. 
        Handles warnings and info messages for watchdog timer timeout
        with no received data. 
    */

    if (first_msg_received_flag_m){
        curr_status = responding_flag_m ? Connected : Unresponsive; 
    }else{
        // throw message box error
        QString msg = QString("Subscriber not responding\nIs '/%1' correct?").arg(QString::fromStdString(sub_name));
        sub_failed_popup_m->setInformativeText(msg);
        sub_failed_popup_m->exec();

        if (sub_failed_popup_m->clickedButton() == confirm_name_btn){
            // allow the subscriber to continue even though no data has been received. 
            first_msg_received_flag_m = true;
            curr_status = Unresponsive;
        }else if (sub_failed_popup_m->clickedButton() == delete_sub_btn){
            // shut down the subscriber and delete the timer.
            lscan_sub_m.shutdown();
            subscriber_watchdog_m->stop();
            subscriber_watchdog_m = nullptr;
            curr_status = Disconnected;
        }else{
            // This would be the case if the user clicked the x button without 
            // confirming either disconnect or continue buttons. Assume topic is 
            // unresponsive
            curr_status = Unresponsive;
        }
    }
    
    // emit subscriberStatus(responding_flag_m);
    responding_flag_m = false;
}

void RosTranslation::reportStatus(){
    /*
    Description:
        reports the status of the topic connection
    */
    emit subscriberStatus(curr_status);
}