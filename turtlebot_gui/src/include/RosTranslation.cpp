#include "RosTranslation.h"
#include <iostream>

RosTranslation::RosTranslation(ros::NodeHandle& nh, QWidget *parent) : 
    nh_m(nh) ,
    QWidget(parent)
{
    // setup dialog box if subscriber setup fails.
    sub_failed_popup_m = new QMessageBox(this);
    confirm_name_btn = sub_failed_popup_m->addButton("Confirm Topic Name",QMessageBox::YesRole); 
    delete_sub_btn   = sub_failed_popup_m->addButton("Delete Topic",QMessageBox::DestructiveRole);
}

RosTranslation::~RosTranslation()
{
    /*
    Description:
        Destructor, shuts down timers.
    */
   if (subscriber_watchdog_m != nullptr){subscriber_watchdog_m->stop();}
}

void RosTranslation::updateScanData(const sensor_msgs::LaserScan& laser_scan_msg){
    /*
    Description:
        convert the laser scan to xyz datapoints, then emit the data as a signal.
    */
    // confirm callback is being called for the watchdog
    if (!first_msg_received_flag_m){subscriber_watchdog_m->setInterval(watchdog_time_interval);}
    responding_flag_m = true;
    first_msg_received_flag_m = true;

    std::vector<float> scan_data_vec = laser_scan_msg.ranges;
    float angle_inc = laser_scan_msg.angle_increment;
    float angle_min = laser_scan_msg.angle_min;
    float angle_max = laser_scan_msg.angle_max;
    float scan_range_min = laser_scan_msg.range_min;
    float scan_range_max = laser_scan_msg.range_max;

    std::vector<std::vector<float>> out_scan;
    for(int i=0; i<3; i++){std::vector<float> v; 
        out_scan.push_back(v);}

    float angle = angle_min;
    float x,y,z;
    for(int i=0; i<scan_data_vec.size(); i++){
        if ((scan_data_vec[i]<angle_max) & (scan_data_vec[i]>angle_min)){
            // throw out out of range data points
            x = scan_data_vec[i] * std::cos(angle);
            y = scan_data_vec[i] * std::sin(angle);
            z = 0;
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
    */
    sub_name = sub;
    lscan_sub_m = nh_m.subscribe(sub_name,3,&RosTranslation::updateScanData,this);
    if (subscriber_watchdog_m == nullptr){
        // initialize watchdog if not already created
        subscriber_watchdog_m = new QTimer(this);
        subscriber_watchdog_m->start(new_sub_watchdog_time_interval_m);
        connect(subscriber_watchdog_m, &QTimer::timeout, this, &RosTranslation::checkSubscribeResponding);
    }
    else{
        // watchdog timeout 5 seconds after a new subscriber is added to ensure 
        // the subscriber exists/ is the correct type and is responding.
        subscriber_watchdog_m->setInterval(new_sub_watchdog_time_interval_m); 
    }

    first_msg_received_flag_m = false;
    responding_flag_m = false;
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
        if (!responding_flag_m){
            //throw error
            std::cout << "No New Data Received\n";
            //TODO: make this text on the screen
        }
    }
    else{
        // throw message box error
        QString msg = QString("Subscriber not responding\nIs '/%1' correct?").arg(QString::fromStdString(sub_name));
        sub_failed_popup_m->setInformativeText(msg);
        sub_failed_popup_m->exec();

        if (sub_failed_popup_m->clickedButton() == confirm_name_btn){
            first_msg_received_flag_m = true;
        }else if (sub_failed_popup_m->clickedButton() == delete_sub_btn){
            lscan_sub_m.shutdown();
            subscriber_watchdog_m->stop();
            subscriber_watchdog_m = nullptr;
        }else{
            std::cout << "something went wrong\n";
            //TODO: make this throw a real error
        }
    }
    responding_flag_m = false;
}
