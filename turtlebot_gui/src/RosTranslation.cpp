/*=========================================
  copyright [2023] Rebecca Miles
 ==========================================*/
#include "include/RosTranslation.h"

RosTranslation::RosTranslation(ros::NodeHandle& nh, QWidget *parent) :
    nh_m(nh),
    QWidget(parent) {
    /**
     * DESCRIPTION:
     *   Constructor, creates basic widget elements that will be used.
     *   In this case that is a MessageBox which will inform the user
     *   if connecting to the chosen topic appears to have failed and
     *   a timer to continually report the status of the topic connection.
     * INPUTS:
     *   ros::NodeHandle& nh - the ros node
     *   QWidget *parent - the parent widget (main window)
    */
    // setup dialog box if subscriber setup fails.
    sub_failed_popup_m = new QMessageBox(this);
    confirm_name_btn = sub_failed_popup_m->addButton("Confirm Topic Name",
                                                    QMessageBox::YesRole);
    delete_sub_btn = sub_failed_popup_m->addButton("Delete Topic",
                                                 QMessageBox::DestructiveRole);

    // setup status reporting timer
    status_update_timer_m = new QTimer(this);
    connect(status_update_timer_m,
            &QTimer::timeout,
            this,
            &RosTranslation::reportStatus);
    status_update_timer_m->start(status_update_interval_m);
}

RosTranslation::~RosTranslation() {
    /**
     * DESCRIPTION:
     *   Destructor, shuts down timers.
    */
    if (subscriber_watchdog_m != nullptr) {subscriber_watchdog_m->stop();}
    if (status_update_timer_m != nullptr) {status_update_timer_m->stop();}
}

void RosTranslation::updateScanData(const sensor_msgs::LaserScan& laser_scan_msg) {
    /**
     * DESCRIPTION:
     *   convert the laser scan to xyz datapoints, then emit the data as a signal.
     * INPUTS:
     *   sensor_msgs::LaserScan& - incoming laser scan ros message
    */
    // confirm callback is being called for the watchdog / status reporter
    if (!first_msg_received_flag_m) {
        subscriber_watchdog_m->setInterval(watchdog_time_interval_m);
        first_msg_received_flag_m = true;
    }
    responding_flag_m = true;
    curr_status = Connected;

    // collect incoming data
    std::vector<float> scan_data_vec = laser_scan_msg.ranges;  // m
    angle_inc_m = laser_scan_msg.angle_increment;
    angle_min_m = laser_scan_msg.angle_min;
    angle_max_m = laser_scan_msg.angle_max;
    scan_range_min_m = laser_scan_msg.range_min;  // m
    scan_range_max_m = laser_scan_msg.range_max;  // m

    // create a 3xn output vector
    std::vector<std::vector<float>> out_scan;
    for (int i = 0; i < 3; i++) {
        std::vector<float> v;
        out_scan.push_back(v);
        }

    float angle = angle_min_m;
    float x, y, z;

    for (int i = 0; i < scan_data_vec.size(); i++) {
        if (checkValidPt(scan_data_vec[i], angle)) {
            // convert range to xy datapoint
            x = scan_data_vec[i] * std::cos(angle);
            y = scan_data_vec[i] * std::sin(angle);
            z = 0;
            // add data to output vector
            out_scan[0].push_back(x);
            out_scan[1].push_back(y);
            out_scan[2].push_back(z);
        }
        angle += angle_inc_m;
    }

    emit laserScanValueChanged(out_scan);
}

bool RosTranslation::checkValidPt(float pt, float angle) {
    /**
     * DESCRIPTION:
     *  checks if a datapoint is valid
     * INPUTS:
     *  float pt - range value to check
     *  float angle - angle value to check 
     * OUTPUTS:
     *  bool - if the given datapoint satisfies all validity conditions
    */
    bool valid_float = !isnan(pt) & !isinf(pt);
    bool valid_range = (pt < scan_range_max_m) & (pt > scan_range_min_m);
    bool valid_angle = (angle < angle_max_m) & (angle > angle_min_m);
    return valid_float & valid_range & valid_angle;
}

void RosTranslation::checkSubscribeResponding() {
    /**
     * DESCRIPTION:
     *   Check if subscriber is sending data.
     *   If no data has ever been received, check if subscriber name
     *   is correct, if not, destroy the subscriber.
     *   Handles warnings and info messages for watchdog timer timeout
     *   with no received data.
    */

    if (first_msg_received_flag_m) {
        curr_status = responding_flag_m ? Connected : Unresponsive;
    } else {
        // throw message box error
        QString msg = QString("Subscriber not responding\nIs '/%1' correct?")
                            .arg(QString::fromStdString(sub_name));
        sub_failed_popup_m->setInformativeText(msg);
        sub_failed_popup_m->exec();

        if (sub_failed_popup_m->clickedButton() == confirm_name_btn) {
            // allow the subscriber to continue even though no data has been received.
            first_msg_received_flag_m = true;
            curr_status = Unresponsive;
        } else if (sub_failed_popup_m->clickedButton() == delete_sub_btn) {
            // shut down the subscriber and delete the timer.
            unsubscribe();
        } else {
            // This would be the case if the user clicked the x button without
            // confirming either disconnect or continue buttons. Assume topic is
            // unresponsive
            curr_status = Unresponsive;
        }
    }

    responding_flag_m = false;
}

void RosTranslation::unsubscribe() {
    /**
     * DESCRIPTION:
     *   shuts down and deletes the subscriber,
     *   shuts off the watchdog timer
    */
    lscan_sub_m.shutdown();
    subscriber_watchdog_m->stop();
    subscriber_watchdog_m = nullptr;
    curr_status = Disconnected;
}

void RosTranslation::reportStatus() {
    /**
     * DESCRIPTION:
     *   reports the status of the topic connection
    */
    emit subscriberStatus(curr_status);
}

void RosTranslation::triggerUnsubscribe() {
    /**
     * DESCRIPTION:
     *   SLOT, unsubscribes from the current topic.
    */
    unsubscribe();
}

void RosTranslation::addSubscriber(std::string& sub) {
    /**
     * DESCRIPTION:
     *   SLOT, create a subscriber and set up the watchdog timer.
     * INPUTS:
     *   std::string& sub - name of the topic to subscriber to
    */
    sub_name = sub;
    lscan_sub_m = nh_m.subscribe(sub_name, 3, &RosTranslation::updateScanData, this);
    if (subscriber_watchdog_m == nullptr) {
        // initialize watchdog if not already created
        subscriber_watchdog_m = new QTimer(this);
        connect(subscriber_watchdog_m,
                &QTimer::timeout,
                this,
                &RosTranslation::checkSubscribeResponding);
        subscriber_watchdog_m->start(new_sub_watchdog_time_interval_m);
    } else {
        // watchdog timeout 5 seconds after a new subscriber is added to ensure
        // the subscriber exists/ is the correct type and is responding.
        subscriber_watchdog_m->setInterval(new_sub_watchdog_time_interval_m);
    }

    first_msg_received_flag_m = false;
    responding_flag_m = false;
    curr_status = TryingToConnect;
}
