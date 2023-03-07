#pragma once
#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include <QtWidgets>
#include <vector>
#include <QTimer>
#include <QMessageBox>
#include "StatusMsgTypes.h"
// #include <set>

class RosTranslation : public QWidget
{
    Q_OBJECT

    public:
        explicit RosTranslation(ros::NodeHandle& nh, QWidget* parent = nullptr);
        ~RosTranslation();
    private:
        void updateScanData(const sensor_msgs::LaserScan&);
        void checkSubscribeFirstResponse(void);
        void checkSubscribeResponding(void);
        void reportStatus(void);

        // Subscriber Timed Out
        QMessageBox *sub_failed_popup_m;
        QPushButton *confirm_name_btn;
        QPushButton *delete_sub_btn;

        std::string sub_name;
        // Flags and watchdog timer to ensure subscriber functionality
        bool responding_flag_m = false;
        bool first_msg_received_flag_m = false;
        QTimer *subscriber_watchdog_m = nullptr;
        
        // amount of time for the watchdog to wait when the subscriber is first connected
        int new_sub_watchdog_time_interval_m = 5000; // mS (5 Seconds)
        // amount of time for the watchdog to wait before it determines the topic is no longer 
        // publishing.
        int watchdog_time_interval_m = 10000; // mS (30 Seconds)

        // timer to continuously report the status of the topic connection
        QTimer *status_update_timer_m;
        int status_update_interval_m = 250; // mS
        msg_type curr_status = Disconnected;

    signals:
        void laserScanValueChanged(std::vector<std::vector<float>>&);
        void subscriberStatus(msg_type&);
    public slots:
        void addSubscriber(std::string&);
    protected:
        // Ros related variables
        ros::NodeHandle nh_m;
        ros::Subscriber lscan_sub_m;
};

