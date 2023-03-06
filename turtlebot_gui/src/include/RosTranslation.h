#pragma once
#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include <QtWidgets>
#include <vector>
#include <QTimer>
#include <QMessageBox>
// #include <set>

class RosTranslation : public QWidget
{
    Q_OBJECT

    private:
        void updateScanData(const sensor_msgs::LaserScan&);
        void checkSubscribeFirstResponse(void);
        void checkSubscribeResponding(void);

        // Subscriber Timed Out
        QMessageBox *sub_failed_popup_m;
        QPushButton *confirm_name_btn;
        QPushButton *delete_sub_btn;

        std::string sub_name;
        // Flags and watchdog timer to ensure subscriber functionality
        bool responding_flag_m = false;
        bool first_msg_received_flag_m = false;
        QTimer *subscriber_watchdog_m = nullptr;

        int new_sub_watchdog_time_interval_m = 5000; // mS (5 Seconds)
        int watchdog_time_interval = 30000; // mS (30 Seconds)
    public:
        explicit RosTranslation(ros::NodeHandle& nh, QWidget* parent = nullptr);
        ~RosTranslation();

    signals:
        void laserScanValueChanged(std::vector<std::vector<float>>&);
    public slots:
        void addSubscriber(std::string&);
    protected:
        // Ros related variables
        ros::NodeHandle nh_m;
        ros::Subscriber lscan_sub_m;
};

