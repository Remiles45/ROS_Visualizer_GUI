#pragma once
#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include <QtWidgets>
#include <vector>
#include <QTimer>
// #include <set>

class RosTranslation : public QWidget
{
    Q_OBJECT

    private:
        void updateScanData(const sensor_msgs::LaserScan&);
        bool subscriber_reached_m;
        QTimer *subscriber_watchdog_m;
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

