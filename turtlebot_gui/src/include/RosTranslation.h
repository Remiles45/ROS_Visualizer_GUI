#pragma once
#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include <QtWidgets>
#include <vector>
#include <array>
// #include <set>

class RosTranslation : public QWidget
{
    Q_OBJECT

    private:
        void updateScanData(const sensor_msgs::LaserScan&);
    public:
        explicit RosTranslation(ros::NodeHandle& nh, QWidget* parent = nullptr);
        ~RosTranslation();

    signals:
        void laserScanValueChanged(std::vector<std::vector<float>>&);
    protected:
        // Ros related variables
        ros::NodeHandle nh_;
        ros::Subscriber lscan_sub;
};

