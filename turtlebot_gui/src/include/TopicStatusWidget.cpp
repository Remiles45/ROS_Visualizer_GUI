#include "TopicStatusWidget.h"

TopicStatus_W::TopicStatus_W(QWidget *parent) : QWidget(parent){
    hlayout = new QHBoxLayout(this);
    info_msg = new QLabel("");
    setupWidget();
}

void TopicStatus_W::setupWidget(){
    hlayout->setSpacing(0);
    hlayout->setMargin(0);

    info_msg->setAlignment(Qt::AlignLeft);

    hlayout->addWidget(info_msg);
    hlayout->addStretch();
}

void TopicStatus_W::updateStatusMsg(msg_type& type){
    // QString msg = status? "Topic Connected" : "No new data received";
    QString msg;
    switch (type)
    {
    case Connected:
        msg = "Topic Connected";
        break;
    case Disconnected:
        msg = "No Topics Connected";
        break;
    case Unresponsive:
        msg = "Displaying old data";
        break;
    case TryingToConnect:
        msg = "Connecting to topic....";
        break;
    default:
        msg = "";
        break;
    }
    info_msg->setText(msg);
}