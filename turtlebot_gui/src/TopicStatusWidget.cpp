/*=========================================
  copyright [2023] Rebecca Miles
 ==========================================*/
#include "include/TopicStatusWidget.h"

TopicStatus_W::TopicStatus_W(QWidget *parent) : QWidget(parent) {
    /*
    Description:
        Constructor, sets up layout and adds a place for status message
        to be shown
    Inputs: 
        QWidget *parent - parent widget (Main window)
    */
    hlayout = new QHBoxLayout(this);
    info_msg = new QLabel("");
    unsubscribe_btn = new QPushButton("Disconnect");

    curr_status_icon = new QLabel();
    curr_status_icon->setPixmap(disconnected_icon);

    setupWidget();
}

void TopicStatus_W::setupWidget() {
    /*
    Description:
        sets up the widget, handles adding items to the layout and setting
        any other settings that are desired.
    */
    hlayout->addWidget(curr_status_icon);
    hlayout->addWidget(info_msg);
    hlayout->addStretch();
    hlayout->addWidget(unsubscribe_btn);

    // set up connects
    connect(
        unsubscribe_btn,
        &QPushButton::released,
        this,
        &TopicStatus_W::handleUnsubButton);
}

void TopicStatus_W::handleUnsubButton() {
    /*
    Description:
        emits a signal to indicate that the user wants to unsubscribe from 
        the current topic.
    */
    emit unsubTrigger();
}

void TopicStatus_W::updateStatusMsg(msg_type& type) {
    /*
    Description:
        SLOT, sets the message to be displayed according to the message type
        (enum) that is indicated by the signal.
    Inputs: 
        msg_type& type - enum defined in StatusMsgTypes.h 
    */
    QString msg;
    switch (type) {
    case Connected:
        msg = "Topic Connected";
        curr_status_icon->setPixmap(connected_icon);
        break;
    case Disconnected:
        msg = "No Topics Connected";
        curr_status_icon->setPixmap(disconnected_icon);
        break;
    case Unresponsive:
        msg = "Displaying old data";
        curr_status_icon->setPixmap(unresponsive_icon);
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
