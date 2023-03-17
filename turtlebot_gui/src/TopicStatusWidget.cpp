/*=========================================
  copyright [2023] Rebecca Miles
 ==========================================*/
#include "include/TopicStatusWidget.h"

TopicStatus_W::TopicStatus_W(QWidget *parent) : QWidget(parent) {
    /**
     * DESCRIPTION:
     *   Constructor, sets up layout and adds a place for status message
     *   to be shown
     * INPUTS: 
     *   QWidget *parent - parent widget (Main window)
    */
    hlayout_m = new QHBoxLayout(this);
    info_msg_m = new QLabel("");
    unsubscribe_btn_m = new QPushButton("Disconnect");

    curr_status_icon_m = new QLabel();
    curr_status_icon_m->setPixmap(disconnected_icon_m);

    setupWidget();
}

void TopicStatus_W::setupWidget() {
    /**
     * DESCRIPTION:
     *   sets up the widget, handles adding items to the layout and setting
     *   any other settings that are desired.
    */
    hlayout_m->addWidget(curr_status_icon_m);
    hlayout_m->addWidget(info_msg_m);
    hlayout_m->addStretch();
    hlayout_m->addWidget(unsubscribe_btn_m);

    // set up connects
    connect(
        unsubscribe_btn_m,
        &QPushButton::released,
        this,
        &TopicStatus_W::handleUnsubButton);
}

void TopicStatus_W::handleUnsubButton() {
    /**
     * DESCRIPTION:
     *   emits a signal to indicate that the user wants to unsubscribe from 
     *   the current topic.
    */
    emit unsubTrigger();
}

void TopicStatus_W::updateStatusMsg(msg_type& type) {
    /**
     * DESCRIPTION:
     *   SLOT, sets the message to be displayed according to the message type
     *   (enum) that is indicated by the signal.
     * INPUTS: 
     *   msg_type& type - enum defined in StatusMsgTypes.h 
    */
    QString msg;
    switch (type) {
    case Connected:
        msg = "Topic Connected";
        curr_status_icon_m->setPixmap(connected_icon_m);
        break;
    case Disconnected:
        msg = "No Topics Connected";
        curr_status_icon_m->setPixmap(disconnected_icon_m);
        break;
    case Unresponsive:
        msg = "Displaying old data";
        curr_status_icon_m->setPixmap(unresponsive_icon_m);
        break;
    case TryingToConnect:
        msg = "Connecting to topic ....";
        break;
    default:
        msg = "";
        break;
    }
    info_msg_m->setText(msg);
}
