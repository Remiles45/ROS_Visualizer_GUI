#include "AddSubscriberWidget.h"
#include <iostream>

AddSubscriber_W::AddSubscriber_W (QWidget* parent) : QWidget(parent){

    hLayout = new QHBoxLayout(this);
    hLayout->setAlignment(Qt::AlignLeft);
    setupWidget();

}

void AddSubscriber_W::setupWidget(){
    // create widgets
    QLabel *subscriber_name_label = new QLabel("Laser Scan Subscriber: ");
    subscriber_name_entry = new QLineEdit("base_scan");
    add_subscriber_btn = new QPushButton("Subscribe");

    // add widgets to the layout
    hLayout->addWidget(subscriber_name_label);
    hLayout->addWidget(subscriber_name_entry);
    hLayout->addWidget(add_subscriber_btn);
    // stretch will automatically fill the excess space in the layout 
    // with a spacer. 
    hLayout->addStretch();


    // set up connects
    connect(add_subscriber_btn, &QPushButton::released, this, &AddSubscriber_W::handleAddSubButton);

} 

void AddSubscriber_W::handleAddSubButton(){
    /*
    Description:
        Called when the subscriber button is clicked. Collects the text from the
        LineEdit and sends it as a signal.  
    */
    QString add_sub = subscriber_name_entry->text();
    std::string subscriber_name = add_sub.toStdString();
    emit addSubSignal(subscriber_name);
}