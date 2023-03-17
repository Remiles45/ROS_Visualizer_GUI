/*=========================================
  copyright [2023] Rebecca Miles
 ==========================================*/
#include "include/AddSubscriberWidget.h"

AddSubscriber_W::AddSubscriber_W(QWidget* parent) : QWidget(parent) {
    /**
     * DESCRIPTION:
     *   Constructor for add subscriber widget. Will consist of a 
     *   QLineEdit and submit button
    */
    setupWidget();
}

void AddSubscriber_W::setupWidget() {
    /**
     * DESCRIPTION:
     *   Set up the widget. Add sub-widgets, place in layout and connect signals
    */
    // create layout
    hLayout = new QHBoxLayout(this);
    hLayout->setAlignment(Qt::AlignLeft);

    // create widgets
    QLabel *subscriber_name_label = new QLabel("Laser Scan Topic: ");
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
    connect(
        add_subscriber_btn,
        &QPushButton::released,
        this,
        &AddSubscriber_W::handleAddSubButton);
}

void AddSubscriber_W::handleAddSubButton() {
    /**
     * DESCRIPTION:
     *   Called when the subscriber button is clicked. Collects the text from the
     *   LineEdit and sends it as a signal.  
    */
    QString add_sub = subscriber_name_entry->text();
    std::string subscriber_name = add_sub.toStdString();
    emit addSubSignal(subscriber_name);
}
