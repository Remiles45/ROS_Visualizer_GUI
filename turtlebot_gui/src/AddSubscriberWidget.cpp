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
    hlayout_m = new QHBoxLayout(this);
    hlayout_m->setAlignment(Qt::AlignLeft);

    // create widgets
    QLabel *subscriber_name_label = new QLabel("Laser Scan Topic: ", this);
    subscriber_name_entry_m = new QLineEdit("base_scan", this);
    add_subscriber_btn_m = new QPushButton("Subscribe", this);

    // add widgets to the layout
    hlayout_m->addWidget(subscriber_name_label);
    hlayout_m->addWidget(subscriber_name_entry_m);
    hlayout_m->addWidget(add_subscriber_btn_m);
    // stretch will automatically fill the excess space in the layout
    // with a spacer.
    hlayout_m->addStretch();

    // set up connects
    connect(
        add_subscriber_btn_m,
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
    QString add_sub = subscriber_name_entry_m->text();
    std::string subscriber_name = add_sub.toStdString();
    emit addSubSignal(subscriber_name);
}
