#pragma once

#include <QLineEdit>
// #include "ViewerWidget.h"
#include <QString>
#include <QHBoxLayout>
#include <QPushButton>
#include <QLabel>

class AddSubscriber_W : public QWidget{
    Q_OBJECT

    public:
        explicit AddSubscriber_W(QWidget* parent = nullptr);
    private:
        void setupWidget(void);
        void handleAddSubButton(void);

        QHBoxLayout *hLayout;
        QLineEdit *subscriber_name_entry;
        QPushButton *add_subscriber_btn;

    signals:
        void addSubSignal(std::string&);
};