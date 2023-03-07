#pragma once

#include <QLabel>
#include <QHBoxLayout>
#include "StatusMsgTypes.h"


class TopicStatus_W : public QWidget{
    Q_OBJECT

    public:
        explicit TopicStatus_W(QWidget* parent = nullptr);
    public slots:
        void updateStatusMsg(msg_type&);
    private: 
        void setupWidget(void);
        QLabel *info_msg;
        QHBoxLayout *hlayout;

};