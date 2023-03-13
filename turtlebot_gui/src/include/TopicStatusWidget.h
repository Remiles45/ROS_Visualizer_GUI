/*=========================================
  copyright [2023] Rebecca Miles
 ==========================================*/
#pragma once

#include <QLabel>
#include <QHBoxLayout>
#include <QPixmap>
#include <QPushButton>
#include "StatusMsgTypes.h"


class TopicStatus_W : public QWidget {
    Q_OBJECT

 public:
    explicit TopicStatus_W(QWidget* parent = nullptr);

 public slots:
    void updateStatusMsg(msg_type&);

 signals:
    void unsubTrigger();

 private:
    void setupWidget(void);
    void handleUnsubButton(void);
    QLabel *info_msg;
    QHBoxLayout *hlayout;
    QLabel *curr_status_icon;
    QPushButton *unsubscribe_btn;

    int icon_height = 15;

    QPixmap connected_icon = QPixmap(":/connected.png").scaledToHeight(icon_height);
    QPixmap unresponsive_icon = QPixmap(":/unresponsive.png").scaledToHeight(icon_height);
    QPixmap disconnected_icon = QPixmap(":/disconnected.png").scaledToHeight(icon_height);
};
