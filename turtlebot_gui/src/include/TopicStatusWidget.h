/*=========================================
  copyright [2023] Rebecca Miles
 ==========================================*/
#pragma once

#include <QLabel>
#include <QHBoxLayout>
#include <QPixmap>
#include <QPushButton>
#include <string>
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
    QLabel *info_msg_m;
    QHBoxLayout *hlayout_m;
    QLabel *curr_status_icon_m;
    QPushButton *unsubscribe_btn_m;

    int icon_height = 15;

    QPixmap connected_icon_m = QPixmap(":/connected.png").scaledToHeight(icon_height);
    QPixmap unresponsive_icon_m = QPixmap(":/unresponsive.png").scaledToHeight(icon_height);
    QPixmap disconnected_icon_m = QPixmap(":/disconnected.png").scaledToHeight(icon_height);
};
