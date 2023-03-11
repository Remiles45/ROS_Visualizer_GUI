/*=========================================
  copyright [2023] Rebecca Miles
 ==========================================*/
#pragma once

#include <QLabel>
#include <QHBoxLayout>
#include <QPixmap>
#include "StatusMsgTypes.h"


class TopicStatus_W : public QWidget {
    Q_OBJECT

 public:
    explicit TopicStatus_W(QWidget* parent = nullptr);

 public slots:
    void updateStatusMsg(msg_type&);

 private:
    void setupWidget(void);
    QLabel *info_msg;
    QHBoxLayout *hlayout;
    QLabel *curr_status_icon;

    int icon_height = 15;

    QPixmap connected_icon = QPixmap(":/src/images/connected_icon.png").scaledToHeight(icon_height);
    QPixmap unresponsive_icon = QPixmap(":/src/images/unresponsive_icon.png").scaledToHeight(icon_height);
    QPixmap disconnected_icon = QPixmap(":/src/images/disconnected_icon.png").scaledToHeight(icon_height);
};
