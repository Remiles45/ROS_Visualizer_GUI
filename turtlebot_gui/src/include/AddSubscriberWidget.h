/*=========================================
  copyright [2023] Rebecca Miles
===========================================*/
#pragma once

#include <QLineEdit>
#include <QString>
#include <QHBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <string>

class AddSubscriber_W : public QWidget{
    Q_OBJECT

 public:
    explicit AddSubscriber_W(QWidget* parent = nullptr);
 private:
    void setupWidget(void);
    void handleAddSubButton(void);

    QHBoxLayout *hlayout_m;
    QLineEdit *subscriber_name_entry_m;
    QPushButton *add_subscriber_btn_m;

 signals:
    void addSubSignal(std::string&);
};
