#pragma once

#include <QMainWindow>
#include <QDesktopWidget>

class MainWindow : public QMainWindow{
    Q_OBJECT //enables use of signals and slots

    public:
        MainWindow();

    private slots: //QT slots (functions that are connected to)


    private:
        // void createActions();
        // void createMenus();
        // void createDockWindows();
        
        int windowWidth;
        int windowHeight;
        QDesktopWidget *displayScreen;
        QMenu *fileMenu;

};