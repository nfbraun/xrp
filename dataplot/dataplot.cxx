#include <QApplication>
#include <QStringList>
#include <QMainWindow>
#include <iostream>
#include "MainWindow.h"

int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    MainWindow w;
    
    if(argc > 1) {
        for(int c=1; c<argc; c++)
            w.loadFile(argv[c]);
        w.scaleToFit();
    }
    
    w.show();
    
    w.showMessages();
    
    return app.exec();
}
