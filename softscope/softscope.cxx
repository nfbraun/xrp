#include <QApplication>
#include <QStringList>
#include <iostream>
#include "ScopeWindow.h"

int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    const char* sourcename = "softscope.fifo";
    if(argc > 1)
        sourcename = argv[1];

    ScopeWindow w(sourcename);
    w.show();
    
    return app.exec();
}
