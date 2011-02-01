#ifndef VC_MAINWINDOW_H
#define VC_MAINWINDOW_H

#include "VCWidget.h"
#include "ChannelWidget.h"
#include "File.h"
#include <QString>
#include <QMainWindow>

class MainWindow: public QMainWindow
{
  Q_OBJECT
  
  public:
    MainWindow(QWidget* parent = 0);
    ~MainWindow();
    
    bool loadFile(const QString& fname);
    void showMessages();
    void scaleToFit() { fVCWidget->scaleToFit(); }
    
  public slots:
    void open();
    void reloadAll();
    void closeFile(QObject* file);
    void closeAllFiles();
    
  private:
    void updateDataMenu();
    
    VCWidget* fVCWidget;
    ChannelWidget* fChannelWidget;
    
    QMenu* fDataMenu;
    QSignalMapper* fDataMapper;
    std::list<File*> fFiles;
    
    bool fErrorFail;
    QString fErrorStr;
};

#endif
