#include "MainWindow.h"
#include "File.h"
#include <QMenuBar>
#include <QMenu>
#include <QAction>
#include <QDockWidget>
#include <QFileDialog>
#include <QMessageBox>

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent), fErrorFail(false)
{
    setWindowTitle(tr("Data plotter"));
    
    fVCWidget = new VCWidget(this);
    setCentralWidget(fVCWidget);
    
    fChannelWidget = new ChannelWidget(fVCWidget, this);
    QDockWidget* dock = new QDockWidget(tr("Channel properties"));
    dock->setWidget(fChannelWidget);
    addDockWidget(Qt::TopDockWidgetArea, dock);
    
    /*** FILE ***/
    QMenu* fileMenu = menuBar()->addMenu(tr("&File"));
    QAction* openAct = new QAction("&Open...", this);
    openAct->setShortcut(QString("Ctrl+O"));
    connect(openAct, SIGNAL(triggered()), this, SLOT(open()));
    fileMenu->addAction(openAct);
    
    QAction* reloadAct = new QAction("&Reload all", this);
    reloadAct->setShortcut(QString("Ctrl+R"));
    connect(reloadAct, SIGNAL(triggered()), this, SLOT(reloadAll()));
    fileMenu->addAction(reloadAct);
    
    fileMenu->addSeparator();
    
    QAction* quitAct = new QAction("&Quit", this);
    quitAct->setShortcut(QString("Ctrl+Q"));
    connect(quitAct, SIGNAL(triggered()), this, SLOT(close()));
    fileMenu->addAction(quitAct);
    
    /*** VIEW ***/
    QMenu* viewMenu = menuBar()->addMenu(tr("&View"));
    
    QAction* channelAct = dock->toggleViewAction();
    channelAct->setShortcut(QString("F9"));
    viewMenu->addAction(channelAct);
    
    QAction* goAct = fChannelWidget->toggleGOAction();
    goAct->setShortcut(QString("F10"));
    viewMenu->addAction(goAct);
    
    viewMenu->addSeparator();
    
    QAction* fitXYAct = new QAction(tr("Fit x/y"), this);
    fitXYAct->setShortcut(QString("e"));
    connect(fitXYAct, SIGNAL(triggered()), fVCWidget,
        SLOT(scaleToFit()));
    viewMenu->addAction(fitXYAct);
    
    QAction* fitXAct = new QAction(tr("Fit x"), this);
    fitXAct->setShortcut(QString("x"));
    connect(fitXAct, SIGNAL(triggered()), fVCWidget,
        SLOT(scaleXToFit()));
    viewMenu->addAction(fitXAct);
    
    QAction* fitYAct = new QAction(tr("Fit y"), this);
    fitYAct->setShortcut(QString("y"));
    connect(fitYAct, SIGNAL(triggered()), fVCWidget,
        SLOT(scaleYToFit()));
    viewMenu->addAction(fitYAct);
    
    /*** DATA ***/
    fDataMenu = menuBar()->addMenu(tr("&Data"));
    updateDataMenu();
    
    fDataMapper = new QSignalMapper(this);
    connect(fDataMapper, SIGNAL(mapped(QObject*)), this, SLOT(closeFile(QObject*)));
}

MainWindow::~MainWindow()
{
    for(std::list<File*>::iterator file = fFiles.begin();
        file != fFiles.end(); file++)
    {
        delete *file;
    }
}

void MainWindow::open()
{
    bool firstFile = fFiles.empty();
    
    QStringList filenames = QFileDialog::getOpenFileNames(this, tr("Open data file"),
        "", tr("Data files (*.dat);;All files (*)"));
    
    bool loadedAny = false;
    for(QStringList::const_iterator fname = filenames.begin();
        fname != filenames.end(); fname++) {
        if(loadFile(*fname)) loadedAny = true;
    }
    
    if(loadedAny) {
        updateDataMenu();
        if(firstFile) scaleToFit();
    }
    
    showMessages();
}

bool MainWindow::loadFile(const QString& fname)
{
    File* file = new File(fname.toStdString().c_str());
    if(file->fail()) {
        fErrorFail = true;
        fErrorStr += file->errorstr().c_str();
        delete file;
        return false;
    }
    
    fFiles.push_back(file);
    updateDataMenu();
    fChannelWidget->addFile(*file);
    
    return true;
}

void MainWindow::showMessages()
{
    if(fErrorFail) {
        QMessageBox::critical(this, tr("Error"), fErrorStr);
    } else if(!fErrorStr.isEmpty()) {
        QMessageBox::information(this, tr("Warning"), fErrorStr);
    }
    
    fErrorStr.clear();
    fErrorFail = false;
}

void MainWindow::updateDataMenu()
{
    fDataMenu->clear();
    
    QAction* closeAllAct = new QAction(tr("Close All"), this);
    connect(closeAllAct, SIGNAL(triggered()), this, SLOT(closeAllFiles()));
    closeAllAct->setEnabled(!fFiles.empty());
    fDataMenu->addAction(closeAllAct);
    
    if(!fFiles.empty())
        fDataMenu->addSeparator();
    
    for(std::list<File*>::iterator file = fFiles.begin();
        file != fFiles.end(); file++)
    {
        QAction* act = fDataMenu->addAction(tr("Close ") + (*file)->basename().c_str());
        fDataMapper->setMapping(act, (QObject*)*file);
        connect(act, SIGNAL(triggered()), fDataMapper, SLOT(map()));
    }
}

void MainWindow::reloadAll()
{
    for(std::list<File*>::iterator file = fFiles.begin();
        file != fFiles.end(); file++)
    {
        std::vector<Channel> oldChannels = (*file)->channels();
        File::reload_result_t result = (*file)->reload();
        fErrorStr += (*file)->errorstr().c_str();
        if(result == File::RELOAD_FAIL) {
            fErrorFail = true;
        } else if(result == File::RELOAD_LAYOUT_CHANGED) {
            fChannelWidget->reloadFile(*(*file), oldChannels);
        }
    }
    fChannelWidget->updateChannelNames();
    fVCWidget->chParamsChanged();
    showMessages();
}

void MainWindow::closeFile(QObject* ffile)
{
    File* file = (File*) ffile;
    fFiles.remove(file);
    updateDataMenu();
    fChannelWidget->removeFile(*file);
    delete file;
}

void MainWindow::closeAllFiles()
{
    for(std::list<File*>::iterator file = fFiles.begin();
        file != fFiles.end(); file++)
    {
        delete *file;
    }
    fFiles.clear();
    updateDataMenu();
    fChannelWidget->removeAllFiles();
}

