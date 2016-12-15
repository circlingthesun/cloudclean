// <Copyright Holder>. Copyright (C) <Copyright Year(s)>. <License>.
#ifndef HEADER_SRC_APP_H_INCLUDED
#define HEADER_SRC_APP_H_INCLUDED

#include "glheaders.h"
#include <QtCore>
#include <QtGui>
#include <QApplication>

#include <boost/shared_ptr.hpp>

#include "gui/layerlistview.h"
#include "gui/cloudlistview.h"
#include "gui/gldata.h"
#include "pluginsystem/core.h"

class PluginManager;
class QProgressBar;
class QStatusBar;
class QTabWidget;
class QUndoStack;
class ActionManager;

class MainWindow;
class GLWidget;
class FlatView;
class LayerList;
class CloudList;

class App : public QApplication
{
        Q_OBJECT
    public:
        App(int& argc, char** argv);
        ~App();
        
        App* INSTANCE();

        QString getProjectName();
        QString getProjectCodeName();
        QString getProjectVendorID();
        QString getProjectVendorName();
        QString getProjectID();
        int getProjectMajorVersion();
        int getProjectMinorVersion();
        int getProjectPatchVersion();
        QString getProjectVersion();
        QString getProjectCopyrightYears();
        QString getProjectInvocation();
        bool notify(QObject *receiver, QEvent *event);

    private:
        void printHelpMessage();
        void printVersionMessage();
        void printVersionTripletMessage();
        void printApplicationIdentifier();
        void setPreference(const std::string& key, const std::string& val);
        void unsetPreference(const std::string& key);
        void printPreference(const std::string& key)const;
        void printAllPreferences()const;
        std::string getKeyName(const std::string& key)const;
        std::string getKeyRepr(const std::string& key)const;
        std::string convert(const QString& str)const;
        QString convert(const std::string& str)const;
        
        static App* _instance;
        QString _invocation;

        PluginManager * pm_;
        Core * core_;

};

#endif
