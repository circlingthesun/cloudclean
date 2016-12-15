#ifndef PLUGINMANAGER_H
#define PLUGINMANAGER_H

#include <map>
#include <string>
#include <vector>
#include <memory>
#include <QObject>
#include "pluginsystem/export.h"

class QMouseEvent;
class QWheelEvent;
class QKeyEvent;
class QUndoStack;
class QDir;
class ActionManager;
class MainWindow;
class Core;
class QPluginLoader;
class QFileSystemWatcher;
class QTimer;

#include "pluginsystem/core.h"
#include "pluginsystem/iplugin.h"

class PluginResource {
  public:
    PluginResource(QString path, Core *core, PluginManager *pm);
    ~PluginResource();

  private:
    bool load(QString path);

  public:
    QFileSystemWatcher * watcher_;
    QPluginLoader * loader_;
    IPlugin * instance_;

  private:
    QTimer * timer_;

};

class PluginMetaData {
 public:
    bool failed_;
    bool loaded_;
    QString name_;
    std::vector<QString> deps_;
    QString path_;
 public:
    PluginMetaData(QString path);
    PluginMetaData();
};

class PLUGINSYS_API PluginManager : public QObject{
    Q_OBJECT
 public:
    PluginManager(Core * core);
    ~PluginManager();

    std::map<QString, PluginMetaData> dep_graph_;
    IPlugin * findPluginByName(QString name);
    QString getFileName(IPlugin * plugin);
    IPlugin * loadPlugin(PluginMetaData &meta, std::map<QString, PluginMetaData> &dep_graph_);
    bool unloadPlugin(IPlugin * plugin);
    void updateDepGraph();
    void loadPlugins();
    void initializePlugins();

    template <typename T>
    T * findPlugin() {
        T * plugin = nullptr;
        for(PluginResource * pr : plugins_){
            plugin = qobject_cast<T *>(pr->instance_);
            if(plugin != nullptr)
                return plugin;
        }
        return nullptr;
    }

 signals:
    void endEdit();

 private:
    std::vector<PluginResource *> plugins_;
    Core * core_;
    std::unique_ptr<QDir> plugins_dir_;
    QFileSystemWatcher * watcher_;
    bool plugins_loaded_;
    QTimer * timer_;
};

#endif // PLUGINMANAGER_H
