/*
 * Software License Agreement (BSD License)
 *
 *  CloudClean
 *  Copyright (c) 2013, Rickert Mulder
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Rickert Mulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "pluginsystem/pluginmanager.h"
#include <cassert>
#include <QString>
#include <QJsonObject>
#include <QJsonArray>
#include <QApplication>
#include <QPluginLoader>
#include <QDebug>
#include <QDir>
#include <QUndoStack>
#include <QFileSystemWatcher>
#include <QTimer>
#include <gui/mainwindow.h>
#include <model/layerlist.h>
#include <model/cloudlist.h>

#include "pluginsystem/core.h"
#include "pluginsystem/plugindeps.h"

static QString DLLExtension() {
#if defined(Q_OS_WIN)
    return QString("dll");
#elif defined(Q_OS_MAC)
    return QString("dylib");
#else
    return QString("so");
#endif
    assert(0 && "Unknown Operative System. Please Define the appropriate dynamic library extension");
    return QString();
}

PluginResource::PluginResource(QString path, Core * core, PluginManager * pm) {
    watcher_ = nullptr;
    loader_ = nullptr;
    instance_ = nullptr;
    timer_ = nullptr;

    load(path);

    timer_ = new QTimer(0);
    timer_->setInterval(1000);
    timer_->setTimerType(Qt::VeryCoarseTimer);

    watcher_ = new QFileSystemWatcher(0);
    watcher_->addPath(path);

    watcher_->connect(watcher_, &QFileSystemWatcher::fileChanged, [this, core, pm] (QString path) {
        timer_->connect(timer_, &QTimer::timeout, [this, core, pm, path] () {
            timer_->disconnect();
            timer_->stop();

            if(instance_ != nullptr) {
                instance_->cleanup();
                delete instance_;
            }
            instance_ = nullptr;

            if(loader_ != nullptr) {
                watcher_->removePath(loader_->fileName());
                loader_->unload();
                if(loader_->isLoaded()) {
                    qDebug() << "Could not unload plugin: " << path;
                }
                delete loader_;
            }
            loader_ = nullptr;

            if(load(path)) {
                qDebug() << "Reloaded plugin: " << instance_->getName();
                instance_->initialize(core);
                instance_->initialize2(pm);
            }
            if(QFile::exists(path)){
                watcher_->addPath(path);
            }
        });
        timer_->start();
    });
}

PluginResource::~PluginResource(){
    if(watcher_ != nullptr)
        delete watcher_;
    if(loader_ != nullptr)
        delete loader_;
    if(timer_ != nullptr)
        delete timer_;
    // Instance is deleted when unloaded?
}

bool PluginResource::load(QString path) {
    loader_ = new QPluginLoader();
    loader_->setFileName(path);
    QJsonObject meta = loader_->metaData().find("MetaData").value().toObject();
    QJsonObject::Iterator export_symbols = meta.find("export_symbols");
    bool found = export_symbols != meta.end();

    if(found && export_symbols.value().toBool()){
        loader_->setLoadHints(QLibrary::ExportExternalSymbolsHint);
        //qDebug() << "Exporting";
    }

    bool loaded = loader_->load();

    if (!loaded) {
        qDebug() << "Could not load plugin: " << path;
        qDebug() << "ERROR: " << loader_->errorString();
        return false;
    }

    QObject *plugin = loader_->instance();

    instance_ = qobject_cast<IPlugin *>(plugin);
    if(!instance_){
        qDebug() << "Not a valid plugin";
        return false;
    }
    return true;
}

PluginManager::PluginManager(Core * core) {
    // Add locations to load dynamic libraries from
    qApp->addLibraryPath(qApp->applicationDirPath());
    core_ = core;

    watcher_ = nullptr;
    plugins_loaded_ = false;
    timer_ = nullptr;

    // Look for plugin directory
    plugins_dir_.reset(new QDir(qApp->applicationDirPath()));
    bool succ = false;
    if (!succ)
        succ = plugins_dir_->cd("plugins");
    if (!succ)
        succ = plugins_dir_->cd("../plugins");
    if (!succ)
        succ = plugins_dir_->cd("../lib/plugins");
    if (!succ)
        succ = plugins_dir_->cd("../lib");
    if (!succ)
        succ = plugins_dir_->cd("/usr/lib/cloudclean/plugins");
    if (!succ){
		plugins_dir_.reset();
        qDebug("Plugins directory not found!");
		return;
    } else {
        qDebug() << "Plugins dir:" << plugins_dir_->absolutePath();
    }

    qApp->addLibraryPath(plugins_dir_->absolutePath());

    timer_ = new QTimer(0);
    timer_->setInterval(1000);
    timer_->setTimerType(Qt::VeryCoarseTimer);
}

PluginManager::~PluginManager() {
    if(watcher_ != nullptr)
        delete watcher_;
    if(timer_ != nullptr)
        delete timer_;
}

IPlugin * PluginManager::findPluginByName(QString name) {
    for(PluginResource * plugin_resource: plugins_) {
        if(plugin_resource != nullptr && plugin_resource->instance_->getName() == name)
            return plugin_resource->instance_;
    }
    return nullptr;
}

bool PluginManager::unloadPlugin(IPlugin * plugin){
    qDebug() << "Unloading: " << plugin->getName();
    for(uint idx = 0; idx < plugins_.size(); idx++) {
        PluginResource * plugin_resource = plugins_[idx];
        if(plugin_resource->instance_ == plugin) {
            plugin_resource->instance_->cleanup();
            plugin_resource->loader_->unload();
            delete plugin_resource;
            plugins_.erase(plugins_.begin() + idx);
            return true;
        }
    }

    qDebug() << "Could not find plugin!";
    return false;
}

QString PluginManager::getFileName(IPlugin * plugin){
    for(PluginResource * pr : plugins_){
        if(pr->instance_ == plugin){
            return pr->loader_->fileName();
        }
    }
    return "";
}

IPlugin * PluginManager::loadPlugin(PluginMetaData & meta, std::map<QString, PluginMetaData> & dep_graph){

    // Load all dependencies
    for(QString dep :meta.deps_) {
        auto dep_it = dep_graph.find(dep);
        if(dep_it == dep_graph.end()){
            qDebug() << "Cannot load " << meta.name_ << ",dependency " << dep << "cannot be found";
            return nullptr;
        }

        PluginMetaData & dep_meta = (*dep_it).second;
        if(dep_meta.failed_) {
            qDebug() << "Cannot load " << meta.name_ << ", dependency " << dep << "failed to load";
            return nullptr;
        }
        if(dep_meta.loaded_)
            continue;
        if(loadPlugin(dep_meta, dep_graph) != nullptr)
            continue;

        //qDebug() << "Loaded: " << dep << " (dependency of " << meta.name_ << ")";
    }

    PluginResource * pr = new PluginResource(meta.path_, core_, this);
    if(pr->instance_ == nullptr) {
        qDebug() << "Failed to load: " << meta.path_;
        return nullptr;
    }

    plugins_.push_back(pr);
    meta.loaded_ = true;
    return pr->instance_;
}

PluginMetaData::PluginMetaData(){
    name_ = "";
    path_ = "";
    loaded_ = false;
    failed_ = true;
}

PluginMetaData::PluginMetaData(QString path){
    path_ = path;
    loaded_ = false;
    failed_ = false;
    QPluginLoader loader;
    loader.setFileName(path);
    QJsonObject meta = loader.metaData().find("MetaData").value().toObject();

    QJsonObject::Iterator name_it = meta.find("name");
    name_ = name_it.value().toString();

    QJsonObject::Iterator dep_it = meta.find("dependencies");
    bool has_deps = dep_it != meta.end();
    if(!has_deps)
        return;

    QJsonArray dep_arr = dep_it.value().toArray();

    //qDebug() << "Deps for " << name_ << " " << dep_arr;

    for(QJsonValueRef valref : dep_arr){
        QString name = valref.toObject().find("name").value().toString();
        deps_.push_back(name);
    }
}

void PluginManager::updateDepGraph() {
    // Update dependency graph
    for (QString fileName : plugins_dir_->entryList(QDir::Files)) {
        QString absfilepath = plugins_dir_->absoluteFilePath(fileName);
        PluginMetaData meta(absfilepath);

        // qDebug() << "Found: " << absfilepath;
        // qDebug() << "Name: " << meta.name_;

        // If meta data does not exists
        if(dep_graph_.find(meta.name_) == dep_graph_.end())
            dep_graph_[meta.name_] = meta;
    }
}

void PluginManager::loadPlugins() {
    if(!plugins_dir_ || plugins_loaded_)
		return;

    QStringList pluginfilters("*." + DLLExtension());
    plugins_dir_->setNameFilters(pluginfilters);

    updateDepGraph();

    // Load plugins
    for (auto node : dep_graph_) {
        PluginMetaData & meta = node.second;
        if(meta.loaded_ != false)
            continue;
        if(loadPlugin(meta, dep_graph_) == nullptr) {
            qDebug() << "Failed to load " << meta.name_;
            continue;
        }

        qDebug() << "Loaded " << meta.name_;
    }

    plugins_loaded_ = true;

    // Setup watcher to look for updates
    watcher_ = new QFileSystemWatcher();
    watcher_->addPath(plugins_dir_->path());
    watcher_->connect(watcher_, &QFileSystemWatcher::directoryChanged, [this] (QString path) {
        // Delay so compilation can complete
        timer_->connect(timer_, &QTimer::timeout, [this, path] () {
            timer_->stop();

            updateDepGraph();

            for (QString fileName : plugins_dir_->entryList(QDir::Files)) {
                QString absfilepath = plugins_dir_->absoluteFilePath(fileName);
                bool already_loaded = false;
                // Make sure the plugin is not already loaded
                for(PluginResource * pr : plugins_){
                    if(pr->loader_ == nullptr)
                        continue;
                    if(pr->loader_->fileName() == absfilepath){
                        already_loaded = true;
                        break;
                    }
                }

                if(already_loaded)
                    continue;

                PluginMetaData meta(absfilepath);
                qDebug() << "Detected new plugin: " << meta.name_;
                IPlugin * instance = loadPlugin(meta, dep_graph_);

                if(instance != nullptr) {
                    instance->initialize(core_);
                    instance->initialize2(this);
                    qDebug() << "Loaded new plugin " << meta.name_;
                }
            }
        });
        timer_->start();
    });
}

void PluginManager::initializePlugins() {
    for(PluginResource * pr : plugins_){
        // qDebug() << "init1: " << pr->instance_->getName();
        pr->instance_->initialize(core_);
    }

    for(PluginResource * pr : plugins_){
        // qDebug() << "init2: " << pr->instance_->getName();
        pr->instance_->initialize2(this);
    }
}


