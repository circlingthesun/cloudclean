#ifndef IPLUGIN_H
#define IPLUGIN_H

#include <QObject>
#include <QtPlugin>
#include "pluginsystem/export.h"

class Core;
class PluginManager;

// So plugins are created with the default constructor
// This cannot take any parameters so use initialise method
// Plugins get to register its slots with signals from other plugins
// or the main app
// How do i stop two plugins from being active at once?
// When a plugin goes active it should emit a signal
// This signal should
// Plugins get to hook into 2d or 3d draw
// Plugins get to listen for mouse events in the 2d or 3d view

class IPlugin: public QObject  {
    Q_OBJECT
 public:
    virtual QString getName() = 0;
    virtual void initialize(Core * core) = 0;
    virtual void initialize2(PluginManager * pm) {}
    virtual void cleanup() = 0;
};

Q_DECLARE_INTERFACE(IPlugin, "za.co.circlingthesun.cloudclean.iplugin")

#endif // IPLUGIN_H
