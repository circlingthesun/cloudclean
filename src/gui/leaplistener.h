#ifndef LEAPLISTENER_H
#define LEAPLISTENER_H

#include "Leap.h"

class GLWidget;

class LeapListener : public Leap::Listener
{
 public:
    LeapListener(GLWidget * glwidget);
    virtual void onInit(const Leap::Controller&);
    virtual void onConnect(const Leap::Controller&);
    virtual void onDisconnect(const Leap::Controller&);
    virtual void onExit(const Leap::Controller&);
    virtual void onFrame(const Leap::Controller&);
    virtual void onFocusGained(const Leap::Controller&);
    virtual void onFocusLost(const Leap::Controller&);
    virtual void onDeviceChange(const Leap::Controller&);
    virtual void onServiceConnect(const Leap::Controller&);
    virtual void onServiceDisconnect(const Leap::Controller&);

 private:
    GLWidget * glwidget_;
    bool disabled_;
};


#endif // LEAPLISTENER_H
