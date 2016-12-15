#include "leaplistener.h"
#include "gui/glwidget.h"

LeapListener::LeapListener(GLWidget *glwidget){
    glwidget_ = glwidget;
    disabled_ = false;
}

void LeapListener::onInit(const Leap::Controller& controller) {
  std::cout << "Initialized" << std::endl;
}

void LeapListener::onConnect(const Leap::Controller& controller) {
  std::cout << "Connected";
  controller.enableGesture(Leap::Gesture::TYPE_CIRCLE);
  controller.enableGesture(Leap::Gesture::TYPE_KEY_TAP);
  controller.enableGesture(Leap::Gesture::TYPE_SCREEN_TAP);
  controller.enableGesture(Leap::Gesture::TYPE_SWIPE);
}

void LeapListener::onDisconnect(const Leap::Controller& controller) {
    //Note: not dispatched when running in a debugger.
    std::cout << "Disconnected";
}

void LeapListener::onExit(const Leap::Controller& controller) {
    std::cout << "Exited" << std::endl;
}

void LeapListener::onFrame(const Leap::Controller& controller) {
  // Get the most recent frame and report some basic information
  const Leap::Frame frame = controller.frame();
  /*std::cout << "Frame id: " << frame.id()
            << ", timestamp: " << frame.timestamp()
            << ", hands: " << frame.hands().count()
            << ", fingers: " << frame.fingers().count()
            << ", tools: " << frame.tools().count()
            << ", gestures: " << frame.gestures().count();
  */

  if (!frame.hands().isEmpty()) {
    // Get the first hand
    const Leap::Hand hand = frame.hands()[0];


    qDebug() << "Radius: " << hand.sphereRadius();

    // Grab
    if(hand.sphereRadius() < 50.0 && hand.translationProbability(controller.frame(1)) > 0.6){
        Leap::Vector v = hand.translation(controller.frame(1));
        v = 0.1 * v;
        glwidget_->camera_.translate(v.x, v.y, v.z);
        glwidget_->update();
    }

    qDebug() << "Hand pos" << hand.palmPosition().x << hand.palmPosition().y << hand.palmPosition().z;

    if(frame.fingers().count() > 200 && frame.hands().count() == 1){
        Leap::Vector trans = hand.translation(controller.frame(1));

        //int dir = trans.y > 0 ? 1 : -1;
        trans = 0.1 * trans;
        //glwidget_->camera_.translate(0, 0, dir*0.1);

        //Leap::Vector rot = hand.rotationAxis(controller.frame());

        float yaw = hand.rotationAngle(controller.frame(1), Leap::Vector(1, 0, 0));
        float pitch = hand.rotationAngle(controller.frame(1), Leap::Vector(1, 0, 0)); // works
        float roll = hand.rotationAngle(controller.frame(1), Leap::Vector(0, 0, 1));

        //qDebug() << yaw <<  pitch <<  roll;

        glwidget_->camera_.rotate3D(yaw, pitch, roll);

        //glwidget_->camera_.rotate2D(0.01 * trans.x, 0.01 * trans.y);

        glwidget_->update();
    }

/*
    // Check if the hand has any fingers
    const Leap::FingerList fingers = hand.fingers();
    if (!fingers.empty()) {
      // Calculate the hand's average finger tip position
      Leap::Vector avgPos;
      for (int i = 0; i < fingers.count(); ++i) {
        avgPos += fingers[i].tipPosition();
      }
      avgPos /= (float)fingers.count();
      std::cout << "Hand has " << fingers.count()
                << " fingers, average finger tip position" << avgPos<< std::endl;
    }

    // Get the hand's sphere radius and palm position
    std::cout << "Hand sphere radius: " << hand.sphereRadius()
              << " mm, palm position: " << hand.palmPosition() << std::endl;

    // Get the hand's normal vector and direction
    const Leap::Vector normal = hand.palmNormal();
    const Leap::Vector direction = hand.direction();

    // Calculate the hand's pitch, roll, and yaw angles
    std::cout << "Hand pitch: " << direction.pitch() * Leap::RAD_TO_DEG << " degrees, "
              << "roll: " << normal.roll() * Leap::RAD_TO_DEG << " degrees, "
              << "yaw: " << direction.yaw() * Leap::RAD_TO_DEG << " degrees"<< std::endl;
  }

  // Get gestures
  const Leap::GestureList gestures = frame.gestures();
  for (int g = 0; g < gestures.count(); ++g) {
    Leap::Gesture gesture = gestures[g];

    switch (gesture.type()) {
      case Leap::Gesture::TYPE_CIRCLE:
      {
        Leap::CircleGesture circle = gesture;
        std::string clockwiseness;

        if (circle.pointable().direction().angleTo(circle.normal()) <= M_PI/4) {
          clockwiseness = "clockwise";
        } else {
          clockwiseness = "counterclockwise";
        }

        // Calculate angle swept since last frame
        float sweptAngle = 0;
        if (circle.state() != Leap::Gesture::STATE_START) {
          Leap::CircleGesture previousUpdate = Leap::CircleGesture(controller.frame().gesture(circle.id()));
          sweptAngle = (circle.progress() - previousUpdate.progress()) * 2 * M_PI;
        }
        std::cout << "Circle id: " << gesture.id()
                  << ", state: " << gesture.state()
                  << ", progress: " << circle.progress()
                  << ", radius: " << circle.radius()
                  << ", angle " << sweptAngle * Leap::RAD_TO_DEG
                  <<  ", " << clockwiseness << std::endl;
        break;
      }
      case Leap::Gesture::TYPE_SWIPE:
      {
        Leap::SwipeGesture swipe = gesture;
        std::cout << "Swipe id: " << gesture.id()
          << ", state: " << gesture.state()
          << ", direction: " << swipe.direction()
          << ", speed: " << swipe.speed() << std::endl;
        break;
      }
      case Leap::Gesture::TYPE_KEY_TAP:
      {
        Leap::KeyTapGesture tap = gesture;
        std::cout << "Key Tap id: " << gesture.id()
          << ", state: " << gesture.state()
          << ", position: " << tap.position()
          << ", direction: " << tap.direction()<< std::endl;
        break;
      }
      case Leap::Gesture::TYPE_SCREEN_TAP:
      {
        Leap::ScreenTapGesture screentap = gesture;
        std::cout << "Screen Tap id: " << gesture.id()
        << ", state: " << gesture.state()
        << ", position: " << screentap.position()
        << ", direction: " << screentap.direction()<< std::endl;
        break;
      }
      default:
        std::cout << "Unknown gesture type.";
        break;
    }
    */
  }

}

void LeapListener::onFocusGained(const Leap::Controller& controller) {
    //std::cout << "Focus Gained";
}

void LeapListener::onFocusLost(const Leap::Controller& controller) {
    //std::cout << "Focus Lost";
}

void LeapListener::onDeviceChange(const Leap::Controller& controller) {
  std::cout << "Device Changed" << std::endl;
  const Leap::DeviceList devices = controller.devices();

  for (int i = 0; i < devices.count(); ++i) {
    std::cout << "id: " << devices[i].toString() << std::endl;
    std::cout << "  isStreaming: " << (devices[i].isStreaming() ? "true" : "false") << std::endl;
  }
}

void LeapListener::onServiceConnect(const Leap::Controller& controller) {
  std::cout << "Service Connected" << std::endl;
}

void LeapListener::onServiceDisconnect(const Leap::Controller& controller) {
  std::cout << "Service Disconnected" << std::endl;
}
