include_directories(
    ${PCL_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
)

QT5_ADD_RESOURCES(registration_RESOURCES_RCC registration.qrc)
FILE (GLOB plugin_SOURCE *.cpp *.cxx *.cc *.C *.c *.h *.hpp)
add_library(registration SHARED ${plugin_SOURCE} ${registration_RESOURCES_RCC} ../../pluginsystem/iplugin.h)
target_link_libraries(
    registration
    cloudclean_util
    cloudclean_pluginsystem
    cloudclean_model
    cloudclean_gui
    ${PCL_LIBRARIES}
)


QT5_USE_MODULES(registration Core Gui OpenGL)

INSTALL(TARGETS registration DESTINATION ${PLUGIN_INSTALL} COMPONENT CloudClean_plugins)
