include_directories(
    ${PCL_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
)

QT5_ADD_RESOURCES(camerasetter_RESOURCES_RCC camerasetter.qrc)
FILE (GLOB plugin_SOURCE *.cpp *.cxx *.cc *.C *.c *.h *.hpp)
add_library(camerasetter SHARED ${plugin_SOURCE} ${camerasetter_RESOURCES_RCC} ../../pluginsystem/iplugin.h)
target_link_libraries(
    camerasetter
    cloudclean_util
    cloudclean_pluginsystem
    cloudclean_model
    cloudclean_gui
    ${PCL_LIBRARIES}
)


QT5_USE_MODULES(camerasetter Core Gui OpenGL)

INSTALL(TARGETS camerasetter DESTINATION ${PLUGIN_INSTALL} COMPONENT CloudClean_plugins)
