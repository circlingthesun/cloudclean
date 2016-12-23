include_directories(
    ${PCL_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
)

FILE (GLOB plugin_SOURCE *.cpp *.cxx *.cc *.C *.c *.h *.hpp)
QT5_ADD_RESOURCES(brush3d_RESOURCES_RCC brush3d_resources.qrc)
add_library(brush3d SHARED ${plugin_SOURCE} ${brush3d_RESOURCES_RCC} ../../pluginsystem/iplugin.h)
target_link_libraries(brush3d
    cloudclean_util
    cloudclean_pluginsystem
    cloudclean_model
    cloudclean_gui
)

if(WIN32)
    target_link_libraries(brush3d ${GLEW_LIBRARIES})
endif()

QT5_USE_MODULES(brush3d Core Gui OpenGL)

INSTALL(TARGETS brush3d DESTINATION ${PLUGIN_INSTALL} COMPONENT CloudClean_plugins)
