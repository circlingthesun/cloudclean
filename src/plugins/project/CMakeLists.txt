add_definitions(-Dproject_EXPORTS)
include_directories(
    ${PCL_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
)

QT5_ADD_RESOURCES(project_RESOURCES_RCC project.qrc)
FILE (GLOB plugin_SOURCE *.cpp *.cxx *.cc *.C *.c *.h *.hpp)
add_library(project SHARED ${plugin_SOURCE} ${project_RESOURCES_RCC} ../../pluginsystem/iplugin.h)
target_link_libraries(
    project
    cloudclean_util
    cloudclean_pluginsystem
    cloudclean_model
    cloudclean_gui
    ${PCL_LIBRARIES}
)


QT5_USE_MODULES(project Core Gui OpenGL)

INSTALL(TARGETS project DESTINATION ${PLUGIN_INSTALL} COMPONENT CloudClean_plugins)
