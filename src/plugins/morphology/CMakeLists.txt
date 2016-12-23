include_directories(
    ${PCL_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
)

QT5_ADD_RESOURCES(morphology_RESOURCES_RCC morphology.qrc)
FILE (GLOB plugin_SOURCE *.cpp *.cxx *.cc *.C *.c *.h *.hpp)
add_library(morphology SHARED ${plugin_SOURCE} ${morphology_RESOURCES_RCC} ../../pluginsystem/iplugin.h)
target_link_libraries(
    morphology
    cloudclean_util
    cloudclean_pluginsystem
    cloudclean_model
    cloudclean_gui
    ${PCL_LIBRARIES}
)


QT5_USE_MODULES(morphology Core Gui OpenGL)

INSTALL(TARGETS morphology DESTINATION ${PLUGIN_INSTALL} COMPONENT CloudClean_plugins)
