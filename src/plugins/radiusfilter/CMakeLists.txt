include_directories(
    ${PCL_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
)

QT5_ADD_RESOURCES(radiusfilter_RESOURCES_RCC radiusfilter.qrc)
FILE (GLOB plugin_SOURCE *.cpp *.cxx *.cc *.C *.c *.h *.hpp)
add_library(radiusfilter SHARED ${plugin_SOURCE} ${radiusfilter_RESOURCES_RCC} ../../pluginsystem/iplugin.h)
target_link_libraries(
    radiusfilter
    cloudclean_util
    cloudclean_pluginsystem
    cloudclean_model
    cloudclean_gui
    ${PCL_LIBRARIES}
)


QT5_USE_MODULES(radiusfilter Core Gui OpenGL)

INSTALL(TARGETS radiusfilter DESTINATION ${PLUGIN_INSTALL} COMPONENT CloudClean_plugins)
