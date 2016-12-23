include_directories(
    ${PCL_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
)

QT5_ADD_RESOURCES(outlierfilter_RESOURCES_RCC outlierfilter.qrc)
FILE (GLOB plugin_SOURCE *.cpp *.cxx *.cc *.C *.c *.h *.hpp)
add_library(outlierfilter SHARED ${plugin_SOURCE} ${outlierfilter_RESOURCES_RCC} ../../pluginsystem/iplugin.h)
target_link_libraries(
    outlierfilter
    cloudclean_util
    cloudclean_pluginsystem
    cloudclean_model
    cloudclean_gui
    ${PCL_LIBRARIES}
)


QT5_USE_MODULES(outlierfilter Core Gui OpenGL)

INSTALL(TARGETS outlierfilter DESTINATION ${PLUGIN_INSTALL} COMPONENT CloudClean_plugins)
