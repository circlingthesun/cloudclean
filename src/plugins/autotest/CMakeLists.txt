include_directories(
    ${PCL_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
)

QT5_ADD_RESOURCES(autotest_RESOURCES_RCC autotest.qrc)
FILE (GLOB plugin_SOURCE *.cpp *.cxx *.cc *.C *.c *.h *.hpp)
add_library(autotest SHARED ${plugin_SOURCE} ${autotest_RESOURCES_RCC} ../../pluginsystem/iplugin.h)
target_link_libraries(
    autotest
    cloudclean_util
    cloudclean_pluginsystem
    cloudclean_model
    cloudclean_gui
    project
    featureeval
    ${PCL_LIBRARIES}
)


QT5_USE_MODULES(autotest Core Gui OpenGL)

INSTALL(TARGETS autotest DESTINATION ${PLUGIN_INSTALL} COMPONENT CloudClean_plugins)
