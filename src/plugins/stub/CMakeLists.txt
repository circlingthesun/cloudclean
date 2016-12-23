include_directories(
    ${PCL_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
)

QT5_ADD_RESOURCES(stub_RESOURCES_RCC stub_resources.qrc)
FILE (GLOB plugin_SOURCE *.cpp *.cxx *.cc *.C *.c *.h *.hpp)
add_library(stub SHARED ${plugin_SOURCE} ${stub_RESOURCES_RCC} ../../pluginsystem/iplugin.h)
target_link_libraries(stub
    cloudclean_util
    cloudclean_pluginsystem
    cloudclean_model
    cloudclean_gui
    ${PCL_LIBRARIES}
)


QT5_USE_MODULES(stub Core Gui OpenGL)

INSTALL(TARGETS stub DESTINATION ${PLUGIN_INSTALL} COMPONENT CloudClean_plugins)
