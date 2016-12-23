include_directories(
    ${PCL_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
)

QT5_ADD_RESOURCES(compare_RESOURCES_RCC compare.qrc)
FILE (GLOB plugin_SOURCE *.cpp *.cxx *.cc *.C *.c *.h *.hpp)
add_library(compare SHARED ${plugin_SOURCE} ${compare_RESOURCES_RCC} ../../pluginsystem/iplugin.h)
target_link_libraries(
    compare
    cloudclean_util
    cloudclean_pluginsystem
    cloudclean_model
    cloudclean_gui
    ${PCL_LIBRARIES}
)


QT5_USE_MODULES(compare Core Gui OpenGL)

INSTALL(TARGETS compare DESTINATION ${PLUGIN_INSTALL} COMPONENT CloudClean_plugins)
