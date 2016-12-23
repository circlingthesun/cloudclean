include_directories(
    ${PCL_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
)

QT5_ADD_RESOURCES(accuracy_RESOURCES_RCC accuracy.qrc)
FILE (GLOB plugin_SOURCE *.cpp *.cxx *.cc *.C *.c *.h *.hpp)
add_library(accuracy SHARED ${plugin_SOURCE} ${accuracy_RESOURCES_RCC} ../../pluginsystem/iplugin.h)
target_link_libraries(
    accuracy
    cloudclean_util
    cloudclean_pluginsystem
    cloudclean_model
    cloudclean_gui
    ${PCL_LIBRARIES}
)


QT5_USE_MODULES(accuracy Core Gui OpenGL)

INSTALL(TARGETS accuracy DESTINATION ${PLUGIN_INSTALL} COMPONENT CloudClean_plugins)
