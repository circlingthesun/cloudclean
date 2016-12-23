include_directories(
    ${PCL_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
)

QT5_ADD_RESOURCES(register_RESOURCES_RCC register.qrc)
FILE (GLOB plugin_SOURCE *.cpp *.cxx *.cc *.C *.c *.h *.hpp)
add_library(register SHARED ${plugin_SOURCE} ${register_RESOURCES_RCC} ../../pluginsystem/iplugin.h)
target_link_libraries(
    register
    cloudclean_util
    cloudclean_pluginsystem
    cloudclean_model
    cloudclean_gui
    ${PCL_LIBRARIES}
)


QT5_USE_MODULES(register Core Gui OpenGL)

INSTALL(TARGETS register DESTINATION ${PLUGIN_INSTALL} COMPONENT CloudClean_plugins)
